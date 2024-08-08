/***************************************************************************
 # Copyright (c) 2015-21, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
import LoadShadingData;
import Scene.RaytracingInline;
import Rendering.Materials.StandardMaterial;
// import Rendering.Materials.Microfacet;
import Rendering.Materials.IsotropicGGX;
import Rendering.ScreenSpaceReSTIR.ScreenSpaceReSTIR;
import Utils.Sampling.TinyUniformSampleGenerator;

RWTexture2D<float4> gColor;
RWTexture2D<float4> gEmission;
RWTexture2D<float4> gDiffuseIllumination;
RWTexture2D<float4> gDiffuseReflectance;
RWTexture2D<float4> gSpecularIllumination;
RWTexture2D<float4> gSpecularReflectance;
RWTexture2D<float4> gDebug;

#define is_valid(name) (is_valid_##name != 0)

/** Perform shading with final samples from ReSTIR.
*/
struct FinalShading
{
    // Static configuration.
    static const bool kUseEnvBackground = USE_ENV_BACKGROUND;

    Texture2D<PackedHitInfo> vbuffer;

    ScreenSpaceReSTIR screenSpaceReSTIR;
    StandardMaterial standardMaterial;

    uint2 frameDim;

    uint numReSTIRInstances;
    uint ReSTIRInstanceID;

    void execute(uint2 pixel)
    {
        if (any(pixel >= frameDim)) return;

        float3 color = {};
        float3 emission = {};
        float3 diffuseReflectance = {};
        float3 diffuseIllumination = {};
        float3 specularReflectance = {};
        float3 specularIllumination = {};
        float hitT = 10000.f; // TODO: Remove magic number also used in SpatioTemporalResampling pass.

        ShadingData sd;
        let lod = ExplicitLodTextureSampler(0.f); // TODO: Implement texture level-of-detail.
        if (loadShadingData(pixel, frameDim, gScene.camera, vbuffer, lod, sd))
        {
            // Create material instance and query its properties.
            let hints = getMaterialInstanceHints();
            let mi = gScene.materials.getMaterialInstance(sd, lod, hints);
            let bsdfProperties = mi.getProperties(sd);

            // Get final sample from ReSTIR.
            float3 dir;
            float distance;
            float3 Li;
            bool valid = screenSpaceReSTIR.getFinalSample(pixel, dir, distance, Li);

            // Check light visibility.
            if (valid)
            {
                // Create a DXR 1.1 query object to trace a ray (the <1> means use alpha testing)
                SceneRayQuery<1> rayQuery;
                const Ray ray = Ray(sd.computeRayOrigin(), dir, 0.f, distance);
                if (!rayQuery.traceVisibilityRay(ray, RAY_FLAG_NONE, 0xff))
                {
                    valid = false;
                }

                //sd.setActiveLobes((uint)LobeType::DiffuseReflection);
                //diffuse = standardMaterial.eval(sd, dir) * Li;

                //sd.setActiveLobes((uint)LobeType::SpecularReflection);
                //specular = standardMaterial.eval(sd, dir) * Li;

                //hitT = distance;
            }

            // Evaluate diffuse and specular lobes.
            float3 diffuse = {};
            float3 specular = {};
            if (valid) {
                TinyUniformSampleGenerator sg = TinyUniformSampleGenerator(pixel, gScreenSpaceReSTIR.frameIndex);

                sd.mtl.setActiveLobes((uint)LobeType::DiffuseReflection);
                diffuse = mi.eval(sd, dir, sg) * Li;

                sd.mtl.setActiveLobes((uint)LobeType::SpecularReflection);
                specular = mi.eval(sd, dir, sg) * Li;

                hitT = distance;
            }

            //emission = sd.emissive;
            emission = bsdfProperties.emission;
            color = diffuse + specular + emission;  // should have been an = instead of +=, but shouldn't matter

            // Demodulate diffuse reflectance (albedo) from diffuse color.
            diffuseReflectance = bsdfProperties.diffuseReflectionAlbedo;
            float3 diffuseFactor = select(diffuseReflectance <= 0.f, 0.f, 1.f / diffuseReflectance);
            diffuseIllumination = diffuse * diffuseFactor;

            // Demodulate preintegrated specular reflectance from specular color.
            float NdotV = saturate(dot(bsdfProperties.guideNormal, sd.V));
            float ggxAlpha = bsdfProperties.roughness * bsdfProperties.roughness;
            specularReflectance = approxSpecularIntegralGGX(bsdfProperties.specularReflectance, ggxAlpha, NdotV);
            float3 specularFactor = select(specularReflectance <= 0.f, 0.f, 1.f / specularReflectance);
            specularIllumination = specular * specularFactor;
        }
        else
        {
            //if (kUseEnvBackground)
            //{
            //    float3 dir = gScene.camera.computeRayPinhole(pixel, frameDim).dir;
            //    emission = gScene.envMap.eval(dir);
            //    color = emission;
            //}
        }

        // Write active outputs.

        if (ReSTIRInstanceID == 0)
        {
            // DI
            if (is_valid(gColor)) gColor[pixel] = float4(color, 1.f) / numReSTIRInstances;
            if (is_valid(gEmission)) gEmission[pixel] = float4(emission, 1.f) / numReSTIRInstances;
            if (is_valid(gDiffuseReflectance)) gDiffuseReflectance[pixel] = float4(diffuseReflectance, 1.f) / numReSTIRInstances;
            if (is_valid(gDiffuseIllumination)) gDiffuseIllumination[pixel] = float4(diffuseIllumination, hitT) / numReSTIRInstances;
            if (is_valid(gSpecularReflectance)) gSpecularReflectance[pixel] = float4(specularReflectance, 1.f) / numReSTIRInstances;
            if (is_valid(gSpecularIllumination)) gSpecularIllumination[pixel] = float4(specularIllumination, hitT) / numReSTIRInstances;
        }
        else
        {
            // GI
            if (is_valid(gColor)) gColor[pixel] += float4(color, 1.f) / numReSTIRInstances;
            if (is_valid(gEmission)) gEmission[pixel] += float4(emission, 1.f) / numReSTIRInstances;
            if (is_valid(gDiffuseReflectance)) gDiffuseReflectance[pixel] += float4(diffuseReflectance, 1.f) / numReSTIRInstances;
            if (is_valid(gDiffuseIllumination)) gDiffuseIllumination[pixel] += float4(diffuseIllumination, hitT) / numReSTIRInstances;
            if (is_valid(gSpecularReflectance)) gSpecularReflectance[pixel] += float4(specularReflectance, 1.f) / numReSTIRInstances;
            if (is_valid(gSpecularIllumination)) gSpecularIllumination[pixel] += float4(specularIllumination, hitT) / numReSTIRInstances;
        }
    }
};

cbuffer CB
{
    FinalShading gFinalShading;
};

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    gFinalShading.execute(dispatchThreadId.xy);
}
