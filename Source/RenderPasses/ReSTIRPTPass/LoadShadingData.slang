/***************************************************************************
 # Copyright (c) 2015-21, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#include "Scene/SceneDefines.slangh"
import Scene.Scene;
__exported import Scene.HitInfo;
__exported import Scene.Shading;
import Utils.Math.MathHelpers;
import Rendering.Materials.TexLODHelpers;
import Params;
import Utils.Debug.PixelDebug;

// This was in 2022 SceneDefines.slangh, but not in Falcor 7.0
#define IS_SET(flags, type) ((flags) & (type))

/** Compute derivatives at primarty triangle hit.
*/
void computeDerivativesAtPrimaryTriangleHit(const TriangleHit hit, uint2 pixel, uint2 frameDim, out float2 ddx, out float2 ddy)
{
    // Get world space position
    VertexData v = gScene.getVertexData(hit);
    float hitT = length(v.posW - gScene.camera.getPosition());
    const float3 nonNormalizedRayDir = gScene.camera.computeNonNormalizedRayDirPinhole(pixel, frameDim);
    float3 rayDir = normalize(nonNormalizedRayDir);

    float3 dDdx, dDdy;
    computeRayDirectionDifferentials(nonNormalizedRayDir, gScene.camera.data.cameraU, gScene.camera.data.cameraV, frameDim, dDdx, dDdy);

    RayDiff rd = RayDiff(float3(0.0f), float3(0.0f), dDdx, dDdy);
    rd = rd.propagate(v.posW.xyz, rayDir, hitT, v.faceNormalW);

    // Bounce using primary hit geometry.
    const float4x4 worldMat = gScene.getWorldMatrix(hit.instanceID);
    const float3x3 worldInvTransposeMat = gScene.getInverseTransposeWorldMatrix(hit.instanceID);
    const uint3 vertexIndices = gScene.getIndices(hit.instanceID, hit.primitiveIndex);
    StaticVertexData vertices[3] = { gScene.getVertex(vertexIndices[0]), gScene.getVertex(vertexIndices[1]), gScene.getVertex(vertexIndices[2]) };
    float2 dBarydx, dBarydy;
    float3 unnormalizedN, normals[3];

    prepareRayDiffAtHitPoint(v, vertices, hit.getBarycentricWeights(), rayDir, worldMat, worldInvTransposeMat, rd, unnormalizedN, normals, dBarydx, dBarydy, ddx, ddy);
}

/** Setup vertex data based on hit point.
    \param[in] hit Hit information.
    \param[in] rayOrigin Ray origin.
    \param[in] rayDir Normalized ray direction.
    \param[out] materialID Material ID.
    \return VertexData struct.
*/
VertexData loadVertexData(const HitInfo hit, const float3 rayOrigin, const float3 rayDir, out uint materialID)
{
    VertexData v = {};
    materialID = {};

#if SCENE_HAS_GEOMETRY_TYPE(GEOMETRY_TYPE_TRIANGLE_MESH)
    if (hit.getType() == HitType::Triangle)
    {
        const TriangleHit triangleHit = hit.getTriangleHit();
        StaticVertexData vertices[3];
        v = gScene.getVertexData(triangleHit, vertices);
        materialID = gScene.getMaterialID(triangleHit.instanceID);
    }
#endif
#if SCENE_HAS_GEOMETRY_TYPE(GEOMETRY_TYPE_DISPLACED_TRIANGLE_MESH)
    if (hit.getType() == HitType::DisplacedTriangle)
    {
        const DisplacedTriangleHit displacedTriangleHit = hit.getDisplacedTriangleHit();
        v = gScene.getVertexData(displacedTriangleHit, -rayDir);
        materialID = gScene.getMaterialID(displacedTriangleHit.instanceID);
    }
#endif
#if SCENE_HAS_GEOMETRY_TYPE(GEOMETRY_TYPE_CURVE)
    if (hit.getType() == HitType::Curve)
    {
        const CurveHit curveHit = hit.getCurveHit();
        v = gScene.getVertexDataFromCurve(curveHit);
        materialID = gScene.getMaterialID(curveHit.instanceID);
    }
#endif
#if SCENE_HAS_GEOMETRY_TYPE(GEOMETRY_TYPE_SDF_GRID)
    if (hit.getType() == HitType::SDFGrid)
    {
        const SDFGridHit sdfGridHit = hit.getSDFGridHit();
        v = gScene.getVertexDataFromSDFGrid(sdfGridHit, rayOrigin, rayDir);
        materialID = gScene.getMaterialID(sdfGridHit.instanceID);
    }
#endif
    return v;
}

/**
 * Determine hints to use when creating the material instance.
 * This function is aligned with PathTracer, but not
 * RTXDIPass, ScreenSpaceReSTIRPass.
 *
 * Note that in some functions, HitInfo is not available (only have ShadingData and Reservoir),
 * then the hints can be manually created to 0 (MaterialInstanceHints::None), since
 * both GBUFFER_ADJUST_SHADING_NORMALS and kAdjustShadingNormals are false.
 */
uint getMaterialInstanceHints(const HitInfo hit, const bool isPrimary)
{
    uint hints = 0;
    if (hit.getType() == HitType::Triangle || hit.getType() == HitType::DisplacedTriangle)
    {
#if GBUFFER_ADJUST_SHADING_NORMALS
        if (kAdjustShadingNormals || isPrimary) hints |= (uint)MaterialInstanceHints::AdjustShadingNormal;
#else
        if (kAdjustShadingNormals && !isPrimary) hints |= (uint)MaterialInstanceHints::AdjustShadingNormal;
#endif
    }
    return hints;
}

/** Setup ShadingData based on loaded vertex/material attributes for a hit point.
    \param[in] hit Hit information.
    \param[in] rayOrigin ray origin.
    \param[in] rayDir Normalized ray direction (= -viewDir).
    \param[in] lod level of detail bias value.
    \return ShadingData struct.
*/
ShadingData loadShadingData(const HitInfo hit, const float3 rayOrigin, const float3 rayDir, const ITextureSampler lod)
{
    uint materialID = {};

    VertexData v = loadVertexData(hit, rayOrigin, rayDir, materialID);
    ShadingData sd = gScene.materials.prepareShadingData(v, materialID, -rayDir, lod);
    return sd;
}

// daqi: currently don't support DisplacedTriangle, which uses rayDir
ShadingData loadShadingDataWithPrevVertexPosition(const HitInfo hit, const float3 prevVertexPosition, const ITextureSampler lod)
{
    uint materialID = {};

    // rayOrigin is just prevVertexPosition
    VertexData v = loadVertexData(hit, prevVertexPosition, float3(0.f) /* rayDir */, materialID);
    float3 viewDir = normalize(prevVertexPosition - v.posW);
    ShadingData sd = gScene.materials.prepareShadingData(v, materialID, viewDir, lod);
    // adjustShadingData(sd, hit, curveSphereRadius, v, isPrimary);
    return sd;
}

/** Setup ShadingData based on loaded vertex/material attributes for a hit point.
    \param[in] hit Hit information.
    \param[in] viewDir Normalized view direction.
    \param[in] isPrimary True if this is the primary hit point.
    \param[in] ddx footprint vector.
    \param[in] ddy footprint vector.
    \return ShadingData struct.
*/
ShadingData loadShadingDataFootprint(const HitInfo hit, const float3 rayOrigin, const float3 rayDir, const float2 ddx, const float2 ddy)
{
    uint materialID = {};
    // only diff is using footprint vector to create texture LOD sampler
    ExplicitGradientTextureSampler lod = { ddx, ddy };

    VertexData v = loadVertexData(hit, rayOrigin, rayDir, materialID);
    ShadingData sd = gScene.materials.prepareShadingData(v, materialID, -rayDir, lod);
    // adjustShadingData(sd, hit, curveSphereRadius, v, isPrimary);
    return sd;
}
