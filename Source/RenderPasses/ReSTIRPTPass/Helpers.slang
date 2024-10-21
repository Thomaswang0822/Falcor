/***************************************************************************
 # Copyright (c) 2024, Haoxuan Wang.  All rights reserved.
 **************************************************************************/

/** This file recreates several deprecated BSDF evaludation functions in Falcor 2022 codebase.
    The key difference is, these functions have additional
    `uint allowedSampledTypes` and `bool allowDeltaEval` parameters, which give user control to
    disable certain lobes in bsdf/pdf eval.
    This feature is not in current Falcor (7.0), but is critical for Hybrid Shift in ReSTIR PT.

NOTE: The lowest-level function arg convention is different. And it seems "wi" was used as wo
in ReSTIRPT code.

// In Falcor 2022 IBxDF.slang

    Low-level interface for BxDFs.

    Conventions:
    - all operations are done in a local coordinate frame.
    - the local frame has normal N=(0,0,1), tangent T=(1,0,0) and bitangent B=(0,1,0).
    - wo is the outgoing or scattering direction and points away from the shading location.
    - wi is the incident or light direction and points away from the shading location.
    - the outgoing direction is always in the positive hemisphere.
    - evaluating the BxDF always includes the foreshortening term (dot(wi, n) = wi.z).

    Evaluates the BxDF.
    \param[in] wo Outgoing direction.
    \param[in] wi Incident direction.
    \return Returns f(wo, wi) * dot(wi, n).
    float3 eval(float3 wo, float3 wi);

// In Falcor 7.0

    Low-level interface for BSDF functions.

    Conventions:
    - All operations are done in a local coordinate frame.
    - The local frame has normal N=(0,0,1), tangent T=(1,0,0) and bitangent B=(0,1,0).
    - The incident and outgoing direction point away from the shading location.
    - The incident direction (wi) is always in the positive hemisphere.
    - The outgoing direction (wo) is sampled.
    - Evaluating the BSDF always includes the foreshortening term (dot(wo, n) = wo.z).

    Evaluates the BSDF.
    \param[in] wi Incident direction.
    \param[in] wo Outgoing direction.
    \param[in,out] sg Sample generator.
    \return Returns f(wi, wo) * dot(wo, n).
    float3 eval<S : ISampleGenerator>(const float3 wi, const float3 wo, inout S sg);

ANOTHER NOTE:
ShadingData.linearRoughness in old 2022 code should be the same as BSDFProperties.roughness
Source\Falcor\RenderPasses\Shared\Denoising\NRDHelpers.slang (exists in both codebases), function float3 getMaterialReflectanceForDeltaPaths()
float ggxAlpha = sd.linearRoughness * sd.linearRoughness;
======
const float ggxAlpha = bsdfProperties.roughness * bsdfProperties.roughness;
 */

import Utils.Sampling.SampleGenerator;
import Rendering.Materials.BSDFs.StandardBSDF;
import Rendering.Materials.StandardMaterialInstance;

/** The deprecated code is
bool hasRoughComponent(const ShadingData sd, float roughnessThreshold) {
    return pDiffuseReflection > 0 || sd.linearRoughness > roughnessThreshold;
}

no need to build full StandardBSDF; use the current active lobes instead, since

pDiffuseReflection =
        (activeLobes & (uint)LobeType::DiffuseReflection) ? detach(diffuseWeight * dielectricBSDF * (1.f - diffTrans)) : 0.f;
*/
bool hasRoughComponent(const uint activeLobes, const float roughness, float roughnessThreshold) {
    return bool(activeLobes & (uint)LobeType::DiffuseReflection) || roughness > roughnessThreshold;
}

enum class SampledBSDFFlags
{
    DiffuseReflection = 1,
    DiffuseTransmission = 2,
    SpecularReflection = 4,
    SpecularTransmission = 8
};

float evalPdfBSDF(const StandardBSDF bsdf, float3 wiLocal, float3 woLocal, out float pdfAll, uint allowedSampledTypes = -1, bool allowDeltaEval = false )
{
    // manually unroll bsdf.evalPdfAll(wo, wi, pdfAll, allowedSampledFlags, allowDeltaEval);
    pdfAll = 0.f;
    float pdfSingle = 0.f;
    if (!allowDeltaEval && bsdf.pDiffuseReflection > 0.f)
    {
        float pdf = bsdf.pDiffuseReflection * bsdf.diffuseReflection.evalPdf(wiLocal, woLocal);
        if (allowedSampledTypes & uint(SampledBSDFFlags::DiffuseReflection)) pdfSingle += pdf;
        pdfAll += pdf;
    }
    if (!allowDeltaEval && bsdf.pDiffuseTransmission > 0.f)
    {
        float pdf = bsdf.pDiffuseTransmission * bsdf.diffuseTransmission.evalPdf(wiLocal, woLocal);
        if (allowedSampledTypes & uint(SampledBSDFFlags::DiffuseTransmission)) pdfSingle += pdf;
        pdfAll += pdf;
    }
    if (bsdf.pSpecularReflection > 0.f)
    {
        float pdf = bsdf.pSpecularReflection * bsdf.specularReflection.evalPdf(wiLocal, woLocal /*, allowDeltaEval*/);
        if (allowedSampledTypes & uint(SampledBSDFFlags::SpecularReflection)) pdfSingle += pdf;
        pdfAll += pdf;
    }
    if (bsdf.pSpecularTransmission > 0.f)
    {
        float pdf = bsdf.pSpecularTransmission * bsdf.specularTransmission.evalPdf(wiLocal, woLocal /*, allowDeltaEval*/);
        if (allowedSampledTypes & uint(SampledBSDFFlags::SpecularTransmission)) pdfSingle += pdf;
        pdfAll += pdf;
    }
    return pdfSingle;
}

float3 evalBSDFCosine(const StandardBSDF bsdf, float3 wiLocal, float3 woLocal, uint allowedSampledTypes = -1, bool allowDeltaEval = false)
{
    // return bsdf.eval(wiLocal, woLocal, allowedSampledFlags, allowDeltaEval);
    SampleGenerator dummySG;

    float3 result = 0.f;
    if ((allowedSampledTypes & uint(SampledBSDFFlags::DiffuseReflection)) && bsdf.pDiffuseReflection > 0.f && !allowDeltaEval)
        result += (1.f - bsdf.specTrans) * (1.f - bsdf.diffTrans) * bsdf.diffuseReflection.eval(wiLocal, woLocal, dummySG);
    if ((allowedSampledTypes & uint(SampledBSDFFlags::DiffuseTransmission)) && bsdf.pDiffuseTransmission > 0.f && !allowDeltaEval)
        result += (1.f - bsdf.specTrans) * bsdf.diffTrans * bsdf.diffuseTransmission.eval(wiLocal, woLocal, dummySG);
    if ((allowedSampledTypes & uint(SampledBSDFFlags::SpecularReflection)) && bsdf.pSpecularReflection > 0.f)
        result += (1.f - bsdf.specTrans) * bsdf.specularReflection.eval(wiLocal, woLocal, dummySG);
    if ((allowedSampledTypes & uint(SampledBSDFFlags::SpecularTransmission)) && bsdf.pSpecularTransmission > 0.f)
        result += bsdf.specTrans * (bsdf.specularTransmission.eval(wiLocal, woLocal, dummySG));

    return result;
}

float evalPdfBSDF(const StandardBSDF bsdf, float3 wiLocal, float3 woLocal, uint allowedSampledTypes = -1, bool allowDeltaEval = false)
{
    // return bsdf.evalPdf(wo, wi, allowedSampledFlags, allowDeltaEval);

    float pdf = 0.f;
    if ((allowedSampledTypes & uint(SampledBSDFFlags::DiffuseReflection)) && !allowDeltaEval && bsdf.pDiffuseReflection > 0.f)
        pdf += bsdf.pDiffuseReflection * bsdf.diffuseReflection.evalPdf(wiLocal, woLocal);
    if ((allowedSampledTypes & uint(SampledBSDFFlags::DiffuseTransmission)) && !allowDeltaEval && bsdf.pDiffuseTransmission > 0.f)
        pdf += bsdf.pDiffuseTransmission * bsdf.diffuseTransmission.evalPdf(wiLocal, woLocal);
    if ((allowedSampledTypes & uint(SampledBSDFFlags::SpecularReflection)) && bsdf.pSpecularReflection > 0.f)
        pdf += bsdf.pSpecularReflection * bsdf.specularReflection.evalPdf(wiLocal, woLocal);
    if ((allowedSampledTypes & uint(SampledBSDFFlags::SpecularTransmission)) && bsdf.pSpecularTransmission > 0.f)
        pdf += bsdf.pSpecularTransmission * bsdf.specularTransmission.evalPdf(wiLocal, woLocal);

    return pdf;
}

/** This is a special-version pdf evaluation used in Hybrid Shift,
where pdf at rcVertex (reconnection vertex) will be eval with wi = -srcConnectionV (src connection view dir)
instead of the matching view dir rcVertexSd.V
*/
float evalPdfBSDFWithV(const ShadingData sd, const StandardMaterialInstance mi, float3 V, float3 wo, uint allowedSampledTypes = -1, bool allowDeltaEval = false )
{
    float3 wiLocal = mi.sf.toLocal(V);
    float3 woLocal = mi.sf.toLocal(wo);
    StandardBSDF bsdf = StandardBSDF(wiLocal, sd.mtl, mi.data);

    // return bsdf.evalPdf(wo, wi, allowedSampledFlags, allowDeltaEval);
    float pdf = 0.f;
    if ((allowedSampledTypes & uint(SampledBSDFFlags::DiffuseReflection)) && !allowDeltaEval && bsdf.pDiffuseReflection > 0.f)
        pdf += bsdf.pDiffuseReflection * bsdf.diffuseReflection.evalPdf(wiLocal, woLocal);
    if ((allowedSampledTypes & uint(SampledBSDFFlags::DiffuseTransmission)) && !allowDeltaEval && bsdf.pDiffuseTransmission > 0.f)
        pdf += bsdf.pDiffuseTransmission * bsdf.diffuseTransmission.evalPdf(wiLocal, woLocal);
    if ((allowedSampledTypes & uint(SampledBSDFFlags::SpecularReflection)) && bsdf.pSpecularReflection > 0.f)
        pdf += bsdf.pSpecularReflection * bsdf.specularReflection.evalPdf(wiLocal, woLocal);
    if ((allowedSampledTypes & uint(SampledBSDFFlags::SpecularTransmission)) && bsdf.pSpecularTransmission > 0.f)
        pdf += bsdf.pSpecularTransmission * bsdf.specularTransmission.evalPdf(wiLocal, woLocal);

    return pdf;
}
