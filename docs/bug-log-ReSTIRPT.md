# Bug Log of ReSTIR PT

## File Refactor Progress

FINISHED

- [X] ComputePathReuseMISWeights.cs.slang
- [X] GeneratePaths.cs.slang (no error, but is different from 7.0 code; refactor likely needed)
- [X] LoadShadingData.slang
- [X] NRDHelpers.slang
- [X] Params.slang (no change; kept Daqi's original)
- [X] PathBuilder.slang
- [X] PathReservoir.slang (no change; kept Daqi's original)
- [X] PathState.slang (no change; kept Daqi's original; it's also in PT pass though)
- [X] PathTracer.slang
- [X] ReflectTypes.cs.slang (no change; kept Daqi's original; dummy code to fit Falcor funcitonality)
- [X] ReSTIRPTPass.cpp (compile, but likely need more work)
- [X] ReSTIRPTPass.h (compile, but likely need more work)
- [X] Shift.slang
- [X] SpatialPathRetrace.cs.slang
- [X] SpatialReuse.cs.slang
- [X] StaticParams.slang (no change; kept Daqi's original)
- [X] TemporalPathRetrace.cs.slang
- [X] TemporalReuse.cs.slang
- [X] TracePass.cs.slang (no change; kept Daqi's original)

## createTexture2D() inconsistent bind-flags

Error Msg:

```shell
Failed to compile render graph:
Error when creating Texture2D of format BGRA8UnormSrgb. The requested bind-flags are not supported. Requested = (ShaderResource | UnorderedAccess), supported = (ShaderResource | RenderTarget).
```

Cause & how to locate:

`logInfo()` in `ResourceCache::allocateResources`.
The error is caused by **ReSTIRPTPass.color**

## 3221225477 Memory Bug

No stack trace, very tricky.

Error Msg:

```shell
......
(Info) BLAS group 0 final size: 9.14 MB
(Warning) GFX Warning: IDevice::getAccelerationStructurePrebuildInfo: IAccelerationStructure::BuildInputs::instanceDescs is null when creating a top-level acceleration structure.

[Done] exited with code=3221225477 in 8.771 seconds
```

Looks like sth wrong happend before/during handling light.

Cause & how to locate:

(After trying to print at so many places), `logInfo()` in `RenderGraphExe::execute`.
The exact cause of this memory error needs further investigation, but the log
tells us it happened during the first call of **ScreenSpaceReSTIRPass**.

Then the memory bug is narrowed down to this line in `ScreenSpaceReSTIR::prepareResources`

```cpp
mpLightTileData = mpDevice->createStructuredBuffer(mpReflectTypes->getRootVar()["lightTileData"], elementCount);
```

At last, `mpReflectTypes->getRootVar()` is a `nullptr`, because I wrongly commented out a section that create the Pass
in the constructor of `ScreenSpaceReSTIR`.

## RayDesc instead of Ray

Error Msg:

```shell
......
Failed to link program:
Rendering/ScreenSpaceReSTIR/InitialResampling.cs.slang (main)

D:\Code\Falcor\build\windows-vs2022\bin\Debug\shaders\Rendering\ScreenSpaceReSTIR\InitialResampling.cs.slang(168): error 30019: expected an expression of type 'Ray', got 'RayDesc'
                    if (Lights::kUseEmissiveLights && evalContext.traceRay(ray, hit))
......
```

A "lovely" bug because the error msg is very informative. The reason under the hood is that Falcor 7.0 uses `Ray` to abstract the API-specific (to DXR) `RayDesc` struct.

## Deprecated StandardMaterial Usage

Error Msg:

```shell
(Error) Caught an exception:

Failed to link program:
Rendering/Materials/StandardMaterial.slang RenderPasses/ScreenSpaceReSTIRPass/FinalShading.cs.slang (main)

Type 'StandardMaterial' in type conformance was not found.
```

One of the difficulties is to draw correspondance b/w Falcor 7.0 and Falcor in 2022. ***Source\RenderPasses\MinimalPathTracer\MinimalPathTracer.rt.slang*** are in both codebases and can serve as
a great reference. A line-by-line comparison gives this key info:

```cs
// 2022 code
static struct Data
{
    // Materials
    StandardMaterial standardMaterial;
} gData;
```

This struct carries material info, but was not in Falcor 7.0 code. In 2022 code it's used as (in `evalDirectAnalytic()`) `gData.standardMaterial.eval(sd, ls.dir)`, and now it becomes `mi.eval(sd, ls.dir, sg)`, where `const IMaterialInstance mi` is the function arg.

The final solution turned to be eaiser than the investigation. Just look at ***FinalShading.cs.slang*** in ***RTXDIPass***: the function and related usage of `StandardMaterial` is up-to-date there.

## Confusing wo vs wi

This is a very serious bug that won't be reported by compiler or runtime (nightmare)! In essense, for doing BSDF sampling (a direction), 2022 code use `wi` as the sampled/scattered direction, which is very counter-intuitive. In Falcor 7.0 codebase, this has been changed. Thus, careful treatment is needed for related code.

This problem is discovered, fortunately, during refactor of ***Shift.slang***, which imports ***Rendering.Materials.MaterialShading*** that is deprecated in Falcor 7.0 codebase.
It contains a handful of BSDF-related function, (sample, eval pdf, eval BSDF, etc.), and they should correspond to functions in `IMaterialInstance` interface. I found it really hard to link old code and current code, then discovered the bug.

Here is something that exists in both codebases (***ClothBRDF.slang***) that best shows the inconsistency.

```cpp
// Falcor 7.0 code
bool sample<S : ISampleGenerator>(const float3 wi, out float3 wo, out float pdf, out float3 weight, out uint lobeType, inout S sg)
{
    wo = sample_cosine_hemisphere_concentric(sampleNext2D(sg), pdf);
    lobeType = (uint)LobeType::DiffuseReflection;

    if (min(wi.z, wo.z) < kMinCosTheta)
    {
        weight = {};
        return false;
    }

    weight = evalWeight(wi, wo);
    return true;
}
```

```cpp
// 2022 code
bool sample<S : ISampleGenerator>(float3 wo, out float3 wi, out float pdf, out float3 weight, out uint lobe, inout S sg)
{
    wi = sample_cosine_hemisphere_concentric(sampleNext2D(sg), pdf);
    lobe = (uint)LobeType::DiffuseReflection;

    if (min(wo.z, wi.z) < kMinCosTheta)
    {
        weight = {};
        return false;
    }

    weight = evalWeight(wo, wi);
    return true;
}
```

Daqi's 2022 code was heavily influenced by this inconsistency, since the `struct PathReservoir` has a field.

```cpp
    float3 rcVertexWi[kRcAttrCount]; // incident direction on reconnection vertex
```

Another layer of confusion arises because ReSTIRPT has both BSDF sampling and NEE (light sampling). In Falcor 7.0 context, which agrees with my personal commonsense:
> In BSDF sampling,
> incident direction `wi` is from shading vertex to previous vertex;
> outgoing direction `wo` is the one being sampled by the model.
>
> In light sampling,
> incident direction `wi` is the direction that connects shading vertex to sampled light;
> outgoing direction `wo`, on the other hand, is from shading vertex to previous vertex.

i.e. BSDF sampling generates `wo`, while NEE generates `wi`. But in 2022 code, `wo` and `wi` were flipped, and thus both sampling methods generate `wi`. This is why field `float3 rcVertexWi[kRcAttrCount]` is "incident direction" in 2022 code. It ought to be "sampled direction".

With this in mind, those deprecated functions in ***Rendering.Materials.MaterialShading*** can then be refactored.

## Deprecated MaterialShading Funtions

In ***Rendering.Materials.MaterialShading***, there are a bunch of stand-alone functions which serves to accomodate legacy code.
In Falcor 7.0, legacy code together with this file is fully deprecated. And corresponding functions is called through the member function of `IMaterialInstance` interface.

Here is an example:

```cs
// In 2022 code, it was
float dstPDF1 = evalPdfBSDF(dstPrimarySd, dstConnectionV);

// Now in Falcor 7.0, first prepare the IMaterialInstance
let hints = getMaterialInstanceHints(dstPrimaryHit, true /* isPrimary */);
let dstMI = gScene.materials.getMaterialInstance(dstPrimarySd, lod, hints);
// then call
float dstPDF1 = dstMI.evalPdf(dstPrimarySd, dstConnectionV);
```

And there are a lot more tricky details. In both codebases, Material-related objects are a higher-level abstraction than BSDF-related objects. We can look at the actual function definitions in the above example:

```cs
/// 2022 code
/** Evaluates the probability density function for both the diffuse and specular sampling strategy.
    \param[in] sd Describes the shading point.
    \param[in] L The normalized incident direction for which to evaluate the pdf.
    \return Probability density with respect to solid angle from the shading point.
*/
float evalPdfBSDF(const ShadingData sd, float3 L, uint allowedSampledFlags = -1, bool allowDeltaEval = false)
{
    float3 wo = sd.toLocal(sd.V);
    float3 wi = sd.toLocal(L);

    FalcorBSDF bsdf;
    bsdf.setup(sd);
    return bsdf.evalPdf(wo, wi, allowedSampledFlags, allowDeltaEval);
}

/// 7.0 code
// Member function of StandardMaterialInstance struct, which implements
// evalPdf() signature of IMaterialInstance interface
/** Evaluates the directional pdf for sampling the given direction.
    \param[in] sd Shading data.
    \param[in] wo Outgoing direction.
    \param[in] useImportanceSampling Hint to use importance sampling, else default to reference implementation if available.
    \return PDF with respect to solid angle for sampling direction wo (0 for delta events).
*/
float evalPdf(const ShadingData sd, const float3 wo, bool useImportanceSampling = true)
{
    float3 wiLocal = sf.toLocal(sd.V);
    float3 woLocal = sf.toLocal(wo);

    if (!isValidHemisphereReflectionOrTransmission(sd, sf, wiLocal, woLocal, wo)) return 0.f;

    if (!useImportanceSampling)
    {
        return evalPdfReference(sd, wiLocal, woLocal);
    }
    else
    {
        StandardBSDF bsdf = StandardBSDF(wiLocal, sd.mtl, data);
        return bsdf.evalPdf(wiLocal, woLocal);
    }
}
```

And we can see `FalcorBSDF` has been renamed to `StandardBSDF` with some updated functionality. The key difference is, previously, users can enable/disable evaluation of certain lobes via `uint allowedSampledFlags = -1, bool allowDeltaEval = false`, but now we lose this control, and evaluation is encapsulated in each BSDF implementation of the template. This
is a good news, since the subtlety of which lobe to sample and whether delta event is handled by the polymorphic implementation of `IMaterialInstance` and `IBSDF` interfaces. In other words, each material and its underlying BSDF implementation will handle the subtlety for users.

One bad news is, the 2022 code has overloaded version of evalPdf:

```cs
// It has an additional output pdfAll
float evalPdfBSDF(const ShadingData sd, float3 L, out float pdfAll, uint allowedSampledFlags = -1, bool allowDeltaEval = false)
{
    float3 wo = sd.toLocal(sd.V);
    float3 wi = sd.toLocal(L);

    FalcorBSDF bsdf;
    bsdf.setup(sd);
    return bsdf.evalPdfAll(wo, wi, pdfAll, allowedSampledFlags, allowDeltaEval);
}

// And the above FalcorBSDF::evalPdfAll is
float evalPdfAll(float3 wo, float3 wi, out float pdfAll, uint allowedSampledTypes = -1, bool allowDeltaEval = false)
{
    pdfAll = 0.f;
    float pdfSingle = 0.f;
    if (!allowDeltaEval && pDiffuseReflection > 0.f)
    {
        float pdf = pDiffuseReflection * diffuseReflection.evalPdf(wo, wi);
        if (allowedSampledTypes & uint(SampledBSDFFlags::DiffuseReflection)) pdfSingle += pdf;
        pdfAll += pdf;
    }
    if (!allowDeltaEval && pDiffuseTransmission > 0.f)
    {
        float pdf = pDiffuseTransmission * diffuseTransmission.evalPdf(wo, wi);
        if (allowedSampledTypes & uint(SampledBSDFFlags::DiffuseTransmission)) pdfSingle += pdf;
        pdfAll += pdf;
    }
    if (pSpecularReflection > 0.f)
    {
        float pdf = pSpecularReflection * specularReflection.evalPdf(wo, wi, allowDeltaEval);
        if (allowedSampledTypes & uint(SampledBSDFFlags::SpecularReflection)) pdfSingle += pdf;
        pdfAll += pdf;
    }
    if (pSpecularReflectionTransmission > 0.f)
    {
        float pdf = pSpecularReflectionTransmission * specularReflectionTransmission.evalPdf(wo, wi, allowDeltaEval);
        if (allowedSampledTypes & uint(SampledBSDFFlags::SpecularReflectionTransmission)) pdfSingle += pdf;
        pdfAll += pdf;
    }
    return pdfSingle;
}
```

The output parameter `pdfAll` should store the "proper" pdf, while the return value `pdfSingle` only counts a subset of values according to `uint allowedSampledTypes`, which is a compression of several `enum class SampledBSDFFlags`. But none of the BSDF extending the `interface IBSDF` in Falcor 7.0 has this freedom.

***The following is my personal understanding and could be wrong.***
> It's possible that these 2 control flags aren't used properly here.
> To be more specific, the flag is `bool allowDeltaEval` in the above
> legacy function `evalPdfAll()`. But when used in ReSTIRPT, the flag is
> called `isDelta2`. The former encodes whether users allow delta event
> evaluation, and the latter encodes whether the material/BSDF has a delta
> event (yes for glass and no for diffuse).
>
> As we have established, in Falcor 7.0, the internal detail "whether the material/BSDF has a delta event" will be handled by the implementation. And
> the `allowDeltaEval` control is **completely gone**. Rigorously,
> the pdf should be evaluated to 0 for delta event, so very likely, this part in 2022 code is buggy.
> This can be seen from the following comparison.

```cs
/// 2022 code, in BxDF.slang
/** Specular reflection and transmission using microfacets.
*/
struct SpecularReflectionTransmissionMicrofacet : IBxDF
{
    /// Some code ......

    float3 eval(float3 wo, float3 wi, bool allowDeltaEval = false)
    {
        if (min(wo.z, abs(wi.z)) < kMinCosTheta) return float3(0.f);

#if EnableDeltaBSDF
        // Handle delta reflection/transmission.
        if (alpha == 0.f)
        {
            // Handle delta reflection/transmission.
            if (allowDeltaEval)
            {
                const bool isReflection = wi.z > 0.f;
                float3 h = normalize(wi + wo * (isReflection ? 1.f : eta));
                if (eta == 1.f && !isReflection && all(isnan(h))) h = float3(0, 0, 1); // wi = -wo case
                h *= float(sign(h.z));

                float woDotH = dot(wo, h);
                float F = evalFresnelDielectric(eta, woDotH);

                if (isReflection) return F;
                else return (1.f - F) * transmissionAlbedo;
            }
            else
                return float3(0.f);
        }
#endif
        /// More code in the function
    }
}
```

```cs
/// 7.0 code, in SpecularMicrofacet.slang
/**
 * Specular reflection and transmission using microfacets.
 */
struct SpecularMicrofacetBSDF : IBSDF, IDifferentiable
{
    /// Some code ......

    [Differentiable]
    float3 eval<S : ISampleGenerator>(const float3 wi, const float3 wo, inout S sg)
    {
        if (min(wi.z, abs(wo.z)) < kMinCosTheta)
            return float3(0.f);

#if EnableDeltaBSDF
        // Handle delta reflection/transmission.
        if (alpha == 0.f)
            return float3(0.f);
#endif
        /// More code in the function
    }
}
```

> And the similar reasoning applies to `uint allowedSampledTypes`: it's also an internal detail that's handled by material implementation code. In `FalcorBSDF::evalPdfAll()`, output `pdfAll` differs by return value `pdfSingle` based on flag `uint allowedSampledTypes` ONLY. But very likely,
> the lobes that are allowed to be sampled are the lobes that have nonzero contribution to the pdf. Thus, there can be just 1 instead of 2 pdf values.
***END OF personal understanding.***

## asTexture() assertion error

Error Msg:

```shell
Assertion failed: this

D:\Code\Falcor\Source\Falcor\Core\API\Resource.cpp:133 (asTexture)

Stacktrace:
 0# Falcor::getStackTrace at D:\Code\Falcor\Source\Falcor\Core\Platform\OS.cpp:334
 1# Falcor::reportAssertion at D:\Code\Falcor\Source\Falcor\Core\Error.cpp:60
 2# Falcor::detail::reportAssertion at D:\Code\Falcor\Source\Falcor\Core\Error.h:160
 3# Falcor::Resource::asTexture at D:\Code\Falcor\Source\Falcor\Core\API\Resource.cpp:133
 4# <lambda_5a9451d12dd38060d6ddc28f9e1498a5>::operator() at D:\Code\Falcor\Source\RenderPasses\ScreenSpaceReSTIRPass\ScreenSpaceReSTIRPass.cpp:373
 ......
```

```cs
/// in ScreenSpaceReSTIRPass::finalShading()

    // Bind output channels as UAV buffers.
    var = mpFinalShadingPass->getRootVar();
    auto bind = [&](const ChannelDesc& channel)
    {
        ref<Texture> pTex = renderData[channel.name]->asTexture();
        var[channel.texname] = pTex;
    };
    for (const auto& channel : kOutputChannels) bind(channel);
```

The solution turns out to be simple: use `getTexture()` instead of `asTexture()`, because `asTexture()` doesn't allow nullptr Texture, while `getTexture()` does.

## Other host-side errors

Error Msg:

```shell
(Info) Begin execute() on pass: ReSTIRPTPass
(Error) Caught an exception:

invalid map<K, T> key
```

There were a handful of other bugs that turned out to be neither challenging nor interesting. They were debugged by Debugger in VS
\+ Falcor stack trace infrastructure.

For example, the above error is caused by accessing some non-exisitant define in `DefineList`, a Falcor wrapper around std::map. Despite of not having a stack trace, its cause was quickly located by the Debugger.

## Buggy ScreenSpaceReSTIR

After fixing all errors (such that the runtime doesn't throw any), the rendering is all black. What a classic! Some quick inspection tells that `ScreenSpaceReSTIR` and `ScreenSpaceReSTIRPass` code on which `ReSTIRPTPass` relies, is buggy.

`ScreenSpaceReSTIR` is essentially pre-2022-version ReSTIR DI + ReSTIR GI. Daqi coded ReSTIR PT on top of it. Thus, it's likely that I created some bug while integrating `ScreenSpaceReSTIR` into
Falcor 7.0.

A brutal solution is simply forgo `ScreenSpaceReSTIR` and disable "comparing with ReSTIR GI" feature. Now the direct lighting input of
ReSTIRPT pass comes from RTXDI.

Now, after all the ReSTIRPT operations, the final output is exactly the same as that of RTXDI. Next step is to (finally) shader-debug ReSTIRPT.

NOTE for myself: The path flag used in PathReservoir by ReSTIRPT is incompatible with Falcor 7.0. Fix this first.
