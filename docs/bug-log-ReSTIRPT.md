# Bug Log of ReSTIR PT

## File Refactor Progress

- [X] ComputePathReuseMISWeights.cs.slang
- [ ] GeneratePaths.cs.slang (no error, but is different from 7.0 code; refactor likely needed)
- [X] LoadShadingData.slang
- [X] NRDHelpers.slang
- [X] Params.slang (no change; kept Daqi's original)
- [X] PathBuilder.slang
- [X] PathReservoir.slang (no change; kept Daqi's original)
- [X] PathState.slang (no change; kept Daqi's original; it's also in PT pass though)
- [ ] PathTracer.slang
- [X] ReflectTypes.cs.slang (no change; kept Daqi's original; dummy code to fit Falcor funcitonality)
- [X] ReSTIRPTPass.cpp (compile, but likely need more work)
- [X] ReSTIRPTPass.h (compile, but likely need more work)
- [ ] Shift.slang
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

## Curr TODO

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
