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
