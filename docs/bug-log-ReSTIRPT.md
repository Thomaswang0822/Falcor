# Bug Log of ReSTIR PT

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

## Curr TODO

Error Msg:

```shell
......
Failed to link program:
Rendering/ScreenSpaceReSTIR/InitialResampling.cs.slang (main)

D:\Code\Falcor\build\windows-vs2022\bin\Debug\shaders\Rendering\ScreenSpaceReSTIR\InitialResampling.cs.slang(168): error 30019: expected an expression of type 'Ray', got 'RayDesc'
                    if (Lights::kUseEmissiveLights && evalContext.traceRay(ray, hit))
......
```

looks not difficult. Hope it's not.
