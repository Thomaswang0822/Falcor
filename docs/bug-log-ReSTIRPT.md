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

## Current TODO

Error Msg:

```shell
......
(Info) BLAS group 0 final size: 9.14 MB
(Warning) GFX Warning: IDevice::getAccelerationStructurePrebuildInfo: IAccelerationStructure::BuildInputs::instanceDescs is null when creating a top-level acceleration structure.

[Done] exited with code=3221225477 in 8.771 seconds
```

Looks like sth wrong happend before/during handling light.
