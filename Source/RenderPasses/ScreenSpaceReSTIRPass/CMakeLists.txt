add_plugin(ScreenSpaceReSTIRPass)

target_sources(ScreenSpaceReSTIRPass PRIVATE
    FinalShading.cs.slang
    LoadShadingData.slang
    PrepareSurfaceData.cs.slang
    ScreenSpaceReSTIRPass.cpp
    ScreenSpaceReSTIRPass.h
)

target_copy_shaders(ScreenSpaceReSTIRPass RenderPasses/ScreenSpaceReSTIRPass)

target_source_group(ScreenSpaceReSTIRPass "RenderPasses")
