/***************************************************************************
 # Copyright (c) 2015-21, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
#pragma once
#include "Falcor.h"
#include "RenderGraph/RenderPass.h"
#include "Rendering/ScreenSpaceReSTIR/ScreenSpaceReSTIR.h"
#include "Rendering/Utils/PixelStats.h"

using namespace Falcor;

/** Direct illumination using screen-space ReSTIR.

    This is similar to the SpatiotemporalReservoirResampling pass but uses the
    ScreenSpaceReSTIR module and serves as an example of how to integrate it.
*/
class ScreenSpaceReSTIRPass : public RenderPass
{
public:
    FALCOR_PLUGIN_CLASS(ScreenSpaceReSTIRPass, "ScreenSpaceReSTIRPass",
        {"Standalone pass for screen-space ReSTIR, which includes ReSTIR DI and GI."})

    static ref<ScreenSpaceReSTIRPass> create(ref<Device> pDevice, const Properties& props) {
        return make_ref<ScreenSpaceReSTIRPass>(pDevice, props);
    }

    ScreenSpaceReSTIRPass(ref<Device> pDevice, const Properties& props);

    virtual Properties getProperties() const override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pRenderContext, const CompileData& compileData) override;
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const ref<Scene>& pScene) override;

    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override;
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override { return false; }

    PixelStats& getPixelStats() const { return *mpPixelStats; }

    void reset();

    static void registerBindings(pybind11::module& m);

private:
    void parseProperties(const Properties& props);
    //void recreatePrograms();
    void prepareSurfaceData(RenderContext* pRenderContext, const ref<Texture>& pVBuffer, int instanceID);
    void finalShading(RenderContext* pRenderContext, const ref<Texture>& pVBuffer, const RenderData& renderData, int instanceID);

    // Internal state
    ref<Scene> mpScene;

    std::vector<std::unique_ptr<ScreenSpaceReSTIR>> mpScreenSpaceReSTIR;
    ScreenSpaceReSTIR::Options mOptions;
    std::unique_ptr<PixelStats> mpPixelStats; ///< Utility class for collecting pixel stats.
    std::unique_ptr<PixelDebug> mpPixelDebug; ///< Utility class for pixel debugging (print in shaders).

    ref<ComputePass> mpPrepareSurfaceDataPass;
    ref<ComputePass> mpFinalShadingPass;

    uint2 mFrameDim = {0, 0};
    bool mOptionsChanged = false;
    bool mGBufferAdjustShadingNormals = false;
    int mNumReSTIRInstances = 1;
    bool mNeedRecreateReSTIRInstances = false;
};
