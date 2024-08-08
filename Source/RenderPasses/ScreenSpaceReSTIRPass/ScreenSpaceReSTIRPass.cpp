/***************************************************************************
 # Copyright (c) 2015-21, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/
#include "ScreenSpaceReSTIRPass.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "RenderGraph/RenderPassStandardFlags.h"

namespace
{
    const char kDesc[] = "Standalone pass for direct lighting with screen-space ReSTIR.";

    const char kPrepareSurfaceDataFile[] = "RenderPasses/ScreenSpaceReSTIRPass/PrepareSurfaceData.cs.slang";
    const char kFinalShadingFile[] = "RenderPasses/ScreenSpaceReSTIRPass/FinalShading.cs.slang";

    const std::string kInputVBuffer = "vbuffer";
    const std::string kInputMotionVectors = "motionVectors";

    const Falcor::ChannelList kInputChannels =
    {
        { kInputVBuffer, "gVBuffer", "Visibility buffer in packed format", false, ResourceFormat::Unknown },
        { kInputMotionVectors, "gMotionVectors", "Motion vector buffer (float format)", true /* optional */, ResourceFormat::RG32Float },
    };

    const Falcor::ChannelList kOutputChannels =
    {
        { "color",                  "gColor",                   "Final color",              true /* optional */, ResourceFormat::RGBA32Float },
        { "emission",               "gEmission",                "Emissive color",           true /* optional */, ResourceFormat::RGBA32Float },
        { "diffuseIllumination",    "gDiffuseIllumination",     "Diffuse illumination",     true /* optional */, ResourceFormat::RGBA32Float },
        { "diffuseReflectance",     "gDiffuseReflectance",      "Diffuse reflectance",      true /* optional */, ResourceFormat::RGBA32Float },
        { "specularIllumination",   "gSpecularIllumination",    "Specular illumination",    true /* optional */, ResourceFormat::RGBA32Float },
        { "specularReflectance",    "gSpecularReflectance",     "Specular reflectance",     true /* optional */, ResourceFormat::RGBA32Float },
        { "debug",                  "gDebug",                   "Debug output",             true /* optional */, ResourceFormat::RGBA32Float },
    };

    // Scripting options.
    const char* kOptions = "options";
    const char* kNumReSTIRInstances = "NumReSTIRInstances";
}

extern "C" FALCOR_API_EXPORT void registerPlugin(Falcor::PluginRegistry& registry)
{
    registry.registerClass<RenderPass, ScreenSpaceReSTIRPass>();
    ScriptBindings::registerBinding(ScreenSpaceReSTIRPass::registerBindings);
    //lib.registerClass("ScreenSpaceReSTIRPass", kDesc, ScreenSpaceReSTIRPass::create);
}

void ScreenSpaceReSTIRPass::reset()
{
    
}

void ScreenSpaceReSTIRPass::registerBindings(pybind11::module& m)
{
    pybind11::class_<ScreenSpaceReSTIRPass, RenderPass, ref<ScreenSpaceReSTIRPass>> pass(m, "ScreenSpaceReSTIRPass");
    pass.def("reset", &ScreenSpaceReSTIRPass::reset);
    pass.def_property_readonly("pixelStats", &ScreenSpaceReSTIRPass::getPixelStats);

    /// There isn't a params.slang that defines SSReSTIRParams, thus no mParams exists.
    /// 
    /*pass.def_property(
        "useFixedSeed",
        [](const ScreenSpaceReSTIRPass* pt) { return pt->mParams.useFixedSeed ? true : false; },
        [](ScreenSpaceReSTIRPass* pt, bool value) { pt->mParams.useFixedSeed = value ? 1 : 0; }
    );
    pass.def_property(
        "fixedSeed",
        [](const ScreenSpaceReSTIRPass* pt) { return pt->mParams.fixedSeed; },
        [](ScreenSpaceReSTIRPass* pt, uint32_t value) { pt->mParams.fixedSeed = value; }
    );*/
}

//std::string ScreenSpaceReSTIRPass::getDesc() { return kDesc; }

ScreenSpaceReSTIRPass::ScreenSpaceReSTIRPass(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    parseProperties(props);
}

void ScreenSpaceReSTIRPass::parseProperties(const Properties& props)
{
    ScreenSpaceReSTIR::Options options;
    for (const auto& [key, value] : props)
    {
        if (key == kOptions)
            mOptions = value;
        else if (key == kNumReSTIRInstances)
            mNumReSTIRInstances = value;
        else
            logWarning("Unknown field '" + key + "' in ScreenSpaceReSTIRPass dictionary");
    }
    //mOptions = make_ref<ScreenSpaceReSTIR::Options>(options);
    //ScreenSpaceReSTIR::Options::create(options);

    if (!mpScreenSpaceReSTIR.empty() && mpScreenSpaceReSTIR[0])
        mpScreenSpaceReSTIR[0]->setOptions(mOptions);
}

//Dictionary ScreenSpaceReSTIRPass::getScriptingDictionary()
//{
//    Dictionary d;
//    d[kOptions] = *mOptions.get();// mpScreenSpaceReSTIR->getOptions();
//    d[kNumReSTIRInstances] = mNumReSTIRInstances;
//    return d;
//}

Properties ScreenSpaceReSTIRPass::getProperties() const
{
    Properties props;
    props[kOptions] = mOptions;
    return props;
}

RenderPassReflection ScreenSpaceReSTIRPass::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;

    addRenderPassOutputs(reflector, kOutputChannels);
    addRenderPassInputs(reflector, kInputChannels);

    return reflector;
}

void ScreenSpaceReSTIRPass::compile(RenderContext* pRenderContext, const CompileData& compileData)
{
    mFrameDim = compileData.defaultTexDims;
}

void ScreenSpaceReSTIRPass::setScene(RenderContext* pRenderContext, const ref<Scene>& pScene)
{
    mpScene = pScene;
    mpPrepareSurfaceDataPass = nullptr;
    mpFinalShadingPass = nullptr;

    if (!mpScreenSpaceReSTIR.empty())
    {
        //mOptions = mpScreenSpaceReSTIR->getOptions();
        mpScreenSpaceReSTIR.clear();
    }

    if (mpScene)
    {
        if (pScene->hasProceduralGeometry())
        {
            logError("This render pass does not support procedural primitives such as curves.");
        }
        // mNumReSTIRInstances is 1
        mpScreenSpaceReSTIR.resize(mNumReSTIRInstances);
        for (int i = 0; i < mNumReSTIRInstances; i++)
        {
            mpScreenSpaceReSTIR[i] = std::make_unique<ScreenSpaceReSTIR>(mpScene, mOptions, mNumReSTIRInstances, i);
        }
    }
}

bool ScreenSpaceReSTIRPass::onMouseEvent(const MouseEvent& mouseEvent)
{
    return (!mpScreenSpaceReSTIR.empty() && mpScreenSpaceReSTIR[0])
        ? mpScreenSpaceReSTIR[0]->getPixelDebug().onMouseEvent(mouseEvent)
        : false;
}


//void ScreenSpaceReSTIRPass::updateDict(const Dictionary& dict)
//{
//    parseDictionary(dict);
//    mOptionsChanged = true;
//    if (!mpScreenSpaceReSTIR.empty() && mpScreenSpaceReSTIR[0])
//        mpScreenSpaceReSTIR[0]->resetReservoirCount();
//}

void ScreenSpaceReSTIRPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mNeedRecreateReSTIRInstances) setScene(pRenderContext, mpScene);

    const auto& pVBuffer = renderData[kInputVBuffer]->asTexture();
    const auto& pMotionVectors = renderData[kInputMotionVectors]->asTexture();

    // Clear outputs if ReSTIR module is not initialized.
    if (mpScreenSpaceReSTIR.empty())
    {
        auto clear = [&](const ChannelDesc& channel)
        {
            auto pTex = renderData[channel.name]->asTexture();
            if (pTex) pRenderContext->clearUAV(pTex->getUAV().get(), float4(0.f));
        };
        for (const auto& channel : kOutputChannels) clear(channel);
        return;
    }

    auto& dict = renderData.getDictionary();

    if (dict.keyExists("enableScreenSpaceReSTIR"))
    {
        for (int i = 0; i < mpScreenSpaceReSTIR.size(); i++)
            mpScreenSpaceReSTIR[i]->enablePass((bool)dict["enableScreenSpaceReSTIR"]);
    }

    // Update refresh flag if changes that affect the output have occured.
    if (mOptionsChanged)
    {
        auto flags = dict.getValue(kRenderPassRefreshFlags, Falcor::RenderPassRefreshFlags::None);
        flags |= Falcor::RenderPassRefreshFlags::RenderOptionsChanged;
        dict[Falcor::kRenderPassRefreshFlags] = flags;
        mOptionsChanged = false;
    }

    // Check if GBuffer has adjusted shading normals enabled.
    mGBufferAdjustShadingNormals = dict.getValue(Falcor::kRenderPassGBufferAdjustShadingNormals, false);

    for (int i = 0; i < mpScreenSpaceReSTIR.size(); i++)
    {
        mpScreenSpaceReSTIR[i]->beginFrame(pRenderContext, mFrameDim);

        prepareSurfaceData(pRenderContext, pVBuffer, i);

        mpScreenSpaceReSTIR[i]->updateReSTIRDI(pRenderContext, pMotionVectors);

        finalShading(pRenderContext, pVBuffer, renderData, i);

        mpScreenSpaceReSTIR[i]->endFrame(pRenderContext);
    }

    auto copyTexture = [pRenderContext](Texture* pDst, const Texture* pSrc)
    {
        if (pDst && pSrc)
        {
            assert(pDst && pSrc);
            assert(pDst->getFormat() == pSrc->getFormat());
            assert(pDst->getWidth() == pSrc->getWidth() && pDst->getHeight() == pSrc->getHeight());
            pRenderContext->copyResource(pDst, pSrc);
        }
        else if (pDst)
        {
            pRenderContext->clearUAV(pDst->getUAV().get(), uint4(0, 0, 0, 0));
        }
    };
    // Copy debug output if available. (only support first ReSTIR instance for now)
    if (const auto& pDebug = renderData["debug"]->asTexture())
    {
        copyTexture(pDebug.get(), mpScreenSpaceReSTIR[0]->getDebugOutputTexture().get());
    }
}

void ScreenSpaceReSTIRPass::renderUI(Gui::Widgets& widget)
{
    mNeedRecreateReSTIRInstances = widget.var("Num ReSTIR Instances", mNumReSTIRInstances, 1, 8);

    if (!mpScreenSpaceReSTIR.empty() && mpScreenSpaceReSTIR[0])
    {
        mOptionsChanged = mpScreenSpaceReSTIR[0]->renderUI(widget);
        for (int i = 1; i < mpScreenSpaceReSTIR.size(); i++)
        {
            mpScreenSpaceReSTIR[i]->copyRecompileStateFromOtherInstance(mpScreenSpaceReSTIR[0]);
        }
    }
}

void ScreenSpaceReSTIRPass::prepareSurfaceData(RenderContext* pRenderContext, const ref<Texture>& pVBuffer, int instanceID)
{
    FALCOR_ASSERT(!mpScreenSpaceReSTIR.empty());
    FALCOR_ASSERT(pVBuffer);
    FALCOR_ASSERT(mpScreenSpaceReSTIR[instanceID]);
    //assert(!mpScreenSpaceReSTIR.empty() && mpScreenSpaceReSTIR[instanceID]);

    FALCOR_PROFILE(pRenderContext, "prepareSurfaceData");

    if (!mpPrepareSurfaceDataPass)
    {
        ProgramDesc desc;
        desc.addShaderModules(mpScene->getShaderModules());
        desc.addShaderLibrary(kPrepareSurfaceDataFile).csEntry("main");
        desc.addTypeConformances(mpScene->getTypeConformances());

        auto defines = mpScene->getSceneDefines();
        defines.add("GBUFFER_ADJUST_SHADING_NORMALS", mGBufferAdjustShadingNormals ? "1" : "0");
        mpPrepareSurfaceDataPass = ComputePass::create(mpDevice, desc, defines, false);
        // WHY do this?
        mpPrepareSurfaceDataPass->setVars(nullptr);
    }

    mpPrepareSurfaceDataPass->addDefine("GBUFFER_ADJUST_SHADING_NORMALS", mGBufferAdjustShadingNormals ? "1" : "0");

    auto rootVar = mpPrepareSurfaceDataPass->getRootVar();
    mpScene->bindShaderData(rootVar["gScene"]);
    //mpPrepareSurfaceDataPass["gScene"] = mpScene->getParameterBlock();

    auto var = rootVar["CB"]["gPrepareSurfaceData"];

    var["vbuffer"] = pVBuffer;
    var["frameDim"] = mFrameDim;
    //mpScreenSpaceReSTIR[instanceID]->setShaderData(var["screenSpaceReSTIR"]);
    mpScreenSpaceReSTIR[instanceID]->bindShaderData(var["screenSpaceReSTIR"]);

    if (instanceID == 0 && mpFinalShadingPass && mpScreenSpaceReSTIR[0]->mRequestParentRecompile)
    {
        mpFinalShadingPass->setVars(nullptr);
        mpScreenSpaceReSTIR[0]->mRequestParentRecompile = false;
    }

    mpPrepareSurfaceDataPass->execute(pRenderContext, mFrameDim.x, mFrameDim.y);
}

void ScreenSpaceReSTIRPass::finalShading(
    RenderContext* pRenderContext,
    const ref<Texture>& pVBuffer,
    const RenderData& renderData,
    int instanceID
)
{
    FALCOR_ASSERT(!mpScreenSpaceReSTIR.empty());
    FALCOR_ASSERT(pVBuffer);
    FALCOR_ASSERT(mpScreenSpaceReSTIR[instanceID]);
    //assert(!mpScreenSpaceReSTIR.empty() && mpScreenSpaceReSTIR[instanceID]);

    FALCOR_PROFILE(pRenderContext, "finalShading");

    if (!mpFinalShadingPass)
    {
        ProgramDesc desc;
        desc.addShaderModules(mpScene->getShaderModules());
        desc.addShaderLibrary(kFinalShadingFile).csEntry("main");
        desc.addTypeConformances(mpScene->getTypeConformances());

        auto defines = mpScene->getSceneDefines();
        defines.add("GBUFFER_ADJUST_SHADING_NORMALS", mGBufferAdjustShadingNormals ? "1" : "0");
        defines.add("USE_ENV_BACKGROUND", mpScene->useEnvBackground() ? "1" : "0");
        defines.add(getValidResourceDefines(kOutputChannels, renderData));

        mpFinalShadingPass = ComputePass::create(mpDevice, desc, defines, false);
        // WHY
        mpFinalShadingPass->setVars(nullptr);
    }

    mpFinalShadingPass->addDefine("GBUFFER_ADJUST_SHADING_NORMALS", mGBufferAdjustShadingNormals ? "1" : "0");
    mpFinalShadingPass->addDefine("USE_ENV_BACKGROUND", mpScene->useEnvBackground() ? "1" : "0");
    mpFinalShadingPass->addDefine("_USE_LEGACY_SHADING_CODE", "0");

    // For optional I/O resources, set 'is_valid_<name>' defines to inform the program of which ones it can access.
    // TODO: This should be moved to a more general mechanism using Slang.
    mpFinalShadingPass->getProgram()->addDefines(getValidResourceDefines(kOutputChannels, renderData));

    //mpFinalShadingPass["gScene"] = mpScene->getParameterBlock();
    auto rootVar = mpFinalShadingPass->getRootVar();
    mpScene->bindShaderData(rootVar["gScene"]);

    auto var = rootVar["CB"]["gFinalShading"];

    var["vbuffer"] = pVBuffer;
    var["frameDim"] = mFrameDim;
    var["numReSTIRInstances"] = mNumReSTIRInstances;
    var["ReSTIRInstanceID"] = instanceID;

    //mpScreenSpaceReSTIR[instanceID]->setShaderDataPass(var["screenSpaceReSTIR"]);
    mpScreenSpaceReSTIR[instanceID]->bindShaderData(var["screenSpaceReSTIR"]);

    // Bind output channels as UAV buffers.
    var = mpFinalShadingPass->getRootVar();
    auto bind = [&](const ChannelDesc& channel)
    {
        ref<Texture> pTex = renderData[channel.name]->asTexture();
        var[channel.texname] = pTex;
    };
    for (const auto& channel : kOutputChannels) bind(channel);

    mpFinalShadingPass->execute(pRenderContext, mFrameDim.x, mFrameDim.y);
}
