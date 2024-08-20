/***************************************************************************
 # Copyright (c) 2015-23, NVIDIA CORPORATION. All rights reserved.
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

/***************************************************************************
 # Copyright (c) 2022, Daqi Lin.  All rights reserved.
 **************************************************************************/

#include "ReSTIRPTPass.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "RenderGraph/RenderPassStandardFlags.h"
#include "Rendering/Lights/EmissiveUniformSampler.h"

namespace
{
    const std::string kGeneratePathsFilename = "RenderPasses/ReSTIRPTPass/GeneratePaths.cs.slang";
    const std::string kTracePassFilename = "RenderPasses/ReSTIRPTPass/TracePass.cs.slang";
    const std::string kReflectTypesFile = "RenderPasses/ReSTIRPTPass/ReflectTypes.cs.slang";
    const std::string kSpatialReusePassFile = "RenderPasses/ReSTIRPTPass/SpatialReuse.cs.slang";
    const std::string kTemporalReusePassFile = "RenderPasses/ReSTIRPTPass/TemporalReuse.cs.slang";
    const std::string kSpatialPathRetraceFile = "RenderPasses/ReSTIRPTPass/SpatialPathRetrace.cs.slang";
    const std::string kTemporalPathRetraceFile = "RenderPasses/ReSTIRPTPass/TemporalPathRetrace.cs.slang";
    const std::string kComputePathReuseMISWeightsFile = "RenderPasses/ReSTIRPTPass/ComputePathReuseMISWeights.cs.slang";

    // Render pass inputs and outputs.
    const std::string kInputVBuffer = "vbuffer";
    const std::string kInputMotionVectors = "motionVectors";
    const std::string kInputViewDir = "viewW";
    const std::string kInputSampleCount = "sampleCount";
    const std::string kInputDirectLighting = "directLighting";

    const Falcor::ChannelList kInputChannels =
    {
        { kInputVBuffer,        "gVBuffer",         "Visibility buffer in packed format", false, ResourceFormat::Unknown },
        { kInputMotionVectors,  "gMotionVectors",   "Motion vector buffer (float format)", true /* optional */, ResourceFormat::RG32Float },
        { kInputDirectLighting,    "gDirectLighting",     "Sample count buffer (integer format)", true /* optional */, ResourceFormat::RGBA32Float },
    };

    const std::string kOutputColor = "color";
    const std::string kOutputAlbedo = "albedo";
    const std::string kOutputSpecularAlbedo = "specularAlbedo";
    const std::string kOutputIndirectAlbedo = "indirectAlbedo";
    const std::string kOutputNormal = "normal";
    const std::string kOutputReflectionPosW = "reflectionPosW";
    const std::string kOutputRayCount = "rayCount";
    const std::string kOutputPathLength = "pathLength";
    const std::string kOutputDebug = "debug";
    const std::string kOutputTime = "time";
    const std::string kOutputNRDDiffuseRadianceHitDist = "nrdDiffuseRadianceHitDist";
    const std::string kOutputNRDSpecularRadianceHitDist = "nrdSpecularRadianceHitDist";
    const std::string kOutputNRDResidualRadianceHitDist = "nrdResidualRadianceHitDist";
    const std::string kOutputNRDEmission = "nrdEmission";
    const std::string kOutputNRDDiffuseReflectance = "nrdDiffuseReflectance";
    const std::string kOutputNRDSpecularReflectance = "nrdSpecularReflectance";


    const Falcor::ChannelList kOutputChannels =
    {
        { kOutputColor,                 "gOutputColor",                 "Output color (linear)", true /* optional */ },
        { kOutputAlbedo,                "gOutputAlbedo",                "Output albedo (linear)", true /* optional */, ResourceFormat::RGBA8Unorm },
        { kOutputNormal,                "gOutputNormal",                "Output normal (linear)", true /* optional */, ResourceFormat::RGBA16Float },
        { kOutputRayCount,              "",                             "Per-pixel ray count", true /* optional */, ResourceFormat::R32Uint },
        { kOutputPathLength,            "",                             "Per-pixel path length", true /* optional */, ResourceFormat::R32Uint },
        { kOutputDebug,                 "",                             "Debug output", true /* optional */, ResourceFormat::RGBA32Float },
        { kOutputTime,                  "",                             "Per-pixel time", true /* optional */, ResourceFormat::R32Uint },
        { kOutputSpecularAlbedo,                "gOutputSpecularAlbedo",                "Output specular albedo (linear)", true /* optional */, ResourceFormat::RGBA8Unorm },
        { kOutputIndirectAlbedo,                "gOutputIndirectAlbedo",                "Output indirect albedo (linear)", true /* optional */, ResourceFormat::RGBA8Unorm },
        { kOutputReflectionPosW,                "gOutputReflectionPosW",                "Output reflection pos (world space)", true /* optional */, ResourceFormat::RGBA32Float },
        { kOutputNRDDiffuseRadianceHitDist,     "gOutputNRDDiffuseRadianceHitDist",     "Output demodulated diffuse color (linear) and hit distance", true /* optional */, ResourceFormat::RGBA32Float },
        { kOutputNRDSpecularRadianceHitDist,    "gOutputNRDSpecularRadianceHitDist",    "Output demodulated specular color (linear) and hit distance", true /* optional */, ResourceFormat::RGBA32Float },
        { kOutputNRDResidualRadianceHitDist,    "gOutputNRDResidualRadianceHitDist",    "Output residual color (linear) and hit distance", true /* optional */, ResourceFormat::RGBA32Float },
        { kOutputNRDEmission,                   "gOutputNRDEmission",                   "Output primary surface emission", true /* optional */, ResourceFormat::RGBA32Float },
        { kOutputNRDDiffuseReflectance,         "gOutputNRDDiffuseReflectance",         "Output primary surface diffuse reflectance", true /* optional */, ResourceFormat::RGBA16Float },
        { kOutputNRDSpecularReflectance,        "gOutputNRDSpecularReflectance",        "Output primary surface specular reflectance", true /* optional */, ResourceFormat::RGBA16Float },
    };

    // UI variables.
    // UPDATE: GUI::DropdownList can be put here; see NRDPass.cpp
    const Gui::DropdownList kColorFormatList =
    {
        { (uint32_t)ColorFormat::RGBA32F, "RGBA32F (128bpp)" },
        { (uint32_t)ColorFormat::LogLuvHDR, "LogLuvHDR (32bpp)" },
    };

    const Gui::DropdownList kMISHeuristicList =
    {
        { (uint32_t)MISHeuristic::Balance, "Balance heuristic" },
        { (uint32_t)MISHeuristic::PowerTwo, "Power heuristic (exp=2)" },
        { (uint32_t)MISHeuristic::PowerExp, "Power heuristic" },
    };

    const Gui::DropdownList kShiftMappingList =
    {
        { (uint32_t)ShiftMapping::Reconnection, "Reconnection" },
        { (uint32_t)ShiftMapping::RandomReplay, "Random Replay" },
        { (uint32_t)ShiftMapping::Hybrid, "Hybrid" },
    };

    const Gui::DropdownList kReSTIRMISList =
    {
        { (uint32_t)ReSTIRMISKind::Constant , "Constant resampling MIS (with balance-heuristic contribution MIS)" },
        { (uint32_t)ReSTIRMISKind::Talbot, "Talbot resampling MIS" },
        { (uint32_t)ReSTIRMISKind::Pairwise, "Pairwise resampling MIS" },
        { (uint32_t)ReSTIRMISKind::ConstantBinary, "Constant resampling MIS (with 1/|Z| contribution MIS)" },
        { (uint32_t)ReSTIRMISKind::ConstantBiased, "Constant resampling MIS (constant contribution MIS, biased)" },
    };

    const Gui::DropdownList kReSTIRMISList2 =
    {
        { (uint32_t)ReSTIRMISKind::Constant , "Constant resampling MIS (with balance-heuristic contribution MIS)" },
        { (uint32_t)ReSTIRMISKind::Talbot, "Talbot resampling MIS" },
        { (uint32_t)ReSTIRMISKind::ConstantBinary, "Constant resampling MIS (with 1/|Z| contribution MIS)" },
        { (uint32_t)ReSTIRMISKind::ConstantBiased, "Constant resampling MIS (constant contribution MIS, biased)" },
    };

    const Gui::DropdownList kPathReusePatternList =
    {
        { (uint32_t)PathReusePattern::Block, std::string("Block")},
        { (uint32_t)PathReusePattern::NRooks, std::string("N-Rooks")},
        { (uint32_t)PathReusePattern::NRooksShift, std::string("N-Rooks Shift")},
    };

    const Gui::DropdownList kSpatialReusePatternList =
    {
        { (uint32_t)SpatialReusePattern::Default, std::string("Default")},
        { (uint32_t)SpatialReusePattern::SmallWindow, std::string("Small Window")},
    };

    const Gui::DropdownList kEmissiveSamplerList =
    {
        { (uint32_t)EmissiveLightSamplerType::Uniform, "Uniform" },
        { (uint32_t)EmissiveLightSamplerType::LightBVH, "LightBVH" },
        { (uint32_t)EmissiveLightSamplerType::Power, "Power" },
    };

    const Gui::DropdownList kLODModeList =
    {
        { (uint32_t)TexLODMode::Mip0, "Mip0" },
        { (uint32_t)TexLODMode::RayDiffs, "Ray Diffs" }
    };

    const Gui::DropdownList kPathSamplingModeList =
    {
        { (uint32_t)PathSamplingMode::ReSTIR, "ReSTIR PT" },
        { (uint32_t)PathSamplingMode::PathReuse, "Bekaert-style Path Reuse" },
        { (uint32_t)PathSamplingMode::PathTracing, "Path Tracing" }
    };

    // Scripting options.
    const std::string kSamplesPerPixel = "samplesPerPixel";
    const std::string kMaxSurfaceBounces = "maxSurfaceBounces";
    const std::string kMaxDiffuseBounces = "maxDiffuseBounces";
    const std::string kMaxSpecularBounces = "maxSpecularBounces";
    const std::string kMaxTransmissionBounces = "maxTransmissionBounces";
    const std::string kAdjustShadingNormals = "adjustShadingNormals";
    const std::string kLODBias = "lodBias";
    const std::string kSampleGenerator = "sampleGenerator";
    const std::string kUseBSDFSampling = "useBSDFSampling";
    const std::string kUseNEE = "useNEE";
    const std::string kUseMIS = "useMIS";
    const std::string kUseRussianRoulette = "useRussianRoulette";
    const std::string kScreenSpaceReSTIROptions = "screenSpaceReSTIROptions";
    const std::string kUseAlphaTest = "useAlphaTest";
    const std::string kMaxNestedMaterials = "maxNestedMaterials";
    const std::string kUseLightsInDielectricVolumes = "useLightsInDielectricVolumes";
    const std::string kLimitTransmission = "limitTransmission";
    const std::string kMaxTransmissionReflectionDepth = "maxTransmissionReflectionDepth";
    const std::string kMaxTransmissionRefractionDepth = "maxTransmissionRefractionDepth";
    const std::string kDisableCaustics = "disableCaustics";
    const std::string kSpecularRoughnessThreshold = "specularRoughnessThreshold";
    const std::string kDisableDirectIllumination = "disableDirectIllumination";
    const std::string kColorFormat = "colorFormat";
    const std::string kMISHeuristic = "misHeuristic";
    const std::string kMISPowerExponent = "misPowerExponent";
    const std::string kFixedSeed = "fixedSeed";
    const std::string kEmissiveSampler = "emissiveSampler";
    const std::string kLightBVHOptions = "lightBVHOptions";
    const std::string kPrimaryLodMode = "primaryLodMode";
    const std::string kUseNRDDemodulation = "useNRDDemodulation";

    const std::string kOutputSize = "outputSize";
    const std::string kFixedOutputSize = "fixedOutputSize";

    const std::string kSpatialMisKind = "spatialMisKind";
    const std::string kTemporalMisKind = "temporalMisKind";
    const std::string kShiftStrategy = "shiftStrategy";
    const std::string kRejectShiftBasedOnJacobian = "rejectShiftBasedOnJacobian";
    const std::string kJacobianRejectionThreshold = "jacobianRejectionThreshold";
    const std::string kNearFieldDistance = "nearFieldDistance";
    const std::string kLocalStrategyType = "localStrategyType";

    const std::string kTemporalHistoryLength = "temporalHistoryLength";
    const std::string kUseMaxHistory = "useMaxHistory";
    const std::string kSeedOffset = "seedOffset";
    const std::string kEnableTemporalReuse = "enableTemporalReuse";
    const std::string kEnableSpatialReuse = "enableSpatialReuse";
    const std::string kNumSpatialRounds = "numSpatialRounds";
    const std::string kPathSamplingMode = "pathSamplingMode";
    const std::string kEnableTemporalReprojection = "enableTemporalReprojection";
    const std::string kNoResamplingForTemporalReuse = "noResamplingForTemporalReuse";
    const std::string kSpatialNeighborCount = "spatialNeighborCount";
    const std::string kFeatureBasedRejection = "featureBasedRejection";
    const std::string kSpatialReusePattern = "spatialReusePattern";
    const std::string kSmallWindowRestirWindowRadius = "smallWindowRestirWindowRadius";
    const std::string kSpatialReuseRadius = "spatialReuseRadius";
    const std::string kUseDirectLighting = "useDirectLighting";
    const std::string kSeparatePathBSDF = "separatePathBSDF";
    const std::string kCandidateSamples = "candidateSamples";
    const std::string kTemporalUpdateForDynamicScene = "temporalUpdateForDynamicScene";
    const std::string kEnableRayStats = "enableRayStats";

    const uint32_t kNeighborOffsetCount = 8192;
    } // namespace

extern "C" FALCOR_API_EXPORT void registerPlugin(Falcor::PluginRegistry& registry)
{
    registry.registerClass<RenderPass, ReSTIRPTPass>();
    ScriptBindings::registerBinding(ReSTIRPTPass::registerBindings);
}

void ReSTIRPTPass::registerBindings(pybind11::module& m)
{
    pybind11::enum_<ShiftMapping> shiftMapping(m, "ShiftMapping");
    shiftMapping.value("Reconnection", ShiftMapping::Reconnection);
    shiftMapping.value("RandomReplay", ShiftMapping::RandomReplay);
    shiftMapping.value("Hybrid", ShiftMapping::Hybrid);

    pybind11::enum_<ReSTIRMISKind> misKind(m, "ReSTIRMISKind");
    misKind.value("Constant", ReSTIRMISKind::Constant);
    misKind.value("Talbot", ReSTIRMISKind::Talbot);
    misKind.value("Pairwise", ReSTIRMISKind::Pairwise);
    misKind.value("ConstantBinary", ReSTIRMISKind::ConstantBinary);
    misKind.value("ConstantBiased", ReSTIRMISKind::ConstantBiased);

    pybind11::enum_<PathSamplingMode> pathSamplingMode(m, "PathSamplingMode");
    pathSamplingMode.value("ReSTIR", PathSamplingMode::ReSTIR);
    pathSamplingMode.value("PathReuse", PathSamplingMode::PathReuse);
    pathSamplingMode.value("PathTracing", PathSamplingMode::PathTracing);

    pybind11::enum_<SpatialReusePattern> spatialReusePattern(m, "SpatialReusePattern");
    spatialReusePattern.value("Default", SpatialReusePattern::Default);
    spatialReusePattern.value("SmallWindow", SpatialReusePattern::SmallWindow);

    pybind11::class_<ReSTIRPTPass, RenderPass, ref<ReSTIRPTPass>> pass(m, "ReSTIRPTPass");
    pass.def("reset", &ReSTIRPTPass::reset);
    pass.def_property_readonly("pixelStats", &ReSTIRPTPass::getPixelStats);

    pass.def_property(
        "useFixedSeed",
        [](const ReSTIRPTPass* pt) { return pt->mParams.useFixedSeed ? true : false; },
        [](ReSTIRPTPass* pt, bool value) { pt->mParams.useFixedSeed = value ? 1 : 0; }
    );
    pass.def_property(
        "fixedSeed",
        [](const ReSTIRPTPass* pt) { return pt->mParams.fixedSeed; },
        [](ReSTIRPTPass* pt, uint32_t value) { pt->mParams.fixedSeed = value; }
    );
}

ReSTIRPTPass::ReSTIRPTPass(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    if (!mpDevice->isShaderModelSupported(ShaderModel::SM6_5))
        FALCOR_THROW("ReSTIRPTPass requires Shader Model 6.5 support.");
    if (!mpDevice->isFeatureSupported(Device::SupportedFeatures::RaytracingTier1_1))
        FALCOR_THROW("ReSTIRPTPass requires Raytracing Tier 1.1 support.");

    //mSERSupported = mpDevice->isFeatureSupported(Device::SupportedFeatures::ShaderExecutionReorderingAPI);

    parseProperties(props);
    validateOptions();

    // Maybe we don't need to manually init mNRooksPatternBuffer

    // Create sample generator.
    mpSampleGenerator = SampleGenerator::create(mpDevice, mStaticParams.sampleGenerator);

    // Create neighbor offset texture.
    mpNeighborOffsets = createNeighborOffsetTexture(kNeighborOffsetCount);

    // Create programs
    auto defines = mStaticParams.getDefines(*this);
    mpGeneratePaths = ComputePass::create(mpDevice, ProgramDesc().addShaderLibrary(kGeneratePathsFilename).csEntry("main"), defines, false);
    mpReflectTypes = ComputePass::create(mpDevice, ProgramDesc().addShaderLibrary(kReflectTypesFile).csEntry("main"), defines, false);
    //mpResolvePass = ComputePass::create(mpDevice, ProgramDesc().addShaderLibrary(kResolvePassFilename).csEntry("main"), defines, false);

    // TODO: should those new passes be created here or lazily?

    // Note: The other programs are lazily created in updatePrograms() because a scene needs to be present when creating them.

    mpPixelStats = std::make_unique<PixelStats>(mpDevice);
    mpPixelDebug = std::make_unique<PixelDebug>(mpDevice);
}

// which is updateDict(const Dictionary& dict) in the 2022 code
void ReSTIRPTPass::setProperties(const Properties& props)
{
    parseProperties(props);
    validateOptions();
    if (auto lightBVHSampler = dynamic_cast<LightBVHSampler*>(mpEmissiveSampler.get()))
        lightBVHSampler->setOptions(mLightBVHOptions);
    /*if (mpRTXDI)
        mpRTXDI->setOptions(mRTXDIOptions);*/
    mRecompile = true;
    mOptionsChanged = true;

    mParams.frameCount = 0;
    mAccumulatedShadowRayCount = 0;
    mAccumulatedClosestHitRayCount = 0;
    mAccumulatedRayCount = 0;
}

void ReSTIRPTPass::parseProperties(const Properties& props)
{
    for (const auto& [key, value] : props)
    {
        if (key == kSamplesPerPixel) mStaticParams.samplesPerPixel = value;
        else if (key == kMaxSurfaceBounces) mStaticParams.maxSurfaceBounces = value;
        else if (key == kMaxDiffuseBounces) mStaticParams.maxDiffuseBounces = value;
        else if (key == kMaxSpecularBounces) mStaticParams.maxSpecularBounces = value;
        else if (key == kMaxTransmissionBounces) mStaticParams.maxTransmissionBounces = value;
        else if (key == kAdjustShadingNormals) mStaticParams.adjustShadingNormals = value;
        else if (key == kLODBias) mParams.lodBias = value;
        else if (key == kSampleGenerator) mStaticParams.sampleGenerator = value;
        else if (key == kFixedSeed) { mParams.fixedSeed = value; mParams.useFixedSeed = true; }
        else if (key == kUseBSDFSampling) mStaticParams.useBSDFSampling = value;
        else if (key == kUseNEE) mStaticParams.useNEE = value;
        else if (key == kUseMIS) mStaticParams.useMIS = value;
        else if (key == kUseRussianRoulette) mStaticParams.useRussianRoulette = value;
        else if (key == kUseAlphaTest) mStaticParams.useAlphaTest = value;
        else if (key == kMaxNestedMaterials) mStaticParams.maxNestedMaterials = value;
        else if (key == kUseLightsInDielectricVolumes) mStaticParams.useLightsInDielectricVolumes = value;
        else if (key == kLimitTransmission) mStaticParams.limitTransmission = value;
        else if (key == kMaxTransmissionReflectionDepth) mStaticParams.maxTransmissionReflectionDepth = value;
        else if (key == kMaxTransmissionRefractionDepth) mStaticParams.maxTransmissionRefractionDepth = value;
        else if (key == kDisableCaustics) mStaticParams.disableCaustics = value;
        else if (key == kSpecularRoughnessThreshold) mParams.specularRoughnessThreshold = value;
        else if (key == kDisableDirectIllumination) mStaticParams.disableDirectIllumination = value;
        else if (key == kPrimaryLodMode)  mStaticParams.primaryLodMode = value;
        // Denoising parameters
        else if (key == kUseNRDDemodulation) mStaticParams.useNRDDemodulation = value;
        else if (key == kColorFormat) mStaticParams.colorFormat = value;
        else if (key == kMISHeuristic) mStaticParams.misHeuristic = value;
        else if (key == kMISPowerExponent) mStaticParams.misPowerExponent = value;
        else if (key == kEmissiveSampler) mStaticParams.emissiveSampler = value;
        else if (key == kLightBVHOptions) mLightBVHOptions = value;
        else if (key == kSpatialMisKind) mStaticParams.spatialMisKind = value;
        else if (key == kTemporalMisKind) mStaticParams.temporalMisKind = value;
        else if (key == kShiftStrategy) mStaticParams.shiftStrategy = value;
        else if (key == kRejectShiftBasedOnJacobian) mParams.rejectShiftBasedOnJacobian = value;
        else if (key == kJacobianRejectionThreshold) mParams.jacobianRejectionThreshold = value;
        else if (key == kNearFieldDistance) mParams.nearFieldDistance = value;
        else if (key == kTemporalHistoryLength) mTemporalHistoryLength = value;
        else if (key == kUseMaxHistory) mUseMaxHistory = value;
        else if (key == kSeedOffset) mSeedOffset = value;
        else if (key == kEnableTemporalReuse) mEnableTemporalReuse = value;
        else if (key == kEnableSpatialReuse) mEnableSpatialReuse = value;
        else if (key == kNumSpatialRounds) mNumSpatialRounds = value;
        else if (key == kPathSamplingMode) mStaticParams.pathSamplingMode = value;
        else if (key == kLocalStrategyType) mParams.localStrategyType = value;
        else if (key == kEnableTemporalReprojection) mEnableTemporalReprojection = value;
        else if (key == kNoResamplingForTemporalReuse) mNoResamplingForTemporalReuse = value;
        else if (key == kSpatialNeighborCount) mSpatialNeighborCount = value;
        else if (key == kFeatureBasedRejection) mFeatureBasedRejection = value;
        else if (key == kSpatialReusePattern) mSpatialReusePattern = value;
        else if (key == kSmallWindowRestirWindowRadius) mSmallWindowRestirWindowRadius = value;
        else if (key == kSpatialReuseRadius) mSpatialReuseRadius = value;
        else if (key == kUseDirectLighting) mUseDirectLighting = value;
        else if (key == kSeparatePathBSDF) mStaticParams.separatePathBSDF = value;
        else if (key == kCandidateSamples) mStaticParams.candidateSamples = value;
        else if (key == kTemporalUpdateForDynamicScene) mStaticParams.temporalUpdateForDynamicScene = value;
        else if (key == kEnableRayStats) mEnableRayStats = value;

        else
            logWarning("Unknown property '{}' in ReSTIRPTPass properties.", key);
    }

    if (props.has(kMaxSurfaceBounces))
    {
        // Initialize bounce counts to 'maxSurfaceBounces' if they weren't explicitly set.
        if (!props.has(kMaxDiffuseBounces))
            mStaticParams.maxDiffuseBounces = mStaticParams.maxSurfaceBounces;
        if (!props.has(kMaxSpecularBounces))
            mStaticParams.maxSpecularBounces = mStaticParams.maxSurfaceBounces;
        if (!props.has(kMaxTransmissionBounces))
            mStaticParams.maxTransmissionBounces = mStaticParams.maxSurfaceBounces;
    }
    else
    {
        // Initialize surface bounces.
        mStaticParams.maxSurfaceBounces =
            std::max(mStaticParams.maxDiffuseBounces, std::max(mStaticParams.maxSpecularBounces, mStaticParams.maxTransmissionBounces));
    }

    bool maxSurfaceBouncesNeedsAdjustment = mStaticParams.maxSurfaceBounces < mStaticParams.maxDiffuseBounces ||
                                            mStaticParams.maxSurfaceBounces < mStaticParams.maxSpecularBounces ||
                                            mStaticParams.maxSurfaceBounces < mStaticParams.maxTransmissionBounces;

    // Show a warning if maxSurfaceBounces will be adjusted in validateOptions().
    if (props.has(kMaxSurfaceBounces) && maxSurfaceBouncesNeedsAdjustment)
    {
        logWarning(
            "'{}' is set lower than '{}', '{}' or '{}' and will be increased.",
            kMaxSurfaceBounces,
            kMaxDiffuseBounces,
            kMaxSpecularBounces,
            kMaxTransmissionBounces
        );
    }
}

void ReSTIRPTPass::validateOptions()
{
    if (mParams.specularRoughnessThreshold < 0.f || mParams.specularRoughnessThreshold > 1.f)
    {
        logWarning("'specularRoughnessThreshold' has invalid value. Clamping to range [0,1].");
        mParams.specularRoughnessThreshold = std::clamp(mParams.specularRoughnessThreshold, 0.f, 1.f);
    }

    // Static parameters.
    if (mStaticParams.samplesPerPixel < 1 || mStaticParams.samplesPerPixel > kMaxSamplesPerPixel)
    {
        logWarning("'samplesPerPixel' must be in the range [1, {}]. Clamping to this range.", kMaxSamplesPerPixel);
        mStaticParams.samplesPerPixel = std::clamp(mStaticParams.samplesPerPixel, 1u, kMaxSamplesPerPixel);
    }

    auto clampBounces = [](uint32_t& bounces, const std::string& name)
    {
        if (bounces > kMaxBounces)
        {
            logWarning("'{}' exceeds the maximum supported bounces. Clamping to {}.", name, kMaxBounces);
            bounces = kMaxBounces;
        }
    };

    clampBounces(mStaticParams.maxSurfaceBounces, kMaxSurfaceBounces);
    clampBounces(mStaticParams.maxDiffuseBounces, kMaxDiffuseBounces);
    clampBounces(mStaticParams.maxSpecularBounces, kMaxSpecularBounces);
    clampBounces(mStaticParams.maxTransmissionBounces, kMaxTransmissionBounces);

    // Make sure maxSurfaceBounces is at least as many as any of diffuse, specular or transmission.
    uint32_t minSurfaceBounces =
        std::max(mStaticParams.maxDiffuseBounces, std::max(mStaticParams.maxSpecularBounces, mStaticParams.maxTransmissionBounces));
    mStaticParams.maxSurfaceBounces = std::max(mStaticParams.maxSurfaceBounces, minSurfaceBounces);
    if (mStaticParams.maxTransmissionReflectionDepth > mStaticParams.maxTransmissionBounces)
    {
        logWarning(
            "'maxTransmissionReflectionDepth' exceeds `maxTransmissionBounces`. Clamping to " +
            std::to_string(mStaticParams.maxTransmissionBounces)
        );
        mStaticParams.maxTransmissionReflectionDepth = mStaticParams.maxTransmissionBounces;
    }

    if (mStaticParams.maxTransmissionRefractionDepth > mStaticParams.maxTransmissionBounces)
    {
        logWarning(
            "'maxTransmissionRefractionDepth' exceeds `maxTransmissionBounces`. Clamping to " +
            std::to_string(mStaticParams.maxTransmissionBounces)
        );
        mStaticParams.maxTransmissionRefractionDepth = mStaticParams.maxTransmissionBounces;
    }

    if (mStaticParams.primaryLodMode == TexLODMode::RayCones)
    {
        logWarning("Unsupported tex lod mode. Defaulting to Mip0.");
        mStaticParams.primaryLodMode = TexLODMode::Mip0;
    }

    //if (mStaticParams.useSER && !mSERSupported)
    //{
    //    logWarning("Shader Execution Reordering (SER) is not supported on this device. Disabling SER.");
    //    mStaticParams.useSER = false;
    //}
}

Properties ReSTIRPTPass::getProperties() const
{
    if (auto lightBVHSampler = dynamic_cast<LightBVHSampler*>(mpEmissiveSampler.get()))
    {
        mLightBVHOptions = lightBVHSampler->getOptions();
    }

    Properties props;

    // Rendering parameters
    props[kSamplesPerPixel] = mStaticParams.samplesPerPixel;
    props[kMaxSurfaceBounces] = mStaticParams.maxSurfaceBounces;
    props[kMaxDiffuseBounces] = mStaticParams.maxDiffuseBounces;
    props[kMaxSpecularBounces] = mStaticParams.maxSpecularBounces;
    props[kMaxTransmissionBounces] = mStaticParams.maxTransmissionBounces;
    //props[kAdjustShadingNormals] = mStaticParams.adjustShadingNormals;  // is a material param
    props[kLODBias] = mParams.lodBias;

    // Sampling parameters
    props[kSampleGenerator] = mStaticParams.sampleGenerator;
    if (mParams.useFixedSeed)
        props[kFixedSeed] = mParams.fixedSeed;
    props[kUseBSDFSampling] = mStaticParams.useBSDFSampling;
    props[kUseRussianRoulette] = mStaticParams.useRussianRoulette;
    props[kUseNEE] = mStaticParams.useNEE;
    props[kUseMIS] = mStaticParams.useMIS;
    props[kMISHeuristic] = mStaticParams.misHeuristic;
    props[kMISPowerExponent] = mStaticParams.misPowerExponent;
    props[kEmissiveSampler] = mStaticParams.emissiveSampler;
    if (mStaticParams.emissiveSampler == EmissiveLightSamplerType::LightBVH)
        props[kLightBVHOptions] = mLightBVHOptions;
    props[kUseRussianRoulette] = mStaticParams.useRussianRoulette;
    //props[kUseRTXDI] = mStaticParams.useRTXDI;
    //props[kRTXDIOptions] = mRTXDIOptions;

    // Material parameters
    props[kUseAlphaTest] = mStaticParams.useAlphaTest;
    props[kAdjustShadingNormals] = mStaticParams.adjustShadingNormals;
    props[kMaxNestedMaterials] = mStaticParams.maxNestedMaterials;
    props[kUseLightsInDielectricVolumes] = mStaticParams.useLightsInDielectricVolumes;
    props[kLimitTransmission] = mStaticParams.limitTransmission;
    props[kMaxTransmissionReflectionDepth] = mStaticParams.maxTransmissionReflectionDepth;
    props[kMaxTransmissionRefractionDepth] = mStaticParams.maxTransmissionRefractionDepth;
    props[kDisableCaustics] = mStaticParams.disableCaustics;
    props[kSpecularRoughnessThreshold] = mParams.specularRoughnessThreshold;
    props[kDisableDirectIllumination] = mStaticParams.disableDirectIllumination;
    props[kPrimaryLodMode] = mStaticParams.primaryLodMode;
    props[kLODBias] = mParams.lodBias;

    // ReSTIR parameters
    props[kMISHeuristic] = mStaticParams.misHeuristic;
    props[kMISPowerExponent] = mStaticParams.misPowerExponent;
    props[kEmissiveSampler] = mStaticParams.emissiveSampler;
    if (mStaticParams.emissiveSampler == EmissiveLightSamplerType::LightBVH)
        props[kLightBVHOptions] = mLightBVHOptions;
    props[kSpatialMisKind] = mStaticParams.spatialMisKind;
    props[kTemporalMisKind] = mStaticParams.temporalMisKind;
    props[kShiftStrategy] = mStaticParams.shiftStrategy;
    props[kRejectShiftBasedOnJacobian] = mParams.rejectShiftBasedOnJacobian;
    props[kJacobianRejectionThreshold] = mParams.jacobianRejectionThreshold;
    props[kNearFieldDistance] = mParams.nearFieldDistance;
    props[kTemporalHistoryLength] = mTemporalHistoryLength;
    props[kUseMaxHistory] = mUseMaxHistory;
    props[kSeedOffset] = mSeedOffset;
    props[kEnableTemporalReuse] = mEnableSpatialReuse;
    props[kEnableSpatialReuse] = mEnableTemporalReuse;
    props[kNumSpatialRounds] = mNumSpatialRounds;
    props[kPathSamplingMode] = mStaticParams.pathSamplingMode;
    props[kLocalStrategyType] = mParams.localStrategyType;
    props[kEnableTemporalReprojection] = mEnableTemporalReprojection;
    props[kNoResamplingForTemporalReuse] = mNoResamplingForTemporalReuse;
    props[kSpatialNeighborCount] = mSpatialNeighborCount;
    props[kFeatureBasedRejection] = mFeatureBasedRejection;
    props[kSpatialReusePattern] = mSpatialReusePattern;
    props[kSmallWindowRestirWindowRadius] = mSmallWindowRestirWindowRadius;
    props[kSpatialReuseRadius] = mSpatialReuseRadius;
    props[kUseDirectLighting] = mUseDirectLighting;
    props[kSeparatePathBSDF] = mStaticParams.separatePathBSDF;
    props[kCandidateSamples] = mStaticParams.candidateSamples;
    props[kTemporalUpdateForDynamicScene] = mStaticParams.temporalUpdateForDynamicScene;
    props[kEnableRayStats] = mEnableRayStats;

    // Denoising parameters
    props[kUseNRDDemodulation] = mStaticParams.useNRDDemodulation;

    // Scheduling parameters
    //props[kUseSER] = mStaticParams.useSER;

    // Output parameters
    props[kOutputSize] = mOutputSizeSelection;
    if (mOutputSizeSelection == RenderPassHelpers::IOSize::Fixed)
        props[kFixedOutputSize] = mFixedOutputSize;
    props[kColorFormat] = mStaticParams.colorFormat;

    return props;
}

RenderPassReflection ReSTIRPTPass::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;
    const uint2 sz = RenderPassHelpers::calculateIOSize(mOutputSizeSelection, mFixedOutputSize, compileData.defaultTexDims);

    addRenderPassInputs(reflector, kInputChannels);
    addRenderPassOutputs(reflector, kOutputChannels, ResourceBindFlags::UnorderedAccess, sz);
    return reflector;
}

// Wasn't there in 2022 code
void ReSTIRPTPass::setFrameDim(const uint2 frameDim)
{
    auto prevFrameDim = mParams.frameDim;
    auto prevScreenTiles = mParams.screenTiles;

    mParams.frameDim = frameDim;
    if (mParams.frameDim.x > kMaxFrameDimension || mParams.frameDim.y > kMaxFrameDimension)
    {
        FALCOR_THROW("Frame dimensions up to {} pixels width/height are supported.", kMaxFrameDimension);
    }

    // Tile dimensions have to be powers-of-two.
    FALCOR_ASSERT(isPowerOf2(kScreenTileDim.x) && isPowerOf2(kScreenTileDim.y));
    FALCOR_ASSERT(kScreenTileDim.x == (1 << kScreenTileBits.x) && kScreenTileDim.y == (1 << kScreenTileBits.y));
    mParams.screenTiles = div_round_up(mParams.frameDim, kScreenTileDim);

    if (any(mParams.frameDim != prevFrameDim) || any(mParams.screenTiles != prevScreenTiles))
    {
        mVarsChanged = true;
    }
}

void ReSTIRPTPass::setScene(RenderContext* pRenderContext, const ref<Scene>& pScene)
{
    mpScene = pScene;
    mParams.frameCount = 0;
    // TODO: these 2 lines are new; does 2022 code put them somewhere else?
    mParams.frameDim = {};
    mParams.screenTiles = {};

    // Need to recreate the RTXDI module when the scene changes.
    // should be useless in ReSTIR PT
    mpRTXDI = nullptr;

    resetPrograms();
    resetLighting();

    if (mpScene)
    {
        if (pScene->hasGeometryType(Scene::GeometryType::Custom))
        {
            logWarning("ReSTIRPTPass: This render pass does not support custom primitives.");
        }

        // check if the scene is dynamic
        bool enableRobustSettingsByDefault = mpScene->hasAnimation() && mpScene->isAnimated();
        mParams.rejectShiftBasedOnJacobian = enableRobustSettingsByDefault;
        mStaticParams.temporalUpdateForDynamicScene = enableRobustSettingsByDefault;

        validateOptions();
    }
}

void ReSTIRPTPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (!beginFrame(pRenderContext, renderData))
        return;
    renderData.getDictionary()["enableScreenSpaceReSTIR"] = mUseDirectLighting;

    // Determine ReSTIR configs
    bool skipTemporalReuse = mReservoirFrameCount == 0;
    if (mStaticParams.pathSamplingMode != PathSamplingMode::ReSTIR)
        mStaticParams.candidateSamples = 1;
    if (mStaticParams.pathSamplingMode == PathSamplingMode::PathReuse)
    {
        mStaticParams.shiftStrategy = ShiftMapping::Reconnection;
        mEnableSpatialReuse = true;
    }
    if (mStaticParams.shiftStrategy == ShiftMapping::Hybrid)
    {
        // the ray tracing pass happens before spatial/temporal reuse,
        // so currently hybrid shift is only implemented for Pairwise and Talbot
        mStaticParams.spatialMisKind = ReSTIRMISKind::Pairwise;
        mStaticParams.temporalMisKind = ReSTIRMISKind::Talbot;
    }

    uint32_t numPasses = mStaticParams.pathSamplingMode == PathSamplingMode::PathTracing ? 1 : mStaticParams.samplesPerPixel;

    for (uint32_t restir_i = 0; restir_i < numPasses; restir_i++)
    {
        // some ordinary updates
        {
            // Update shader program specialization.
            updatePrograms();

            // Prepare resources.
            prepareResources(pRenderContext, renderData);

            // Prepare the path tracer parameter block.
            // This should be called after all resources have been created.
            prepareReSTIRPT(renderData);

            // Reset atomic counters.

            // Clear time output texture.

            if (const auto& texture = renderData[kOutputTime])
            {
                pRenderContext->clearUAV(texture->getUAV().get(), uint4(0));
            }

            {
                FALCOR_ASSERT(mpCounters);
                pRenderContext->clearUAV(mpCounters->getUAV().get(), uint4(0));

                mpReSTIRPTBlock->getRootVar()["gSppId"] = restir_i;
                mpReSTIRPTBlock->getRootVar()["gNumSpatialRounds"] = mNumSpatialRounds;

                if (restir_i == 0)
                    // Generate paths at primary hits.
                    generatePaths(pRenderContext, renderData, 0);

                // Launch main trace pass.
                FALCOR_ASSERT(mpTracePass);
                tracePass(pRenderContext, renderData, *mpTracePass, 0 /* sampleID */);
            }
        }

        if (mStaticParams.pathSamplingMode != PathSamplingMode::PathTracing)
        {
            // Launch restir merge pass.
            if (mStaticParams.pathSamplingMode == PathSamplingMode::ReSTIR)
            {
                if (mEnableTemporalReuse && !skipTemporalReuse)
                {
                    if (mStaticParams.shiftStrategy == ShiftMapping::Hybrid)
                        PathRetracePass(pRenderContext, restir_i, renderData, true, 0);
                    // a separate pass to trace rays for hybrid shift/random number replay
                    PathReusePass(pRenderContext, restir_i, renderData, true, 0, !mEnableSpatialReuse);
                }
            }
            else if (mStaticParams.pathSamplingMode == PathSamplingMode::PathReuse)
            {
                PathReusePass(pRenderContext, restir_i, renderData, false, -1, false);
            }

            if (mEnableSpatialReuse)
            {
                // multiple rounds?
                for (int spatialRoundId = 0; spatialRoundId < mNumSpatialRounds; spatialRoundId++)
                {
                    // a separate pass to trace rays for hybrid shift/random number replay
                    if (mStaticParams.shiftStrategy == ShiftMapping::Hybrid)
                        PathRetracePass(pRenderContext, restir_i, renderData, false, spatialRoundId);
                    PathReusePass(pRenderContext, restir_i, renderData, false, spatialRoundId, spatialRoundId == mNumSpatialRounds - 1);
                }
            }

            if (restir_i == numPasses - 1)
                mReservoirFrameCount++; // mark as at least one temporally reused frame

            if (mEnableTemporalReuse && mStaticParams.pathSamplingMode == PathSamplingMode::ReSTIR)
            {
                if ((!mEnableSpatialReuse || mNumSpatialRounds % 2 == 0))
                    pRenderContext->copyResource(mpTemporalReservoirs[restir_i].get(), mpOutputReservoirs.get());
                if (restir_i == numPasses - 1)
                    pRenderContext->copyResource(mpTemporalVBuffer.get(), renderData[kInputVBuffer].get());
            }
        }  // end of if not PT


    }  // end of for restir_i

    // moved to endFrame()
    //mParams.frameCount++;

    endFrame(pRenderContext, renderData);
}

void ReSTIRPTPass::renderUI(Gui::Widgets& widget)
{
    bool dirty = false;

    // Rendering options.
    dirty |= renderRenderingUI(widget);

    // Stats and debug options.
    renderStatsUI(widget);
    dirty |= renderDebugUI(widget);

    if (dirty)
    {
        validateOptions();
        mOptionsChanged = true;
    }
}

bool ReSTIRPTPass::renderRenderingUI(Gui::Widgets& widget)
{
    bool dirty = false;

    /// In Faclor 7.0, getGlobalClock() can only be called on a class that extends SampleApp
    /// e.g. class HelloDXR : public SampleApp
    /// But here ReSTIRPTPass extends RenderPass, and no obvious fix can handle this.
    /*
    if (mpScene && mpScene->hasAnimation())
    {
        if (gpFramework->getGlobalClock().isPaused())
        {
            if (widget.button("Resume Animation"))
                gpFramework->getGlobalClock().play();
        }
        else
        {
            if (widget.button("Pause Animation"))
                gpFramework->getGlobalClock().pause();
        }
    }
    */

    dirty |= widget.checkbox("Direct lighting (ReSTIR DI)", mUseDirectLighting);

    bool pathSamplingModeChanged = widget.dropdown("Path Sampling Mode", kPathSamplingModeList, reinterpret_cast<uint32_t&>(mStaticParams.pathSamplingMode));
    if (pathSamplingModeChanged)
    {
        if (mStaticParams.pathSamplingMode == PathSamplingMode::PathReuse)
        {
            mStaticParams.shiftStrategy = ShiftMapping::Reconnection;
            mStaticParams.separatePathBSDF = false;
        }
        else mStaticParams.separatePathBSDF = true;
    }

    //if (auto group = widget.group("Path Reuse Controls", true))
    {
        dirty |= pathSamplingModeChanged;

        if (mStaticParams.pathSamplingMode == PathSamplingMode::ReSTIR)
        {
            if (widget.button("Clean Reservoirs"))
            {
                mReservoirFrameCount = 0;
            }

            dirty |= widget.var("Candidate Samples", mStaticParams.candidateSamples, 1u, 64u);
            widget.tooltip("Number candidate samples for ReSTIR PT.\n");

            if (auto group = widget.group("Shift Mapping Options", true))
            {
                if (widget.dropdown("Shift Mapping", kShiftMappingList, reinterpret_cast<uint32_t&>(mStaticParams.shiftStrategy)))
                {
                    dirty = true;
                }

                dirty |= widget.checkbox("Reject Shift based on Jacobian (unbiased)", mParams.rejectShiftBasedOnJacobian);

                if (mParams.rejectShiftBasedOnJacobian)
                {
                    dirty |= widget.var("Shift rejection jacobian threshold", mParams.jacobianRejectionThreshold, 0.f, 100.f);
                }
            }
        }

        if (mStaticParams.pathSamplingMode == PathSamplingMode::ReSTIR && mStaticParams.shiftStrategy == ShiftMapping::Hybrid)
        {
            if (auto group = widget.group("Local Strategies", true))
            {
                bool enableRoughnessCondition = mParams.localStrategyType & (uint32_t)LocalStrategy::RoughnessCondition;
                bool enableDistanceCondition = mParams.localStrategyType & (uint32_t)LocalStrategy::DistanceCondition;

                dirty |= widget.checkbox("Roughness Condition", enableRoughnessCondition);
                dirty |= widget.checkbox("Distance Condition", enableDistanceCondition);

                if (dirty)
                {
                    mParams.localStrategyType = enableDistanceCondition << ((uint32_t)LocalStrategy::DistanceCondition - 1) | enableRoughnessCondition << ((uint32_t)LocalStrategy::RoughnessCondition - 1);
                }
            }

            if (auto group = widget.group("Classification thresholds", true))
            {
                dirty |= widget.var("Near field distance", mParams.nearFieldDistance, 0.f, 100.f);
                dirty |= widget.var("Specular roughness threshold", mParams.specularRoughnessThreshold, 0.f, 1.f);
            }
        }

        if (mStaticParams.pathSamplingMode == PathSamplingMode::ReSTIR)
        {
            dirty |= widget.checkbox("Spatial Reuse", mEnableSpatialReuse);
            dirty |= widget.checkbox("Temporal Reuse", mEnableTemporalReuse);
        }

        if (mStaticParams.pathSamplingMode == PathSamplingMode::PathReuse)
        {
            dirty |= widget.dropdown("Bekaert-Style Path Reuse Pattern", kPathReusePatternList, reinterpret_cast<uint32_t&>(mPathReusePattern));
        }
        else if (mStaticParams.pathSamplingMode == PathSamplingMode::ReSTIR && mEnableSpatialReuse)
        {
            if (auto group = widget.group("Spatial reuse controls", true))
            {
                dirty |= widget.var("Num Spatial Rounds", mNumSpatialRounds, 1, 5);
                dirty |= widget.dropdown("Spatial Reuse Pattern", kSpatialReusePatternList, reinterpret_cast<uint32_t&>(mSpatialReusePattern));
                dirty |= widget.checkbox("Feature-based rejection", mFeatureBasedRejection);

                if (SpatialReusePattern(mSpatialReusePattern) == SpatialReusePattern::SmallWindow)
                {
                    dirty |= widget.var("Window radius", mSmallWindowRestirWindowRadius, 0u, 32u);
                }
                else
                {
                    dirty |= widget.var("Spatial Neighbor Count", mSpatialNeighborCount, 0, 6);
                    dirty |= widget.var("Spatial Reuse Radius", mSpatialReuseRadius, 0.f, 100.f);
                }

                dirty |= widget.dropdown("Spatial Resampling MIS Kind", kReSTIRMISList, reinterpret_cast<uint32_t&>(mStaticParams.spatialMisKind));
                widget.tooltip("Current implementation only support pairwise MIS for hybird shift.\n");
            }
        }

        if (mStaticParams.pathSamplingMode == PathSamplingMode::ReSTIR && mEnableTemporalReuse)
        {
            if (auto group = widget.group("Temporal reuse controls", true))
            {
                dirty |= widget.var("Temporal History Length", mTemporalHistoryLength, 0, 100);
                dirty |= widget.checkbox("Use M capping", mUseMaxHistory);
                dirty |= widget.checkbox("Temporal Reprojection", mEnableTemporalReprojection);
                dirty |= widget.checkbox("Temporal Update for Dynamic Scenes", mStaticParams.temporalUpdateForDynamicScene);
                widget.tooltip("Resample cached radiance in reconnection vertex in temporal reservoir for dynamic scenes (eliminate lags).");
                dirty |= widget.checkbox("Disable Resampling in Temporal Reuse", mNoResamplingForTemporalReuse);
                dirty |= widget.dropdown("Temporal Resampling MIS Kind", kReSTIRMISList2, reinterpret_cast<uint32_t&>(mStaticParams.temporalMisKind));
                widget.tooltip("Current implementation only support Talbot MIS for hybird shift.\n");
            }
        }
    }

    //if (auto group = widget.group("Shared Path Sampler Options", true))

    {
        dirty |= widget.var("Samples/pixel", mStaticParams.samplesPerPixel, 1u, kMaxSamplesPerPixel);

        widget.tooltip("Number of samples per pixel. One path is traced for each sample.\n");

        dirty |= widget.checkbox("Use Sampled BSDFs", mStaticParams.separatePathBSDF);
        widget.tooltip("Control whether to use mixture BSDF or sampled BSDF in path tracing/path reuse.\n");

        if (widget.var("Max bounces (override all)", mStaticParams.maxSurfaceBounces, 0u, kMaxBounces))
        {
            // Allow users to change the max surface bounce parameter in the UI to clamp all other surface bounce parameters.
            mStaticParams.maxDiffuseBounces = mStaticParams.maxSurfaceBounces;
            mStaticParams.maxSpecularBounces = mStaticParams.maxSurfaceBounces;
            mStaticParams.maxTransmissionBounces = mStaticParams.maxSurfaceBounces;
            dirty = true;
        }

        if (widget.var("Max surface bounces", mStaticParams.maxSurfaceBounces, 0u, kMaxBounces))
        {
            // Allow users to change the max surface bounce parameter in the UI to clamp all other surface bounce parameters.
            mStaticParams.maxDiffuseBounces = std::min(mStaticParams.maxDiffuseBounces, mStaticParams.maxSurfaceBounces);
            mStaticParams.maxSpecularBounces = std::min(mStaticParams.maxSpecularBounces, mStaticParams.maxSurfaceBounces);
            mStaticParams.maxTransmissionBounces = std::min(mStaticParams.maxTransmissionBounces, mStaticParams.maxSurfaceBounces);
            dirty = true;
        }
        widget.tooltip("Maximum number of surface bounces (diffuse + specular + transmission).\n"
            "Note that specular reflection events from a material with a roughness greater than specularRoughnessThreshold are also classified as diffuse events.");

        dirty |= widget.var("Max diffuse bounces", mStaticParams.maxDiffuseBounces, 0u, kMaxBounces);
        widget.tooltip("Maximum number of diffuse bounces.\n0 = direct only\n1 = one indirect bounce etc.");

        dirty |= widget.var("Max specular bounces", mStaticParams.maxSpecularBounces, 0u, kMaxBounces);
        widget.tooltip("Maximum number of specular bounces.\n0 = direct only\n1 = one indirect bounce etc.");

        dirty |= widget.var("Max transmission bounces", mStaticParams.maxTransmissionBounces, 0u, kMaxBounces);
        widget.tooltip("Maximum number of transmission bounces.\n0 = no transmission\n1 = one transmission bounce etc.");

        // Sampling options.

        if (widget.dropdown("Sample generator", SampleGenerator::getGuiDropdownList(), mStaticParams.sampleGenerator))
        {
            mpSampleGenerator = SampleGenerator::create(mpDevice, mStaticParams.sampleGenerator);
            dirty = true;
        }

        dirty |= widget.checkbox("BSDF importance sampling", mStaticParams.useBSDFSampling);
        widget.tooltip("BSDF importance sampling should normally be enabled.\n\n"
            "If disabled, cosine-weighted hemisphere sampling is used for debugging purposes");

        dirty |= widget.checkbox("Disable direct illumination", mStaticParams.disableDirectIllumination);
        widget.tooltip("If enabled, incoming radiance is collected starting from first order indirect.");

        dirty |= widget.checkbox("Russian roulette", mStaticParams.useRussianRoulette);
        widget.tooltip("Use russian roulette to terminate low throughput paths.");

        dirty |= widget.checkbox("Next-event estimation (NEE)", mStaticParams.useNEE);
        widget.tooltip("Use next-event estimation.\nThis option enables direct illumination sampling at each path vertex.");

        if (mStaticParams.useNEE)
        {
            dirty |= widget.checkbox("Multiple importance sampling (MIS)", mStaticParams.useMIS);
            widget.tooltip("When enabled, BSDF sampling is combined with light sampling for the environment map and emissive lights.\n"
                "Note that MIS has currently no effect on analytic lights.");

            if (mStaticParams.useMIS)
            {
                dirty |= widget.dropdown("MIS heuristic", kMISHeuristicList, reinterpret_cast<uint32_t&>(mStaticParams.misHeuristic));

                if (mStaticParams.misHeuristic == MISHeuristic::PowerExp)
                {
                    dirty |= widget.var("MIS power exponent", mStaticParams.misPowerExponent, 0.01f, 10.f);
                }
            }

            if (mpScene && mpScene->useEmissiveLights())
            {
                if (auto group = widget.group("Emissive sampler"))
                {
                    if (widget.dropdown("Emissive sampler", kEmissiveSamplerList, (uint32_t&)mStaticParams.emissiveSampler))
                    {
                        resetLighting();
                        dirty = true;
                    }
                    widget.tooltip("Selects which light sampler to use for importance sampling of emissive geometry.", true);

                    if (mpEmissiveSampler)
                    {
                        if (mpEmissiveSampler->renderUI(group)) mOptionsChanged = true;
                    }
                }
            }
        }
    }

    if (auto group = widget.group("Material controls"))
    {
        dirty |= widget.checkbox("Alpha test", mStaticParams.useAlphaTest);
        widget.tooltip("Use alpha testing on non-opaque triangles.");

        dirty |= widget.checkbox("Adjust shading normals on secondary hits", mStaticParams.adjustShadingNormals);
        widget.tooltip("Enables adjustment of the shading normals to reduce the risk of black pixels due to back-facing vectors.\nDoes not apply to primary hits which is configured in GBuffer.", true);

        dirty |= widget.var("Max nested materials", mStaticParams.maxNestedMaterials, 2u, 4u);
        widget.tooltip("Maximum supported number of nested materials.");

        dirty |= widget.checkbox("Use deterministic BSDF evaluations", mStaticParams.useDeterministicBSDF);
        widget.tooltip("If disabled, BSDF evaluations of BSDF-sampled directions are only correct on expectation.", true);

        dirty |= widget.checkbox("Use lights in dielectric volumes", mStaticParams.useLightsInDielectricVolumes);
        widget.tooltip("Use lights inside of volumes (transmissive materials). We typically don't want this because lights are occluded by the interface.");

        dirty |= widget.checkbox("Limit transmission", mStaticParams.limitTransmission);
        widget.tooltip("Limit specular transmission by handling reflection/refraction events only up to a given transmission depth.");

        if (mStaticParams.limitTransmission)
        {
            dirty |= widget.var("Max transmission reflection depth", mStaticParams.maxTransmissionReflectionDepth, 0u, kMaxBounces);
            widget.tooltip("Maximum transmission depth at which to sample specular reflection.\n"
                "0: Reflection is never sampled.\n"
                "1: Reflection is only sampled on primary hits.\n"
                "N: Reflection is only sampled on the first N hits.");

            dirty |= widget.var("Max transmission refraction depth", mStaticParams.maxTransmissionRefractionDepth, 0u, kMaxBounces);
            widget.tooltip("Maximum transmission depth at which to sample specular refraction (after that, IoR is set to 1).\n"
                "0: Refraction is never sampled.\n"
                "1: Refraction is only sampled on primary hits.\n"
                "N: Refraction is only sampled on the first N hits.");
        }

        dirty |= widget.checkbox("Disable caustics", mStaticParams.disableCaustics);
        widget.tooltip("Disable sampling of caustic light paths (i.e. specular events after diffuse events).");

        dirty |= widget.checkbox("Disable direct illumination", mStaticParams.disableDirectIllumination);
        widget.tooltip("Disable computation of all direct illumination.");

        dirty |= widget.var("TexLOD bias", mParams.lodBias, -16.f, 16.f, 0.01f);
    }

    if (auto group = widget.group("Denoiser options"))
    {
        dirty |= widget.checkbox("Use NRD demodulation", mStaticParams.useNRDDemodulation);
        widget.tooltip("Global switch for NRD demodulation");
    }

    if (auto group = widget.group("Output options"))
    {
        dirty |= widget.dropdown("Color format", kColorFormatList, (uint32_t&)mStaticParams.colorFormat);
        widget.tooltip("Selects the color format used for internal per-sample color and denoiser buffers");
    }

    if (dirty)
        mRecompile = true;
    return dirty;
}


bool ReSTIRPTPass::renderDebugUI(Gui::Widgets& widget)
{
    bool dirty = false;

    if (auto group = widget.group("Debugging"))
    {
        dirty |= group.checkbox("Use fixed seed", mParams.useFixedSeed);
        group.tooltip(
            "Forces a fixed random seed for each frame.\n\n"
            "This should produce exactly the same image each frame, which can be useful for debugging."
        );
        if (mParams.useFixedSeed)
        {
            dirty |= group.var("Seed", mParams.fixedSeed);
        }

        mpPixelDebug->renderUI(group);
    }

    return dirty;
}

void ReSTIRPTPass::renderStatsUI(Gui::Widgets& widget)
{
    if (auto g = widget.group("Statistics"))
    {
        // Show ray stats
        mpPixelStats->renderUI(g);
    }
}

bool ReSTIRPTPass::onMouseEvent(const MouseEvent& mouseEvent)
{
    return mpPixelDebug->onMouseEvent(mouseEvent);
}

void ReSTIRPTPass::reset()
{
    mParams.frameCount = 0;
}

ReSTIRPTPass::TracePass::TracePass(
    ref<Device> pDevice,
    const std::string& name,
    const std::string& passDefine,
    const ref<Scene>& pScene,
    const DefineList& defines,
    const TypeConformanceList& globalTypeConformances
)
    : name(name), passDefine(passDefine)
{
    const uint32_t kRayTypeScatter = 0;
    const uint32_t kMissScatter = 0;

    bool useSER = defines.at("USE_SER") == "1";

    ProgramDesc desc;
    desc.addShaderModules(pScene->getShaderModules());
    desc.addShaderLibrary(kTracePassFilename);
    if (pDevice->getType() == Device::Type::D3D12 && useSER)
        desc.addCompilerArguments({"-Xdxc", "-enable-lifetime-markers"});
    desc.setMaxPayloadSize(160); // This is conservative but the required minimum is 140 bytes.
    desc.setMaxAttributeSize(pScene->getRaytracingMaxAttributeSize());
    desc.setMaxTraceRecursionDepth(1);
    if (!pScene->hasProceduralGeometry())
        desc.setRtPipelineFlags(RtPipelineFlags::SkipProceduralPrimitives);

    // Create ray tracing binding table.
    pBindingTable = RtBindingTable::create(1, 1, pScene->getGeometryCount());

    // Specify entry point for raygen and miss shaders.
    // The raygen shader needs type conformances for *all* materials in the scene.
    // The miss shader doesn't need need any type conformances because it does not use materials.
    pBindingTable->setRayGen(desc.addRayGen("rayGen", globalTypeConformances));
    pBindingTable->setMiss(kMissScatter, desc.addMiss("scatterMiss"));

    // Specify hit group entry points for every combination of geometry and material type.
    // The code for each hit group gets specialized for the actual types it's operating on.
    // First query which material types the scene has.
    auto materialTypes = pScene->getMaterialSystem().getMaterialTypes();

    for (const auto materialType : materialTypes)
    {
        auto typeConformances = pScene->getMaterialSystem().getTypeConformances(materialType);

        // Add hit groups for triangles.
        if (auto geometryIDs = pScene->getGeometryIDs(Scene::GeometryType::TriangleMesh, materialType); !geometryIDs.empty())
        {
            auto shaderID =
                desc.addHitGroup("scatterTriangleClosestHit", "scatterTriangleAnyHit", "", typeConformances, to_string(materialType));
            pBindingTable->setHitGroup(kRayTypeScatter, geometryIDs, shaderID);
        }

        // Add hit groups for displaced triangle meshes.
        if (auto geometryIDs = pScene->getGeometryIDs(Scene::GeometryType::DisplacedTriangleMesh, materialType); !geometryIDs.empty())
        {
            auto shaderID = desc.addHitGroup(
                "scatterDisplacedTriangleMeshClosestHit", "", "displacedTriangleMeshIntersection", typeConformances, to_string(materialType)
            );
            pBindingTable->setHitGroup(kRayTypeScatter, geometryIDs, shaderID);
        }

        // Add hit groups for curves.
        if (auto geometryIDs = pScene->getGeometryIDs(Scene::GeometryType::Curve, materialType); !geometryIDs.empty())
        {
            auto shaderID = desc.addHitGroup("scatterCurveClosestHit", "", "curveIntersection", typeConformances, to_string(materialType));
            pBindingTable->setHitGroup(kRayTypeScatter, geometryIDs, shaderID);
        }

        // Add hit groups for SDF grids.
        if (auto geometryIDs = pScene->getGeometryIDs(Scene::GeometryType::SDFGrid, materialType); !geometryIDs.empty())
        {
            auto shaderID =
                desc.addHitGroup("scatterSdfGridClosestHit", "", "sdfGridIntersection", typeConformances, to_string(materialType));
            pBindingTable->setHitGroup(kRayTypeScatter, geometryIDs, shaderID);
        }
    }

    pProgram = Program::create(pDevice, desc, defines);
}

void ReSTIRPTPass::TracePass::prepareProgram(ref<Device> pDevice, const DefineList& defines)
{
    FALCOR_ASSERT(pProgram != nullptr && pBindingTable != nullptr);
    pProgram->setDefines(defines);
    if (!passDefine.empty())
        pProgram->addDefine(passDefine);
    pVars = RtProgramVars::create(pDevice, pProgram, pBindingTable);
}

void ReSTIRPTPass::resetPrograms()
{
    // these 3 passes are  std::unique_ptr<TracePass> instead of ref<ComputePass>
    mpTracePass = nullptr;
    mpTraceDeltaReflectionPass = nullptr;
    mpTraceDeltaTransmissionPass = nullptr;

    mpGeneratePaths = nullptr;
    mpReflectTypes = nullptr;

    mRecompile = true;
}

void ReSTIRPTPass::updatePrograms()
{
    FALCOR_ASSERT(mpScene);

    if (mRecompile == false)
        return;

    mStaticParams.rcDataOfflineMode = mSpatialNeighborCount > 3 && mStaticParams.shiftStrategy == ShiftMapping::Hybrid;

    // If we get here, a change that require recompilation of shader programs has occurred.
    // This may be due to change of scene defines, type conformances, shader modules, or other changes that require recompilation.
    // When type conformances and/or shader modules change, the programs need to be recreated. We assume programs have been reset upon such
    // changes. When only defines have changed, it is sufficient to update the existing programs and recreate the program vars.

    auto defines = mStaticParams.getDefines(*this);
    auto globalTypeConformances = mpScene->getTypeConformances();

    // Create trace pass.
    if (!mpTracePass)
        mpTracePass = std::make_unique<TracePass>(mpDevice, "tracePass", "", mpScene, defines, globalTypeConformances);

    mpTracePass->prepareProgram(mpDevice, defines);

    // Create specialized trace passes.
    if (mOutputNRDAdditionalData)
    {
        if (!mpTraceDeltaReflectionPass)
            mpTraceDeltaReflectionPass = std::make_unique<TracePass>(
                mpDevice, "traceDeltaReflectionPass", "DELTA_REFLECTION_PASS", mpScene, defines, globalTypeConformances
            );
        if (!mpTraceDeltaTransmissionPass)
            mpTraceDeltaTransmissionPass = std::make_unique<TracePass>(
                mpDevice, "traceDeltaTransmissionPass", "DELTA_TRANSMISSION_PASS", mpScene, defines, globalTypeConformances
            );

        mpTraceDeltaReflectionPass->prepareProgram(mpDevice, defines);
        mpTraceDeltaTransmissionPass->prepareProgram(mpDevice, defines);
    }

    // Create compute passes.
    ProgramDesc baseDesc;
    baseDesc.addShaderModules(mpScene->getShaderModules());
    baseDesc.addTypeConformances(globalTypeConformances);

    if (!mpGeneratePaths)
    {
        ProgramDesc desc = baseDesc;
        desc.addShaderLibrary(kGeneratePathsFilename).csEntry("main");
        mpGeneratePaths = ComputePass::create(mpDevice, desc, defines, false);
    }
    if (!mpReflectTypes)
    {
        ProgramDesc desc = baseDesc;
        desc.addShaderLibrary(kReflectTypesFile).csEntry("main");
        mpReflectTypes = ComputePass::create(mpDevice, desc, defines, false);
    }

    // lambda function that update defines and reset vars
    auto preparePass = [&](ref<ComputePass> pass)
    {
        // Note that we must use set instead of add defines to replace any stale state.
        pass->getProgram()->setDefines(defines);

        // Recreate program vars. This may trigger recompilation if needed.
        // Note that program versions are cached, so switching to a previously used specialization is faster.
        pass->setVars(nullptr);
    };

    preparePass(mpGeneratePaths);
    //preparePass(mpTracePass);
    preparePass(mpReflectTypes);
    preparePass(mpSpatialPathRetracePass);
    preparePass(mpTemporalPathRetracePass);
    preparePass(mpSpatialReusePass);
    preparePass(mpTemporalReusePass);
    preparePass(mpComputePathReuseMISWeightsPass);

    mVarsChanged = true;
    mRecompile = false;
}

void ReSTIRPTPass::prepareResources(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Compute allocation requirements for paths and output samples.
    // Note that the sample buffers are padded to whole tiles, while the max path count depends on actual frame dimension.
    // If we don't have a fixed sample count, assume the worst case.
    uint32_t tileCount = mParams.screenTiles.x * mParams.screenTiles.y;
    const uint32_t reservoirCount = tileCount * kScreenTileDim.x * kScreenTileDim.y;
    const uint32_t screenPixelCount = mParams.frameDim.x * mParams.frameDim.y;
    const uint32_t sampleCount = reservoirCount; // we are effectively only using 1spp for ReSTIR

    // Allocate output sample offset buffer if needed.
    // This buffer stores the output offset to where the samples for each pixel are stored consecutively.
    // The offsets are local to the current tile, so 16-bit format is sufficient and reduces bandwidth usage.
    /// Not ther in 2022 code
    /*
    if (!mFixedSampleCount)
    {
        if (!mpSampleOffset || mpSampleOffset->getWidth() != mParams.frameDim.x || mpSampleOffset->getHeight() != mParams.frameDim.y)
        {
            FALCOR_ASSERT(kScreenTileDim.x * kScreenTileDim.y * kMaxSamplesPerPixel <= (1u << 16));
            mpSampleOffset = mpDevice->createTexture2D(
                mParams.frameDim.x,
                mParams.frameDim.y,
                ResourceFormat::R16Uint,
                1,
                1,
                nullptr,
                ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess
            );
            mVarsChanged = true;
        }
    }
    */

    auto var = mpReflectTypes->getRootVar();

    /// NOTE: MemoryType::DeviceLocal was Buffer::CpuAccess::None in 2022 code
    /// They both mean buffer is stored on device and CPU has no access
    if (mStaticParams.pathSamplingMode != PathSamplingMode::PathTracing)
    {
        if (mStaticParams.shiftStrategy == ShiftMapping::Hybrid &&
            (!mReconnectionDataBuffer || mStaticParams.rcDataOfflineMode && mReconnectionDataBuffer->getElementSize() != 512 ||
             !mStaticParams.rcDataOfflineMode && mReconnectionDataBuffer->getElementSize() != 256))
        {
            mReconnectionDataBuffer = mpDevice->createStructuredBuffer(
                var["reconnectionDataBuffer"],
                reservoirCount,
                ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                MemoryType::DeviceLocal,
                nullptr,
                false
            );
            // printf("rcDataSize size: %d\n", mReconnectionDataBuffer->getElementSize());
        }
        if (mStaticParams.shiftStrategy != ShiftMapping::Hybrid)
            mReconnectionDataBuffer = nullptr;

        uint32_t baseReservoirSize = 88;
        uint32_t pathTreeReservoirSize = 128;

        if (mpOutputReservoirs &&
            (mStaticParams.pathSamplingMode == PathSamplingMode::PathReuse &&
                 mpOutputReservoirs->getElementSize() != pathTreeReservoirSize ||
             mStaticParams.pathSamplingMode != PathSamplingMode::PathReuse && mpOutputReservoirs->getElementSize() != baseReservoirSize ||
             mpTemporalReservoirs.size() != mStaticParams.samplesPerPixel && mStaticParams.pathSamplingMode != PathSamplingMode::PathReuse))
        {
            mpOutputReservoirs = mpDevice->createStructuredBuffer(
                var["outputReservoirs"],
                reservoirCount,
                ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                MemoryType::DeviceLocal,
                nullptr,
                false
            );
            // printf("reservoir size: %d\n", mpOutputReservoirs->getElementSize());

            if (mStaticParams.pathSamplingMode != PathSamplingMode::PathReuse)
            {
                mpTemporalReservoirs.resize(mStaticParams.samplesPerPixel);
                for (uint32_t i = 0; i < mStaticParams.samplesPerPixel; i++)
                    mpTemporalReservoirs[i] = mpDevice->createStructuredBuffer(
                        var["outputReservoirs"],
                        reservoirCount,
                        ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                        MemoryType::DeviceLocal,
                        nullptr,
                        false
                    );
            }
            mVarsChanged = true;
        }

        if (mStaticParams.pathSamplingMode == PathSamplingMode::PathReuse)
        {
            if (!mPathReuseMISWeightBuffer)
            {
                mPathReuseMISWeightBuffer = mpDevice->createStructuredBuffer(
                    var["misWeightBuffer"],
                    reservoirCount,
                    ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                    MemoryType::DeviceLocal,
                    nullptr,
                    false
                );
                mVarsChanged = true;
            }
            mpTemporalReservoirs.clear();
        }
        else
            mPathReuseMISWeightBuffer = nullptr;

        // Allocate path buffers.
        if (!mpOutputReservoirs || reservoirCount != mpOutputReservoirs->getElementCount())
        {
            mpOutputReservoirs = mpDevice->createStructuredBuffer(
                var["outputReservoirs"],
                reservoirCount,
                ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                MemoryType::DeviceLocal,
                nullptr,
                false
            );
            // printf("reservoir size: %d\n", mpOutputReservoirs->getElementSize());

            if (mStaticParams.pathSamplingMode == PathSamplingMode::PathReuse)
            {
                mPathReuseMISWeightBuffer = mpDevice->createStructuredBuffer(
                    var["misWeightBuffer"],
                    reservoirCount,
                    ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                    MemoryType::DeviceLocal,
                    nullptr,
                    false
                );
            }
            else
            {
                mpTemporalReservoirs.resize(mStaticParams.samplesPerPixel);
                for (uint32_t i = 0; i < mStaticParams.samplesPerPixel; i++)
                    mpTemporalReservoirs[i] = mpDevice->createStructuredBuffer(
                        var["outputReservoirs"],
                        reservoirCount,
                        ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                        MemoryType::DeviceLocal,
                        nullptr,
                        false
                    );
            }
            mVarsChanged = true;
        }
    }

    if (!mpTemporalVBuffer || mpTemporalVBuffer->getHeight() != mParams.frameDim.y || mpTemporalVBuffer->getWidth() != mParams.frameDim.x)
    {
        mpTemporalVBuffer = mpDevice->createTexture2D(mParams.frameDim.x, mParams.frameDim.y, mpScene->getHitInfo().getFormat(), 1, 1);
    }
}

void ReSTIRPTPass::prepareReSTIRPT(const RenderData& renderData)
{
    // Create path tracer parameter block if needed.
    if (!mpReSTIRPTBlock || mVarsChanged)
    {
        auto reflector = mpReflectTypes->getProgram()->getReflector()->getParameterBlock("pathTracer");
        mpReSTIRPTBlock = ParameterBlock::create(mpDevice, reflector);
        FALCOR_ASSERT(mpReSTIRPTBlock);
        mVarsChanged = true;
    }

    // Bind resources.
    auto var = mpReSTIRPTBlock->getRootVar();
    bindShaderData(var, renderData, true, false);
    var["outputReservoirs"] = mpOutputReservoirs;
    var["directLighting"] = renderData.getTexture(kInputDirectLighting);

}

void ReSTIRPTPass::resetLighting()
{
    // Retain the options for the emissive sampler.
    if (auto lightBVHSampler = dynamic_cast<LightBVHSampler*>(mpEmissiveSampler.get()))
    {
        mLightBVHOptions = lightBVHSampler->getOptions();
    }

    mpEmissiveSampler = nullptr;
    mpEnvMapSampler = nullptr;
    mRecompile = true;
}

void ReSTIRPTPass::prepareMaterials(RenderContext* pRenderContext)
{
    // This functions checks for scene changes that require shader recompilation.
    // Whenever materials or geometry is added/removed to the scene, we reset the shader programs to trigger
    // recompilation with the correct defines, type conformances, shader modules, and binding table.

    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::RecompileNeeded) ||
        is_set(mpScene->getUpdates(), Scene::UpdateFlags::GeometryChanged))
    {
        resetPrograms();
    }
}

bool ReSTIRPTPass::prepareLighting(RenderContext* pRenderContext)
{
    bool lightingChanged = false;

    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::RenderSettingsChanged))
    {
        lightingChanged = true;
        mRecompile = true;
    }

    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::SDFGridConfigChanged))
    {
        mRecompile = true;
    }

    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::EnvMapChanged))
    {
        mpEnvMapSampler = nullptr;
        lightingChanged = true;
        mRecompile = true;
    }

    if (mpScene->useEnvLight())
    {
        if (!mpEnvMapSampler)
        {
            mpEnvMapSampler = std::make_unique<EnvMapSampler>(mpDevice, mpScene->getEnvMap());
            lightingChanged = true;
            mRecompile = true;
        }
    }
    else
    {
        if (mpEnvMapSampler)
        {
            mpEnvMapSampler = nullptr;
            lightingChanged = true;
            mRecompile = true;
        }
    }

    // Request the light collection if emissive lights are enabled.
    if (mpScene->getRenderSettings().useEmissiveLights)
    {
        mpScene->getLightCollection(pRenderContext);
    }

    if (mpScene->useEmissiveLights())
    {
        if (!mpEmissiveSampler)
        {
            const auto& pLights = mpScene->getLightCollection(pRenderContext);
            FALCOR_ASSERT(pLights && pLights->getActiveLightCount(pRenderContext) > 0);
            FALCOR_ASSERT(!mpEmissiveSampler);

            switch (mStaticParams.emissiveSampler)
            {
            case EmissiveLightSamplerType::Uniform:
                mpEmissiveSampler = std::make_unique<EmissiveUniformSampler>(pRenderContext, mpScene);
                break;
            case EmissiveLightSamplerType::LightBVH:
                mpEmissiveSampler = std::make_unique<LightBVHSampler>(pRenderContext, mpScene, mLightBVHOptions);
                break;
            case EmissiveLightSamplerType::Power:
                mpEmissiveSampler = std::make_unique<EmissivePowerSampler>(pRenderContext, mpScene);
                break;
            default:
                FALCOR_THROW("Unknown emissive light sampler type");
            }
            lightingChanged = true;
            mRecompile = true;
        }
    }
    else
    {
        if (mpEmissiveSampler)
        {
            // Retain the options for the emissive sampler.
            if (auto lightBVHSampler = dynamic_cast<LightBVHSampler*>(mpEmissiveSampler.get()))
            {
                mLightBVHOptions = lightBVHSampler->getOptions();
            }

            mpEmissiveSampler = nullptr;
            lightingChanged = true;
            mRecompile = true;
        }
    }

    if (mpEmissiveSampler)
    {
        lightingChanged |= mpEmissiveSampler->update(pRenderContext);
        auto defines = mpEmissiveSampler->getDefines();
        if (mpTracePass && mpTracePass->pProgram->addDefines(defines))
            mRecompile = true;
    }

    return lightingChanged;
}

void ReSTIRPTPass::setNRDData(const ShaderVar& var, const RenderData& renderData) const
{
    var["primaryHitEmission"] = renderData.getTexture(kOutputNRDEmission);
    var["primaryHitDiffuseReflectance"] = renderData.getTexture(kOutputNRDDiffuseReflectance);
    var["primaryHitSpecularReflectance"] = renderData.getTexture(kOutputNRDSpecularReflectance);
}

void ReSTIRPTPass::bindShaderData(const ShaderVar& var, const RenderData& renderData, bool isPathTracer, bool isPathGenerator) const
{
    // Bind static resources that don't change per frame.
    if (mVarsChanged)
    {
        if (isPathTracer && mpEnvMapSampler)
            mpEnvMapSampler->bindShaderData(var["envMapSampler"]);

        // weren't there in 2022
        //var["sampleOffset"] = mpSampleOffset; // Can be nullptr
        //var["sampleColor"] = mpSampleColor;
        //var["sampleGuideData"] = mpSampleGuideData;
    }

    var["params"].setBlob(mParams);
    var["vbuffer"] = renderData.getTexture(kInputVBuffer);
    // var["viewDir"] = pViewDir;         // Can be nullptr
    // var["sampleCount"] = pSampleCount; // Can be nullptr
    var["outputColor"] = renderData.getTexture(kOutputColor);

    // Bind runtime data.
    //setNRDData(var["outputNRD"], renderData);
    if (mOutputNRDData && isPathTracer)
    {
        setNRDData(var["outputNRD"], renderData);
        ///< Output resolved diffuse color in .rgb and hit distance in .a for NRD. Only valid if kOutputNRDData == true.
        var["outputNRDDiffuseRadianceHitDist"] = renderData.getTexture(kOutputNRDDiffuseRadianceHitDist);
        ///< Output resolved specular color in .rgb and hit distance in .a for NRD. Only valid if kOutputNRDData == true.
        var["outputNRDSpecularRadianceHitDist"] = renderData.getTexture(kOutputNRDSpecularRadianceHitDist);
        ///< Output resolved residual color in .rgb and hit distance in .a for NRD. Only valid if kOutputNRDData == true.
        var["outputNRDResidualRadianceHitDist"] = renderData.getTexture(kOutputNRDResidualRadianceHitDist);
    }

    if (isPathTracer)
    {
        var["isLastRound"] = !mEnableSpatialReuse && !mEnableTemporalReuse;
        var["useDirectLighting"] = mUseDirectLighting;
        var["kUseEnvLight"] = mpScene->useEnvLight();
        var["kUseEmissiveLights"] = mpScene->useEmissiveLights();
        var["kUseAnalyticLights"] = mpScene->useAnalyticLights();
    }
    else if (isPathGenerator)
    {
        var["kUseEnvBackground"] = mpScene->useEnvBackground();
    }

    if (auto outputDebug = var.findMember("outputDebug"); outputDebug.isValid())
    {
        outputDebug = renderData.getTexture(kOutputDebug); // Can be nullptr
    }
    if (auto outputTime = var.findMember("outputTime"); outputTime.isValid())
    {
        outputTime = renderData.getTexture(kOutputTime); // Can be nullptr
    }

    if (isPathTracer && mpEmissiveSampler)
    {
        // TODO: Do we have to bind this every frame?
        mpEmissiveSampler->bindShaderData(var["emissiveSampler"]);
    }

    /// Not in 2022 code
    /*
    ref<Texture> pViewDir;
    if (mpScene->getCamera()->getApertureRadius() > 0.f)
    {
        pViewDir = renderData.getTexture(kInputViewDir);
        if (!pViewDir)
            logWarning("Depth-of-field requires the '{}' input. Expect incorrect rendering.", kInputViewDir);
    }

    ref<Texture> pSampleCount;
    if (!mFixedSampleCount)
    {
        pSampleCount = renderData.getTexture(kInputSampleCount);
        if (!pSampleCount)
            FALCOR_THROW("ReSTIRPTPass: Missing sample count input texture");
    }
    */
}

bool ReSTIRPTPass::beginFrame(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mOptionsChanged)
    {
        mReservoirFrameCount = 0;
    }
    const auto& pOutputColor = renderData.getTexture(kOutputColor);
    FALCOR_ASSERT(pOutputColor);

    // Set output frame dimension.
    setFrameDim(uint2(pOutputColor->getWidth(), pOutputColor->getHeight()));

    // Validate all I/O sizes match the expected size.
    // If not, we'll disable the path tracer to give the user a chance to fix the configuration before re-enabling it.
    bool resolutionMismatch = false;
    auto validateChannels = [&](const auto& channels)
    {
        for (const auto& channel : channels)
        {
            auto pTexture = renderData.getTexture(channel.name);
            if (pTexture && (pTexture->getWidth() != mParams.frameDim.x || pTexture->getHeight() != mParams.frameDim.y))
                resolutionMismatch = true;
        }
    };
    validateChannels(kInputChannels);
    validateChannels(kOutputChannels);

    if (mEnabled && resolutionMismatch)
    {
        logError("ReSTIRPTPass I/O sizes don't match. The pass will be disabled.");
        mEnabled = false;
    }

    if (mpScene == nullptr || !mEnabled)
    {
        pRenderContext->clearUAV(pOutputColor->getUAV().get(), float4(0.f));

        // Set refresh flag if changes that affect the output have occured.
        // This is needed to ensure other passes get notified when the path tracer is enabled/disabled.
        if (mOptionsChanged)
        {
            auto& dict = renderData.getDictionary();
            auto flags = dict.getValue(kRenderPassRefreshFlags, Falcor::RenderPassRefreshFlags::None);
            if (mOptionsChanged)
                flags |= Falcor::RenderPassRefreshFlags::RenderOptionsChanged;
            dict[Falcor::kRenderPassRefreshFlags] = flags;
        }

        return false;
    }

    // Update materials.
    prepareMaterials(pRenderContext);

    // Update the env map and emissive sampler to the current frame.
    bool lightingChanged = prepareLighting(pRenderContext);

    //// Prepare RTXDI.
    //prepareRTXDI(pRenderContext);
    //if (mpRTXDI)
    //    mpRTXDI->beginFrame(pRenderContext, mParams.frameDim);

    // Update refresh flag if changes that affect the output have occured.
    auto& dict = renderData.getDictionary();
    if (mOptionsChanged || lightingChanged)
    {
        auto flags = dict.getValue(kRenderPassRefreshFlags, Falcor::RenderPassRefreshFlags::None);
        if (mOptionsChanged)
            flags |= Falcor::RenderPassRefreshFlags::RenderOptionsChanged;
        if (lightingChanged)
            flags |= Falcor::RenderPassRefreshFlags::LightingChanged;
        dict[Falcor::kRenderPassRefreshFlags] = flags;
        mOptionsChanged = false;
    }

    // Check if GBuffer has adjusted shading normals enabled.
    bool gbufferAdjustShadingNormals = dict.getValue(Falcor::kRenderPassGBufferAdjustShadingNormals, false);
    if (gbufferAdjustShadingNormals != mGBufferAdjustShadingNormals)
    {
        mGBufferAdjustShadingNormals = gbufferAdjustShadingNormals;
        mRecompile = true;
    }
    /*
    /// NOT in ReSTIR PT
    // Check if fixed sample count should be used. When the sample count input is connected we load the count from there instead.
    mFixedSampleCount = renderData[kInputSampleCount] == nullptr;

    // Check if guide data should be generated.
    mOutputGuideData = renderData[kOutputAlbedo] != nullptr || renderData[kOutputSpecularAlbedo] != nullptr ||
                       renderData[kOutputIndirectAlbedo] != nullptr || renderData[kOutputGuideNormal] != nullptr ||
                       renderData[kOutputReflectionPosW] != nullptr;
    */

    // Check if NRD data should be generated.
    mOutputNRDData =
        renderData[kOutputNRDDiffuseRadianceHitDist] != nullptr
        || renderData[kOutputNRDSpecularRadianceHitDist] != nullptr
        || renderData[kOutputNRDResidualRadianceHitDist] != nullptr
        || renderData[kOutputNRDEmission] != nullptr
        || renderData[kOutputNRDDiffuseReflectance] != nullptr
        || renderData[kOutputNRDSpecularReflectance] != nullptr;


    // Check if additional NRD data should be generated.
    bool prevOutputNRDAdditionalData = mOutputNRDAdditionalData;
    /*
    /// NOT in ReSTIR PT
    mOutputNRDAdditionalData =
        renderData[kOutputNRDDeltaReflectionRadianceHitDist] != nullptr ||
        renderData[kOutputNRDDeltaTransmissionRadianceHitDist] != nullptr || renderData[kOutputNRDDeltaReflectionReflectance] != nullptr ||
        renderData[kOutputNRDDeltaReflectionEmission] != nullptr || renderData[kOutputNRDDeltaReflectionNormWRoughMaterialID] != nullptr ||
        renderData[kOutputNRDDeltaReflectionPathLength] != nullptr || renderData[kOutputNRDDeltaReflectionHitDist] != nullptr ||
        renderData[kOutputNRDDeltaTransmissionReflectance] != nullptr || renderData[kOutputNRDDeltaTransmissionEmission] != nullptr ||
        renderData[kOutputNRDDeltaTransmissionNormWRoughMaterialID] != nullptr ||
        renderData[kOutputNRDDeltaTransmissionPathLength] != nullptr || renderData[kOutputNRDDeltaTransmissionPosW] != nullptr;
    */

    if (mOutputNRDAdditionalData != prevOutputNRDAdditionalData)
        mRecompile = true;

    // Check if time data should be generated.
    mOutputTime = renderData[kOutputTime] != nullptr;

    // Enable pixel stats if rayCount or pathLength outputs are connected.
    if (renderData[kOutputRayCount] != nullptr
        || renderData[kOutputPathLength] != nullptr
        || mEnableRayStats)
    {
        mpPixelStats->setEnabled(true);
    }

    mpPixelStats->beginFrame(pRenderContext, mParams.frameDim);
    mpPixelDebug->beginFrame(pRenderContext, mParams.frameDim);

    // Update the random seed.
    int initialShaderPasses = mStaticParams.pathSamplingMode == PathSamplingMode::PathTracing ? 1 : mStaticParams.samplesPerPixel;
    mParams.seed = mParams.useFixedSeed ? mParams.fixedSeed : mParams.frameCount;

    return true;
}

void ReSTIRPTPass::endFrame(RenderContext* pRenderContext, const RenderData& renderData)
{
    mpPixelStats->endFrame(pRenderContext);
    mpPixelDebug->endFrame(pRenderContext);

    if (mEnableRayStats)
    {
        PixelStats::Stats stats;
        mpPixelStats->getStats(stats);
        mAccumulatedShadowRayCount += (uint64_t)stats.visibilityRays;  // was .shadowRays
        mAccumulatedClosestHitRayCount += (uint64_t)stats.closestHitRays;
        mAccumulatedRayCount += (uint64_t)stats.totalRays;
    }

    auto copyTexture = [pRenderContext](Texture* pDst, const Texture* pSrc)
    {
        if (pDst && pSrc)
        {
            FALCOR_ASSERT(pDst && pSrc);
            FALCOR_ASSERT(pDst->getFormat() == pSrc->getFormat());
            FALCOR_ASSERT(pDst->getWidth() == pSrc->getWidth() && pDst->getHeight() == pSrc->getHeight());
            pRenderContext->copyResource(pDst, pSrc);
        }
        else if (pDst)
        {
            pRenderContext->clearUAV(pDst->getUAV().get(), uint4(0, 0, 0, 0));
        }
    };

    // Copy pixel stats to outputs if available.
    copyTexture(renderData.getTexture(kOutputRayCount).get(), mpPixelStats->getRayCountTexture(pRenderContext).get());
    copyTexture(renderData.getTexture(kOutputPathLength).get(), mpPixelStats->getPathLengthTexture().get());

    //if (mpRTXDI)
    //    mpRTXDI->endFrame(pRenderContext);

    mVarsChanged = false;
    mParams.frameCount++;
}

void ReSTIRPTPass::generatePaths(RenderContext* pRenderContext, const RenderData& renderData, int sampleId)
{
    FALCOR_PROFILE(pRenderContext, "generatePaths");

    // Check shader assumptions.
    // We launch one thread group per screen tile, with threads linearly indexed.
    const uint32_t tileSize = kScreenTileDim.x * kScreenTileDim.y;
    FALCOR_ASSERT(kScreenTileDim.x == 16 && kScreenTileDim.y == 16); // TODO: Remove this temporary limitation when Slang bug has been
                                                                     // fixed, see comments in shader.
    FALCOR_ASSERT(kScreenTileBits.x <= 4 && kScreenTileBits.y <= 4); // Since we use 8-bit deinterleave.
    FALCOR_ASSERT(mpGeneratePaths->getThreadGroupSize().x == tileSize);
    FALCOR_ASSERT(mpGeneratePaths->getThreadGroupSize().y == 1 && mpGeneratePaths->getThreadGroupSize().z == 1);

    // Additional specialization. This shouldn't change resource declarations.
    mpGeneratePaths->addDefine(
        "USE_VIEW_DIR", (mpScene->getCamera()->getApertureRadius() > 0 && renderData[kInputViewDir] != nullptr) ? "1" : "0"
    );
    mpGeneratePaths->addDefine("OUTPUT_TIME", mOutputTime ? "1" : "0");
    mpGeneratePaths->addDefine("OUTPUT_GUIDE_DATA", mOutputGuideData ? "1" : "0");
    mpGeneratePaths->addDefine("OUTPUT_NRD_DATA", mOutputNRDData ? "1" : "0");
    mpGeneratePaths->addDefine("OUTPUT_NRD_ADDITIONAL_DATA", mOutputNRDAdditionalData ? "1" : "0");

    // Bind resources.
    auto var = mpGeneratePaths->getRootVar()["CB"]["gPathGenerator"];
    bindShaderData(var, renderData, false, true);

    // was: mpGeneratePaths["gScene"] = mpScene->getParameterBlock();
    mpScene->bindShaderData(mpGeneratePaths->getRootVar()["gScene"]);
    var["gSampleId"] = sampleId;

    //if (mpRTXDI)
    //    mpRTXDI->bindShaderData(mpGeneratePaths->getRootVar());

    // Launch one thread per pixel.
    // The dimensions are padded to whole tiles to allow re-indexing the threads in the shader.
    mpGeneratePaths->execute(pRenderContext, {mParams.screenTiles.x * tileSize, mParams.screenTiles.y, 1u});
}

void ReSTIRPTPass::tracePass(RenderContext* pRenderContext, const RenderData& renderData, TracePass& tracePass, int sampleID)
{
    FALCOR_PROFILE(pRenderContext, tracePass.name);

    FALCOR_ASSERT(tracePass.pProgram != nullptr && tracePass.pBindingTable != nullptr && tracePass.pVars != nullptr);

    // Additional specialization. This shouldn't change resource declarations.
    bool outputDebug = renderData[kOutputDebug] != nullptr;
    tracePass.pProgram->addDefine("OUTPUT_TIME", mOutputTime ? "1" : "0");
    tracePass.pProgram->addDefine("OUTPUT_DEBUG", outputDebug ? "1" : "0");
    tracePass.pProgram->addDefine(
        "USE_VIEW_DIR", (mpScene->getCamera()->getApertureRadius() > 0 && renderData[kInputViewDir] != nullptr) ? "1" : "0"
    );
    tracePass.pProgram->addDefine("OUTPUT_GUIDE_DATA", mOutputGuideData ? "1" : "0");
    tracePass.pProgram->addDefine("OUTPUT_NRD_DATA", mOutputNRDData ? "1" : "0");
    tracePass.pProgram->addDefine("OUTPUT_NRD_ADDITIONAL_DATA", mOutputNRDAdditionalData ? "1" : "0");

    // Bind global resources.
    auto var = tracePass.pVars->getRootVar();
    mpScene->setRaytracingShaderData(pRenderContext, var);

    if (mVarsChanged)
        mpSampleGenerator->bindShaderData(var);
    // Shouldn't use RTXDI
    //if (mpRTXDI)
    //    mpRTXDI->bindShaderData(var);

    mpPixelStats->prepareProgram(tracePass.pProgram, var);
    mpPixelDebug->prepareProgram(tracePass.pProgram, var);

    // Bind the path tracer.
    var["gPathTracer"] = mpReSTIRPTBlock;
    var["CB"]["gSampleId"] = sampleID;

    // Full screen dispatch.
    mpScene->raytrace(pRenderContext, tracePass.pProgram.get(), tracePass.pVars, uint3(mParams.frameDim, 1));
}

void ReSTIRPTPass::PathReusePass(
    RenderContext* pRenderContext,
    uint32_t restir_i,
    const RenderData& renderData,
    bool temporalReuse,
    int spatialRoundId,
    bool isLastRound
)
{
    bool isPathReuseMISWeightComputation = spatialRoundId == -1;

    FALCOR_PROFILE(
        pRenderContext, temporalReuse ? "temporalReuse" : (isPathReuseMISWeightComputation ? "MISWeightComputation" : "spatialReuse")
    );

    ref<ComputePass> pass =
        isPathReuseMISWeightComputation ? mpComputePathReuseMISWeightsPass : (temporalReuse ? mpTemporalReusePass : mpSpatialReusePass);
    if (isPathReuseMISWeightComputation)
    {
        spatialRoundId = 0;
        restir_i = 0;
    }

    // Check shader assumptions.
    // We launch one thread group per screen tile, with threads linearly indexed.
    const uint32_t tileSize = kScreenTileDim.x * kScreenTileDim.y;
    FALCOR_ASSERT(kScreenTileDim.x == 16 && kScreenTileDim.y == 16); // TODO: Remove this temporary limitation when Slang bug has been fixed, see comments in shader.
    FALCOR_ASSERT(kScreenTileBits.x <= 4 && kScreenTileBits.y <= 4); // Since we use 8-bit deinterleave.
    FALCOR_ASSERT(pass->getThreadGroupSize().x == 16);
    FALCOR_ASSERT(pass->getThreadGroupSize().y == 16 && pass->getThreadGroupSize().z == 1);

    // Additional specialization. This shouldn't change resource declarations.
    pass->addDefine("OUTPUT_TIME", mOutputTime ? "1" : "0");
    pass->addDefine("TEMPORAL_REUSE", temporalReuse ? "1" : "0");
    pass->addDefine("OUTPUT_NRD_DATA", mOutputNRDData ? "1" : "0");

    // Bind resources.
    auto var = pass->getRootVar()["CB"]["gPathReusePass"];

    // TODO 2022: refactor arguments
    bindShaderData(var, renderData, false, false);

    // NOTE: mNRooksPatternBuffer isn't filled
    if (mStaticParams.pathSamplingMode == PathSamplingMode::PathReuse)
    {
        var["nRooksPattern"] = mNRooksPatternBuffer;
    }

    if (mStaticParams.pathSamplingMode == PathSamplingMode::PathReuse)
    {
        var["misWeightBuffer"] = mPathReuseMISWeightBuffer;
    }
    else if (!isPathReuseMISWeightComputation)
    {
        var["temporalReservoirs"] = spatialRoundId % 2 == 0 ? mpTemporalReservoirs[restir_i] : mpOutputReservoirs;
    }
    var["reconnectionDataBuffer"] = mReconnectionDataBuffer;
    var["gNumSpatialRounds"] = mNumSpatialRounds;

    if (temporalReuse)
    {
        var["temporalVbuffer"] = mpTemporalVBuffer;
        var["motionVectors"] = renderData.getTexture(kInputMotionVectors);
        var["gEnableTemporalReprojection"] = mEnableTemporalReprojection;
        var["gNoResamplingForTemporalReuse"] = mNoResamplingForTemporalReuse;
        if (!mUseMaxHistory)
            var["gTemporalHistoryLength"] = 1e30f;
        else
            var["gTemporalHistoryLength"] = (float)mTemporalHistoryLength;
    }
    else
    {
        var["gSpatialReusePattern"] =
            mStaticParams.pathSamplingMode == PathSamplingMode::PathReuse ? (uint32_t)mPathReusePattern : (uint32_t)mSpatialReusePattern;

        if (!isPathReuseMISWeightComputation)
        {
            var["gNeighborCount"] = mSpatialNeighborCount;
            var["gGatherRadius"] = mSpatialReuseRadius;
            var["gSpatialRoundId"] = spatialRoundId;
            var["gSmallWindowRadius"] = mSmallWindowRestirWindowRadius;
            var["gFeatureBasedRejection"] = mFeatureBasedRejection;
            var["neighborOffsets"] = mpNeighborOffsets;
        }

        if (mOutputNRDData && !isPathReuseMISWeightComputation)
        {
            var["outputNRDDiffuseRadianceHitDist"] = renderData.getTexture(kOutputNRDDiffuseRadianceHitDist);
            var["outputNRDSpecularRadianceHitDist"] = renderData.getTexture(kOutputNRDSpecularRadianceHitDist);
            var["outputNRDResidualRadianceHitDist"] = renderData.getTexture(kOutputNRDResidualRadianceHitDist);
            var["primaryHitEmission"] = renderData.getTexture(kOutputNRDEmission);
            var["gSppId"] = restir_i;
        }
    }

    if (!isPathReuseMISWeightComputation)
    {
        var["directLighting"] = renderData.getTexture(kInputDirectLighting);
        var["useDirectLighting"] = mUseDirectLighting;
    }
    var["gIsLastRound"] = mStaticParams.pathSamplingMode == PathSamplingMode::PathReuse || isLastRound;

    /// was pass["gScene"] = mpScene->getParameterBlock();
    mpScene->bindShaderData(pass->getRootVar()["gScene"]);
    /// was pass["gPathTracer"] = mpPathTracerBlock;
    pass->getRootVar()["gPathTracer"] = mpReSTIRPTBlock;

    mpPixelStats->prepareProgram(pass->getProgram(), pass->getRootVar());
    mpPixelDebug->prepareProgram(pass->getProgram(), pass->getRootVar());

    {
        // Launch one thread per pixel.
        // The dimensions are padded to whole tiles to allow re-indexing the threads in the shader.
        pass->execute(pRenderContext, {mParams.screenTiles.x * kScreenTileDim.x, mParams.screenTiles.y * kScreenTileDim.y, 1u});
    }
}

void ReSTIRPTPass::PathRetracePass(
    RenderContext* pRenderContext,
    uint32_t restir_i,
    const RenderData& renderData,
    bool temporalReuse,
    int spatialRoundId
)
{
    FALCOR_PROFILE(pRenderContext, temporalReuse ? "temporalPathRetrace" : "spatialPathRetrace");
    ref<ComputePass> pass = (temporalReuse ? mpTemporalPathRetracePass : mpSpatialPathRetracePass);

    // Check shader assumptions.
    // We launch one thread group per screen tile, with threads linearly indexed.
    const uint32_t tileSize = kScreenTileDim.x * kScreenTileDim.y;
    FALCOR_ASSERT(kScreenTileDim.x == 16 && kScreenTileDim.y == 16); // TODO: Remove this temporary limitation when Slang bug has been fixed, see comments in shader.
    FALCOR_ASSERT(kScreenTileBits.x <= 4 && kScreenTileBits.y <= 4); // Since we use 8-bit deinterleave.
    FALCOR_ASSERT(pass->getThreadGroupSize().x == 16);
    FALCOR_ASSERT(pass->getThreadGroupSize().y == 16 && pass->getThreadGroupSize().z == 1);

    // Additional specialization. This shouldn't change resource declarations.
    pass->addDefine("OUTPUT_TIME", mOutputTime ? "1" : "0");
    pass->addDefine("TEMPORAL_REUSE", temporalReuse ? "1" : "0");

    // Bind resources.
    auto var = pass->getRootVar()["CB"]["gPathRetracePass"];

    // TODO: refactor arguments
    bindShaderData(var, renderData, false, false);
    var["outputReservoirs"] = spatialRoundId % 2 == 1 ? mpTemporalReservoirs[restir_i] : mpOutputReservoirs;

    // NOTE: mNRooksPatternBuffer isn't filled
    if (mStaticParams.pathSamplingMode == PathSamplingMode::PathReuse)
    {
        var["nRooksPattern"] = mNRooksPatternBuffer;
    }

    var["temporalReservoirs"] = spatialRoundId % 2 == 0 ? mpTemporalReservoirs[restir_i] : mpOutputReservoirs;
    var["reconnectionDataBuffer"] = mReconnectionDataBuffer;
    var["gNumSpatialRounds"] = mNumSpatialRounds;

    if (temporalReuse)
    {
        var["temporalVbuffer"] = mpTemporalVBuffer;
        var["motionVectors"] = renderData.getTexture(kInputMotionVectors);
        var["gEnableTemporalReprojection"] = mEnableTemporalReprojection;
        var["gNoResamplingForTemporalReuse"] = mNoResamplingForTemporalReuse;
        if (!mUseMaxHistory)
            var["gTemporalHistoryLength"] = 1e30f;
        else
            var["gTemporalHistoryLength"] = (float)mTemporalHistoryLength;
    }
    else
    {
        var["gSpatialRoundId"] = spatialRoundId;
        var["neighborOffsets"] = mpNeighborOffsets;
        var["gGatherRadius"] = mSpatialReuseRadius;
        var["gNeighborCount"] = mSpatialNeighborCount;
        var["gSmallWindowRadius"] = mSmallWindowRestirWindowRadius;
        var["gSpatialReusePattern"] =
            mStaticParams.pathSamplingMode == PathSamplingMode::PathReuse ? (uint32_t)mPathReusePattern : (uint32_t)mSpatialReusePattern;
        var["gFeatureBasedRejection"] = mFeatureBasedRejection;
    }

    /// was pass["gScene"] = mpScene->getParameterBlock();
    mpScene->bindShaderData(pass->getRootVar()["gScene"]);
    /// was pass["gPathTracer"] = mpPathTracerBlock;
    pass->getRootVar()["gPathTracer"] = mpReSTIRPTBlock;

    mpPixelStats->prepareProgram(pass->getProgram(), pass->getRootVar());
    mpPixelDebug->prepareProgram(pass->getProgram(), pass->getRootVar());

    {
        // Launch one thread per pixel.
        // The dimensions are padded to whole tiles to allow re-indexing the threads in the shader.
        pass->execute(pRenderContext, {mParams.screenTiles.x * kScreenTileDim.x, mParams.screenTiles.y * kScreenTileDim.y, 1u});
    }
}

ref<Texture> ReSTIRPTPass::createNeighborOffsetTexture(uint32_t sampleCount)
{
    /// was std::unique_ptr<int8_t[]> offsets(new int8_t[sampleCount * 2]);
    std::unique_ptr<int8_t[]> offsets = std::make_unique<int8_t[]>(sampleCount * 2);
    const int R = 254;
    const float phi2 = 1.f / 1.3247179572447f;
    float u = 0.5f;
    float v = 0.5f;
    for (uint32_t index = 0; index < sampleCount * 2;)
    {
        u += phi2;
        v += phi2 * phi2;
        if (u >= 1.f)
            u -= 1.f;
        if (v >= 1.f)
            v -= 1.f;

        float rSq = (u - 0.5f) * (u - 0.5f) + (v - 0.5f) * (v - 0.5f);
        if (rSq > 0.25f)
            continue;

        offsets[index++] = int8_t((u - 0.5f) * R);
        offsets[index++] = int8_t((v - 0.5f) * R);
    }

    return mpDevice->createTexture1D(sampleCount, ResourceFormat::RG8Snorm, 1, 1, offsets.get());
}

DefineList ReSTIRPTPass::StaticParams::getDefines(const ReSTIRPTPass& owner) const
{
    DefineList defines;

    // Path tracer configuration.
    defines.add("SAMPLES_PER_PIXEL", std::to_string(samplesPerPixel));  // 0 indicates a variable sample count
    defines.add("CANDIDATE_SAMPLES", std::to_string(candidateSamples)); // 0 indicates a variable sample count
    defines.add("MAX_SURFACE_BOUNCES", std::to_string(maxSurfaceBounces));
    defines.add("MAX_DIFFUSE_BOUNCES", std::to_string(maxDiffuseBounces));
    defines.add("MAX_SPECULAR_BOUNCES", std::to_string(maxSpecularBounces));
    defines.add("MAX_TRANSMISSON_BOUNCES", std::to_string(maxTransmissionBounces));
    defines.add("ADJUST_SHADING_NORMALS", adjustShadingNormals ? "1" : "0");
    defines.add("USE_BSDF_SAMPLING", useBSDFSampling ? "1" : "0");
    defines.add("USE_NEE", useNEE ? "1" : "0");
    defines.add("USE_MIS", useMIS ? "1" : "0");
    defines.add("USE_RUSSIAN_ROULETTE", useRussianRoulette ? "1" : "0");
    defines.add("USE_ALPHA_TEST", useAlphaTest ? "1" : "0");
    defines.add("USE_LIGHTS_IN_DIELECTRIC_VOLUMES", useLightsInDielectricVolumes ? "1" : "0");
    defines.add("LIMIT_TRANSMISSION", limitTransmission ? "1" : "0");
    defines.add("MAX_TRANSMISSION_REFLECTION_DEPTH", std::to_string(maxTransmissionReflectionDepth));
    defines.add("MAX_TRANSMISSION_REFRACTION_DEPTH", std::to_string(maxTransmissionRefractionDepth));
    defines.add("DISABLE_CAUSTICS", disableCaustics ? "1" : "0");
    defines.add("DISABLE_DIRECT_ILLUMINATION", disableDirectIllumination ? "1" : "0");
    defines.add("PRIMARY_LOD_MODE", std::to_string((uint32_t)primaryLodMode));
    defines.add("USE_NRD_DEMODULATION", useNRDDemodulation ? "1" : "0");
    defines.add("COLOR_FORMAT", std::to_string((uint32_t)colorFormat));
    defines.add("MIS_HEURISTIC", std::to_string((uint32_t)misHeuristic));
    defines.add("MIS_POWER_EXPONENT", std::to_string(misPowerExponent));
    defines.add("_USE_DETERMINISTIC_BSDF", useDeterministicBSDF ? "1" : "0");
    defines.add("NEIGHBOR_OFFSET_COUNT", std::to_string(kNeighborOffsetCount));
    defines.add("SHIFT_STRATEGY", std::to_string((uint32_t)shiftStrategy));
    defines.add("PATH_SAMPLING_MODE", std::to_string((uint32_t)pathSamplingMode));

    // Sampling utilities configuration.
    FALCOR_ASSERT(owner.mpSampleGenerator);
    defines.add(owner.mpSampleGenerator->getDefines());

    if (owner.mpEmissiveSampler)
        defines.add(owner.mpEmissiveSampler->getDefines());
    if (owner.mpRTXDI)
        defines.add(owner.mpRTXDI->getDefines());

    // We don't use the legacy shading code anymore (MaterialShading.slang).
    defines.add("_USE_LEGACY_SHADING_CODE", "0");

    defines.add("INTERIOR_LIST_SLOT_COUNT", std::to_string(maxNestedMaterials));

    defines.add("GBUFFER_ADJUST_SHADING_NORMALS", owner.mGBufferAdjustShadingNormals ? "1" : "0");

    // Scene-specific configuration.
    const auto& scene = owner.mpScene;
    if (scene)
        defines.add(scene->getSceneDefines());
    defines.add("USE_ENV_LIGHT", scene && scene->useEnvLight() ? "1" : "0");
    defines.add("USE_ANALYTIC_LIGHTS", scene && scene->useAnalyticLights() ? "1" : "0");
    defines.add("USE_EMISSIVE_LIGHTS", scene && scene->useEmissiveLights() ? "1" : "0");
    defines.add("USE_CURVES", scene && (scene->hasGeometryType(Scene::GeometryType::Curve)) ? "1" : "0");
    defines.add("USE_SDF_GRIDS", scene && scene->hasGeometryType(Scene::GeometryType::SDFGrid) ? "1" : "0");
    defines.add("USE_HAIR_MATERIAL", scene && scene->getMaterialCountByType(MaterialType::Hair) > 0u ? "1" : "0");

    // Set default (off) values for additional features.
    defines.add("USE_VIEW_DIR", "0");
    defines.add("OUTPUT_GUIDE_DATA", "0");
    defines.add("OUTPUT_NRD_DATA", "0");
    defines.add("OUTPUT_NRD_ADDITIONAL_DATA", "0");

    // ReSTIR config
    defines.add("SPATIAL_RESTIR_MIS_KIND", std::to_string((uint32_t)spatialMisKind));
    defines.add("TEMPORAL_RESTIR_MIS_KIND", std::to_string((uint32_t)temporalMisKind));

    defines.add("TEMPORAL_UPDATE_FOR_DYNAMIC_SCENE", temporalUpdateForDynamicScene ? "1" : "0");

    defines.add("BPR", pathSamplingMode == PathSamplingMode::PathReuse ? "1" : "0");

    defines.add("SEPARATE_PATH_BSDF", separatePathBSDF ? "1" : "0");

    defines.add("RCDATA_PATH_NUM", rcDataOfflineMode ? "12" : "6");
    defines.add("RCDATA_PAD_SIZE", rcDataOfflineMode ? "2" : "1");

    return defines;
}
