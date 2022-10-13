# FLIP GitHub repository: https://github.com/NVlabs/flip.

# A render graph that creates a reference image and a test image
# and compares them using LDR-FLIP. The reference is an accumulation
# of 1 spp path traced images while the test is generated by
# rendering 1 spp path traced images, followed
# by denoising with the SVGF denoiser.
def render_graph_LDRFLIPDemo():
    g = RenderGraph("LDRFLIPDemo")
    loadRenderPassLibrary("AccumulatePass.dll")
    loadRenderPassLibrary("GBuffer.dll")
    loadRenderPassLibrary("PathTracer.dll")
    loadRenderPassLibrary("SVGFPass.dll")
    loadRenderPassLibrary("ToneMapper.dll")
    loadRenderPassLibrary("FLIPPass.dll")

    # Generate G-buffer
    GBufferRaster = createPass("GBufferRaster", {'cull': CullMode.CullBack})
    g.addPass(GBufferRaster, "GBufferRaster")

    ######################################################################################################
    ##########             Reference graph (accumulated path tracer results)                ##############
    ######################################################################################################
    PathTracerReference = createPass("PathTracer")
    g.addPass(PathTracerReference, "PathTracerReference")
    AccumulatePass = createPass("AccumulatePass", {'enabled': True, 'precisionMode': AccumulatePrecision.Single, 'maxAccumulatedFrames': 2 ** 16})
    g.addPass(AccumulatePass, "AccumulatePass")
    ToneMappingPassReference = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 1.0})
    g.addPass(ToneMappingPassReference, "ToneMappingPassReference")

    g.addEdge("GBufferRaster.vbuffer", "PathTracerReference.vbuffer")
    g.addEdge("PathTracerReference.color", "AccumulatePass.input")
    g.addEdge("AccumulatePass.output", "ToneMappingPassReference.src")


    ######################################################################################################
    ##########              Test graph (SVGF on top of path tracer results)                  #############
    ##########       NOTE: This graph can be replaced with your own rendering setup          #############
    ######################################################################################################
    PathTracerTest = createPass("PathTracer")
    g.addPass(PathTracerTest, "PathTracerTest")
    SVGFPass = createPass("SVGFPass", {'Enabled': True, 'Iterations': 4, 'FeedbackTap': 1, 'VarianceEpsilon': 1.0e-4, 'PhiColor': 10.0, 'PhiNormal': 128.0, 'Alpha': 0.05, 'MomentsAlpha': 0.2})
    g.addPass(SVGFPass, "SVGFPass")
    ToneMappingPassTest = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 1.0})
    g.addPass(ToneMappingPassTest, "ToneMappingPassTest")

    g.addEdge("GBufferRaster.vbuffer", "PathTracerTest.vbuffer")
    g.addEdge("GBufferRaster.emissive", "SVGFPass.Emission")
    g.addEdge("GBufferRaster.posW", "SVGFPass.WorldPosition")
    g.addEdge("GBufferRaster.normW", "SVGFPass.WorldNormal")
    g.addEdge("GBufferRaster.pnFwidth", "SVGFPass.PositionNormalFwidth")
    g.addEdge("GBufferRaster.linearZ", "SVGFPass.LinearZ")
    g.addEdge("GBufferRaster.mvec", "SVGFPass.MotionVec")
    g.addEdge("PathTracerTest.color", "SVGFPass.Color")
    g.addEdge("PathTracerTest.albedo", "SVGFPass.Albedo")
    g.addEdge("SVGFPass.Filtered image", "ToneMappingPassTest.src")


    ######################################################################################################
    ##########         Compute LDR-FLIP between tone mapped reference and test               #############
    ######################################################################################################
    FLIPPass = createPass('FLIPPass', {'enabled': True, 'isHDR': False, 'useMagma': True, 'monitorWidthPixels': 3840, 'monitorWidthMeters': 0.7, 'monitorDistanceMeters': 0.7, 'computePooledFLIPValues': False, 'useRealMonitorInfo': False})
    g.addPass(FLIPPass, 'FLIPPass')
    g.addEdge('ToneMappingPassTest.dst', 'FLIPPass.testImage') # NOTE: Input to LDR-FLIP must have low dynamic range (and be in [0, 1] range). This is achieved with standard tone mappers.
    g.addEdge('ToneMappingPassReference.dst', 'FLIPPass.referenceImage') # NOTE: Input to LDR-FLIP must have low dynamic range (and be in [0, 1] range). This is achieved with standard tone mappers.

    g.markOutput("FLIPPass.errorMapDisplay")
    g.markOutput("ToneMappingPassTest.dst")
    g.markOutput("ToneMappingPassReference.dst")

    return g

m.loadScene('Arcade/Arcade.pyscene')

LDRFLIPDemo = render_graph_LDRFLIPDemo()
try: m.addGraph(LDRFLIPDemo)
except NameError: None