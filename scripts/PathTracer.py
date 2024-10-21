from falcor import *

def render_graph_PathTracer():
    g = RenderGraph("PathTracer")
    PathTracer = createPass("PathTracer", {'samplesPerPixel': 1})
    g.addPass(PathTracer, "PathTracer")
    VBufferRT = createPass("VBufferRT")
    g.addPass(VBufferRT, "VBufferRT")
    AccumulatePass = createPass("AccumulatePass", {'enabled': False, 'precisionMode': 'Double'})
    g.addPass(AccumulatePass, "AccumulatePass")
    ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0})
    g.addPass(ToneMapper, "ToneMapper")
    Denoiser = createPass("OptixDenoiser", {'enabled': False})
    g.addPass(Denoiser, "Denoiser")

    g.addEdge("VBufferRT.vbuffer", "PathTracer.vbuffer")
    g.addEdge("VBufferRT.viewW", "PathTracer.viewW")
    g.addEdge("VBufferRT.mvec", "PathTracer.mvec")

    # denoiser input, see OptixDenoiser.cpp
    g.addEdge("PathTracer.color", "Denoiser.color")
    g.addEdge("VBufferRT.mvec", "Denoiser.mvec")
    g.addEdge("PathTracer.albedo", "Denoiser.albedo")
    g.addEdge("PathTracer.guideNormal", "Denoiser.normal")

    g.addEdge("Denoiser.output", "AccumulatePass.input")
    g.addEdge("AccumulatePass.output", "ToneMapper.src")
    g.markOutput("ToneMapper.dst")
    return g

PathTracer = render_graph_PathTracer()
try: m.addGraph(PathTracer)
except NameError: None
