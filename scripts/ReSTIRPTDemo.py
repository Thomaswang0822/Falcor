from falcor import *
import os


def render_graph_ReSTIRPT():
    g = RenderGraph("ReSTIRPT")

    ReSTIRGIPlusPass = createPass("ReSTIRPT", {'samplesPerPixel': 1})
    g.addPass(ReSTIRGIPlusPass, "ReSTIRPT")
    VBufferRT = createPass("VBufferRT")
    g.addPass(VBufferRT, "VBufferRT")
    AccumulatePass = createPass("AccumulatePass", {'enabled': False, 'precisionMode': 'Double'})
    g.addPass(AccumulatePass, "AccumulatePass")
    ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0})
    g.addPass(ToneMapper, "ToneMapper")
    ScreenSpaceReSTIRPass = createPass("ScreenSpaceReSTIRPass")
    g.addPass(ScreenSpaceReSTIRPass, "ScreenSpaceReSTIRPass")

    g.addEdge("VBufferRT.vbuffer", "ReSTIRPT.vbuffer")
    g.addEdge("VBufferRT.mvec", "ReSTIRPT.mvec")

    g.addEdge("VBufferRT.vbuffer", "ScreenSpaceReSTIRPass.vbuffer")
    g.addEdge("VBufferRT.mvec", "ScreenSpaceReSTIRPass.motionVectors")
    g.addEdge("ScreenSpaceReSTIRPass.color", "ReSTIRPT.directLighting")

    g.addEdge("ReSTIRPT.color", "AccumulatePass.input")
    g.addEdge("AccumulatePass.output", "ToneMapper.src")

    g.markOutput("ToneMapper.dst")
    g.markOutput("AccumulatePass.output")

    return g

graph_ReSTIRPT = render_graph_ReSTIRPT()

m.addGraph(graph_ReSTIRPT)
m.loadScene('VeachAjar/VeachAjarAnimated.pyscene')
