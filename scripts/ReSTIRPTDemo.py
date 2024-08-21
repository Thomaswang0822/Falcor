from falcor import *
import os


def render_graph_ReSTIRPT():
    g = RenderGraph("ReSTIRPT")

    ReSTIRPTPass = createPass("ReSTIRPTPass", {'samplesPerPixel': 1})
    g.addPass(ReSTIRPTPass, "ReSTIRPTPass")
    VBufferRT = createPass("VBufferRT")
    g.addPass(VBufferRT, "VBufferRT")
    AccumulatePass = createPass("AccumulatePass", {'enabled': False, 'precisionMode': 'Double'})
    g.addPass(AccumulatePass, "AccumulatePass")
    ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0})
    g.addPass(ToneMapper, "ToneMapper")
    ScreenSpaceReSTIRPass = createPass("ScreenSpaceReSTIRPass")
    g.addPass(ScreenSpaceReSTIRPass, "ScreenSpaceReSTIRPass")

    g.addEdge("VBufferRT.vbuffer", "ReSTIRPTPass.vbuffer")
    g.addEdge("VBufferRT.mvec", "ReSTIRPTPass.motionVectors")

    g.addEdge("VBufferRT.vbuffer", "ScreenSpaceReSTIRPass.vbuffer")
    g.addEdge("VBufferRT.mvec", "ScreenSpaceReSTIRPass.motionVectors")
    g.addEdge("ScreenSpaceReSTIRPass.color", "ReSTIRPTPass.directLighting")

    g.addEdge("ReSTIRPTPass.color", "AccumulatePass.input")
    g.addEdge("AccumulatePass.output", "ToneMapper.src")

    g.markOutput("ToneMapper.dst")
    g.markOutput("AccumulatePass.output")

    return g

graph_ReSTIRPT = render_graph_ReSTIRPT()

m.addGraph(graph_ReSTIRPT)
m.loadScene('VeachAjar/VeachAjar.pyscene')
