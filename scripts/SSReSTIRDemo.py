from falcor import *
import os


def render_graph_SSReSTIPR():
    g = RenderGraph("SSReSTIPR")

    # Create passes
    ScreenSpaceReSTIRPass = createPass("ScreenSpaceReSTIRPass")
    g.addPass(ScreenSpaceReSTIRPass, "ScreenSpaceReSTIRPass")
    VBufferRT = createPass("VBufferRT")
    g.addPass(VBufferRT, "VBufferRT")
    AccumulatePass = createPass("AccumulatePass", {'enabled': False, 'precisionMode': 'Double'})
    g.addPass(AccumulatePass, "AccumulatePass")
    ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0})
    g.addPass(ToneMapper, "ToneMapper")


    # Add edges to connect passes
    g.addEdge("VBufferRT.vbuffer", "ScreenSpaceReSTIRPass.vbuffer")
    g.addEdge("VBufferRT.mvec", "ScreenSpaceReSTIRPass.motionVectors")

    g.addEdge("ScreenSpaceReSTIRPass.color", "AccumulatePass.input")
    g.addEdge("AccumulatePass.output", "ToneMapper.src")

    g.markOutput("ToneMapper.dst")

    # Besides tone-mapped final color, can check intermediate output
    g.markOutput("AccumulatePass.output")

    return g

graph_SSReSTIPR = render_graph_SSReSTIPR()

try:
    m.addGraph(graph_SSReSTIPR)
except Exception(e):
    print(f"Got error when creating render graph: {e}")
