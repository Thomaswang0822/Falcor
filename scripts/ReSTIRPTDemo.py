from falcor import *
import os


def render_graph_ReSTIRPT():
    g = RenderGraph("ReSTIRPT")

    # Create passes
    ReSTIRPTPass = createPass("ReSTIRPTPass", {'samplesPerPixel': 1})
    g.addPass(ReSTIRPTPass, "ReSTIRPTPass")
    VBufferRT = createPass("VBufferRT", {'samplePattern': 'Center', 'sampleCount': 1, 'useAlphaTest': True})
    g.addPass(VBufferRT, "VBufferRT")
    AccumulatePass = createPass("AccumulatePass", {'enabled': False, 'precisionMode': 'Double'})
    g.addPass(AccumulatePass, "AccumulatePass")
    ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0})
    g.addPass(ToneMapper, "ToneMapper")

    # Add edges to connect passes
    g.addEdge("VBufferRT.vbuffer", "ReSTIRPTPass.vbuffer")
    # g.addEdge("VBufferRT.viewW", "ReSTIRPTPass.viewW")
    g.addEdge("VBufferRT.mvec", "ReSTIRPTPass.motionVectors")

    # TODO: connect RTXDI output to ReSTIRPTPass.directLighting "internally",
    # @see PathTracer, but we possibly need more than RTXDIPass.color
    RTXDIPass = createPass("RTXDIPass")
    g.addPass(RTXDIPass, "RTXDIPass")
    g.addEdge("VBufferRT.vbuffer", "RTXDIPass.vbuffer")
    g.addEdge("VBufferRT.mvec", "RTXDIPass.mvec")
    g.addEdge("RTXDIPass.color", "ReSTIRPTPass.directLighting")
    # END TODO

    g.addEdge("ReSTIRPTPass.color", "AccumulatePass.input")
    g.addEdge("AccumulatePass.output", "ToneMapper.src")

    g.markOutput("ToneMapper.dst")

    # Besides tone-mapped final color, can check intermediate output
    g.markOutput("RTXDIPass.color")

    return g

graph_ReSTIRPT = render_graph_ReSTIRPT()

m.addGraph(graph_ReSTIRPT)
