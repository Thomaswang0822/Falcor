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
    Denoiser = createPass("OptixDenoiser", {'enabled': False})
    g.addPass(Denoiser, "Denoiser")

    # Add edges to connect passes
    g.addEdge("VBufferRT.vbuffer", "ReSTIRPTPass.vbuffer")
    # g.addEdge("VBufferRT.viewW", "ReSTIRPTPass.viewW")
    g.addEdge("VBufferRT.mvec", "ReSTIRPTPass.motionVectors")

    # RTXDI renders to an optional buffer `directLighting` in ReSTIRPT
    RTXDIPass = createPass("RTXDIPass")
    g.addPass(RTXDIPass, "RTXDIPass")
    g.addEdge("VBufferRT.vbuffer", "RTXDIPass.vbuffer")
    g.addEdge("VBufferRT.mvec", "RTXDIPass.mvec")
    g.addEdge("RTXDIPass.color", "ReSTIRPTPass.directLighting")

    # denoiser input, see OptixDenoiser.cpp
    g.addEdge("ReSTIRPTPass.color", "Denoiser.color")
    g.addEdge("VBufferRT.mvec", "Denoiser.mvec")
    # but turns out albedo and normal bring negative effect for ReSTIR PT
    g.addEdge("ReSTIRPTPass.albedo", "Denoiser.albedo")
    g.addEdge("ReSTIRPTPass.normal", "Denoiser.normal")

    g.addEdge("Denoiser.output", "AccumulatePass.input")
    g.addEdge("AccumulatePass.output", "ToneMapper.src")

    g.markOutput("ToneMapper.dst")

    # Besides tone-mapped final color, can check intermediate output
    g.markOutput("RTXDIPass.color")

    return g

graph_ReSTIRPT = render_graph_ReSTIRPT()

m.addGraph(graph_ReSTIRPT)
