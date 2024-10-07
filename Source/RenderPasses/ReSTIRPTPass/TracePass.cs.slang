/***************************************************************************
 # Copyright (c) 2022, Daqi Lin.  All rights reserved.
 **************************************************************************/
import Scene.Raytracing;
import Utils.Debug.PixelDebug;
import Rendering.Utils.PixelStats;
import PathTracer;
import PathState;
import Params;
import PathReservoir;

ParameterBlock<PathTracer> gPathTracer;

cbuffer CB
{
    int gSampleId;
}

void tracePath(uint pathID, inout PathReservoir giReservoir, uint sampleIdx)
{
    PathState path;
    gPathTracer.generatePath(pathID, path);

    printSetPixel(path.getPixel());
    logSetPixel(path.getPixel());

    while (path.isActive())
    {
        if (path.isHit())
        {
            gPathTracer.handleHit(path);
            if (path.isTerminated()) break;

            //// ***DEBUG
            //// EVERY path is terminated at length = 0
            
            //if (path.isTerminated()) {   // !path.isActive()
            //    // GREEN
            //    gPathTracer.outputColor[path.getPixel()] = 
            //        float4(0.f, path.hit.getPrimitiveIndex() / 255.f, 0.f, 1.f);
            //    return;
            //}
            //else {
            //    // BLUE
            //    gPathTracer.outputColor[path.getPixel()] = 
            //        float4(0.f, 0.f, path.hit.getPrimitiveIndex() / 255.f, 1.f);
            //    return;
            //}
            
            gPathTracer.nextVertex(path);
        }
        else
        {
            gPathTracer.handleMiss(path);
        }
    }

    gPathTracer.finalize(path);
    gPathTracer.writeOutput(path, giReservoir, sampleIdx);
}

[shader("miss")]
void scatterMiss(inout PathState path)
{
    gPathTracer.handleMiss(path);
}

//
// Shader entry point for ray generation shader.
//

// [numthreads(16, 16, 1)]
// void main(uint3 dispatchThreadId : SV_DispatchThreadID)
[shader("raygeneration")]
void rayGen()
{
    uint2 pixel = DispatchRaysIndex().xy;
    uint2 frameDim = DispatchRaysDimensions().xy;
    if (all(pixel >= frameDim)) return;

    // Skip pixel if there is no hit in the vbuffer.
    HitInfo hit = HitInfo(gPathTracer.vbuffer[pixel]);
    if (!hit.isValid()) return;

    PathReservoir giReservoir;

    static const uint itersPerShaderPass = PathSamplingMode(kPathSamplingMode) == PathSamplingMode::PathTracing ? kSamplesPerPixel : kCandidateSamples;

    if (itersPerShaderPass == 1)
    {
        // Handle fixed 1 spp case.
        uint pathID = pixel.x | (pixel.y << 12) | (gSampleId << 24);
        tracePath(pathID, giReservoir, 0);
    }
    else//(itersPerShaderPass > 1)
    {
        // Handle fixed multiple spp case.
        for (uint sampleIdx = 0; sampleIdx < itersPerShaderPass; ++sampleIdx)
        {
            uint pathID = pixel.x | (pixel.y << 12) | (sampleIdx << 24);
            tracePath(pathID, giReservoir, sampleIdx);
        }
    }
}
