/***************************************************************************
 # Copyright (c) 2022, Daqi Lin.  All rights reserved.
 **************************************************************************/

/***************************************************************************
 # Copyright (c) 2024, Haoxuan Wang.  All rights reserved.
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
