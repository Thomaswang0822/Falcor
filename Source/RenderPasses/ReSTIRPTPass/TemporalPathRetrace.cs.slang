/***************************************************************************
 # Copyright (c) 2022, Daqi Lin.  All rights reserved.
 **************************************************************************/
import Params;
import Shift;
import Scene.HitInfo;
import PathReservoir;
import PathTracer;
import Scene.Scene;
import Utils.Debug.PixelDebug;
import Rendering.Utils.PixelStats;
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Math.Ray;
import LoadShadingData;
import Rendering.Materials.MaterialShading;

/** TODO.
*/
struct PathRetracePass
{
    static const uint kNeighborOffsetCount = NEIGHBOR_OFFSET_COUNT;
    static const uint kNeighborOffsetMask = kNeighborOffsetCount - 1;

    // Resources
    RestirPathTracerParams params;                        ///< Runtime parameters.

    Texture2D<PackedHitInfo> vbuffer;                     ///< Fullscreen V-buffer for the primary hits.
    Texture2D<PackedHitInfo> temporalVbuffer;
    Texture2D<float2> motionVectors;

    ByteAddressBuffer nRooksPattern;

    RWTexture2D<float4> outputColor;                      ///< Output resolved color.
    RWStructuredBuffer<PathReservoir> outputReservoirs;
    RWStructuredBuffer<PathReservoir> temporalReservoirs;
    RWStructuredBuffer<PixelReconnectionData> reconnectionDataBuffer;

    int  gNumSpatialRounds;
    bool gEnableTemporalReprojection;

    float gTemporalHistoryLength;
    bool gNoResamplingForTemporalReuse;

    bool isValidPackedHitInfo(PackedHitInfo packed)
    {
        return packed.x != 0;
    }

    bool isValidScreenRegion(int2 pixel) { return all(pixel >= 0 && pixel < params.frameDim); }

    /// 2024 UPDATE:
    /// Previously, struct HitInfo stores unpacked data in an array of uint.
    /// Now HitInfo has a PackedHitInfo field directly.
    ShadingData getPixelShadingData(int2 pixel, out PackedHitInfo PrimaryHitPacked)
    {
        ShadingData sd = {};
        Ray ray = gScene.camera.computeRayPinhole(pixel, params.frameDim);
        PrimaryHitPacked = vbuffer[pixel];
        if (isValidPackedHitInfo(PrimaryHitPacked))
        {
            // HitInfo PrimaryHit; PrimaryHit.unpack(PrimaryHitPacked);
            HitInfo PrimaryHit = HitInfo(PrimaryHitPacked); // Initialize hit info from a packed hit info.

            let lod = ExplicitLodTextureSampler(0.f);
            sd = loadShadingData(PrimaryHit, ray.origin, -ray.dir, lod);
        }
        return sd;
    }

    /// 2024 UPDATE:
    /// Previously, struct HitInfo stores unpacked data in an array of uint.
    /// Now HitInfo has a PackedHitInfo field directly.
    /// Also, computeRayPinholePrevFrame() is deprecated. It differs from computeRayPinhole()
    /// only by using prev-frame camera jitter, which is also deprecated in 7.0 code.
    /// Thus, we ignore camera jitters in prev frame and use computeRayPinhole() on prev-frame pixel.
    ShadingData getPixelTemporalShadingData(int2 prevPixel, out PackedHitInfo PrimaryHitPacked)
    {
        ShadingData sd = {};
        Ray ray = gScene.camera.computeRayPinhole(prevPixel, params.frameDim);
        PrimaryHitPacked = temporalVbuffer[prevPixel];
        if (isValidPackedHitInfo(PrimaryHitPacked))
        {
            // HitInfo PrimaryHit; PrimaryHit.unpack(PrimaryHitPacked);
            HitInfo PrimaryHit = HitInfo(PrimaryHitPacked); // Initialize hit info from a packed hit info.

            let lod = ExplicitLodTextureSampler(0.f);
            sd = loadShadingData(PrimaryHit, ray.origin, -ray.dir, lod);
        }
        return sd;
    }

    /*
    * Build a reservoir of paths whose suffix can be merged to a pixel's prefix.
    * This method is optimized for small windows such that we can run the entire
    * algorithm in one pass.
    */
    void ReSTIR(const uint2 pixel)
    {
        // TODO: How to seed efficiently?
        var sg = TinyUniformSampleGenerator(pixel, (kCandidateSamples + 1 + gNumSpatialRounds) * params.seed + kCandidateSamples);

        // Compute offset into per-sample buffers. All samples are stored consecutively at this offset.
        const uint centralOffset = params.getReservoirOffset(pixel);
        PathReservoir centralReservoir = outputReservoirs[centralOffset];

        PackedHitInfo centralPrimaryHitPacked;
        ShadingData centralPrimarySd = getPixelShadingData(pixel, centralPrimaryHitPacked);
        if (!isValidPackedHitInfo(centralPrimaryHitPacked)) return;

        // color averaging test

        // fetch temporal reservoir
        float2 motionVector = motionVectors[pixel];
        int2 prevPixel = pixel;

        bool foundTemporalSurface = true;

        if (gEnableTemporalReprojection)
        {
            prevPixel = pixel + motionVector * params.frameDim + (sampleNext2D(sg) * 1.f - 0.f);
        }

        if (!isValidScreenRegion(prevPixel) || !foundTemporalSurface) return;

        PackedHitInfo temporalPrimaryHitPacked;
        ShadingData temporalPrimarySd = getPixelTemporalShadingData(prevPixel, temporalPrimaryHitPacked);
        if (!isValidPackedHitInfo(temporalPrimaryHitPacked)) return;

        PathReservoir temporalReservoir = temporalReservoirs[params.getReservoirOffset(prevPixel)];

        // talbot MIS
        // compute mis weight for current pixel

        HitInfo dstRcPrevVertexHit;
        HitInfo dstRcPrevVertexHit2;
        float3 dstRcPrevVertexWo;
        float3 dstRcPrevVertexWo2;
        float3 Tp;
        float3 Tp2;

        // write to memory
        if (centralReservoir.pathFlags.rcVertexLength() > 1)
        {
            Tp = traceHybridShiftRays(params, true, temporalPrimaryHitPacked, temporalPrimarySd, centralReservoir, dstRcPrevVertexHit, dstRcPrevVertexWo);
            reconnectionDataBuffer[centralOffset].data[0] = ReconnectionData(dstRcPrevVertexHit, dstRcPrevVertexWo, Tp);
        }
        if (temporalReservoir.pathFlags.rcVertexLength() > 1)
        {
            Tp2 = traceHybridShiftRays(params, false, centralPrimaryHitPacked, centralPrimarySd, temporalReservoir, dstRcPrevVertexHit2, dstRcPrevVertexWo2);
            reconnectionDataBuffer[centralOffset].data[1] = ReconnectionData(dstRcPrevVertexHit2, dstRcPrevVertexWo2, Tp2);
        }
    }

    /** Entry point for merge pass.
        \param[in] pixel Pixel coordinates.
    */
    void execute(const uint2 _pixel)
    {
        const uint2 pixel = _pixel;

        if (any(pixel >= params.frameDim)) return;

        printSetPixel(pixel);
        logSetPixel(pixel);

        ReSTIR(pixel);
    }
};

cbuffer CB
{
    PathRetracePass gPathRetracePass;
}


[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID)
{
    gPathRetracePass.execute(dispatchThreadId.xy);
}
