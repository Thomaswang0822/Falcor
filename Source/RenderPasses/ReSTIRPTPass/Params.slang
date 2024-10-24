/***************************************************************************
 # Copyright (c) 2022, Daqi Lin.  All rights reserved.
 **************************************************************************/
#pragma once
#include "Utils/HostDeviceShared.slangh"

BEGIN_NAMESPACE_FALCOR

/** Enumeration of the atomic counters used.
*/
enum class Counters
{
    // TODO: Better naming
    kGeneralPaths = 0,
    kGeneralPathsProcessed,
    kSpecularPaths,
    kSpecularPathsProcessed,
    kDeltaReflectionPaths,
    kDeltaReflectionPathsProcessed,
    kDeltaTransmissionPaths,
    kDeltaTransmissionPathsProcessed,

    kGeneralQueued,
    kGeneralQueuedProcessed,

    kThreadCountSpecular,
    kThreadCountGeneral,
    kThreadCountDeltaReflection,
    kThreadCountDeltaTransmission,

    // Must be last
    kCount
};

/** Per-sample color format.
 */
enum class ColorFormat : uint32_t
{
    RGBA32F = 0,
    LogLuvHDR = 1,
};

FALCOR_ENUM_INFO(ColorFormat, {
    { ColorFormat::RGBA32F, "RGBA32F" },
    { ColorFormat::LogLuvHDR, "LogLuvHDR" },
});
FALCOR_ENUM_REGISTER(ColorFormat);

enum class MISHeuristic : uint32_t
{
    Balance = 0,  ///< Balance heuristic.
    PowerTwo = 1, ///< Power heuristic (exponent = 2.0).
    PowerExp = 2, ///< Power heuristic (variable exponent).
};

FALCOR_ENUM_INFO(MISHeuristic, {
    { MISHeuristic::Balance, "Balance" },
    { MISHeuristic::PowerTwo, "PowerTwo" },
    { MISHeuristic::PowerExp, "PowerExp" },
});
FALCOR_ENUM_REGISTER(MISHeuristic);

// Define tile sizes in pixels.
// The frame is divided into tiles stored in scanline order, with pixels in tiles enumerated in Morton order.
static const uint2 kScreenTileDim = { 16, 16 };     ///< Screen-tile dimension in pixels.
static const uint2 kScreenTileBits = { 4, 4 };      ///< Bits needed to describe pixel position within a screen-tile.

// Define path configuration limits.
static const uint kMaxSamplesPerPixel = 64;         ///< Maximum supported sample count. We can use tiling to support large sample counts if needed.
static const uint kMaxFrameDimension = 4096;        ///< Maximum supported frame dimension in pixels along x or y. We can increase the bit allocation if needed.
static const uint kMaxBounces = 14;                ///< Maximum supported number of bounces per bounce category (value 255 is reserved for internal use). The resulting path length may be longer than this.
static const uint kMaxLightSamplesPerVertex = 8;    ///< Maximum number of shadow rays per path vertex for next-event estimation.

/// ReSTIR PT enums

enum class ShiftMapping : uint32_t
{
    Reconnection = 0,
    RandomReplay = 1,
    Hybrid = 2,
};

FALCOR_ENUM_INFO(ShiftMapping, {
        { ShiftMapping::Reconnection, "Reconnection" },
        { ShiftMapping::RandomReplay, "RandomReplay" },
        { ShiftMapping::Hybrid, "Hybrid" },
});
FALCOR_ENUM_REGISTER(ShiftMapping);

enum class LocalStrategy : uint32_t
{
    None = 0x0,
    RoughnessCondition = 0x1,
    DistanceCondition = 0x2,
};

FALCOR_ENUM_INFO(LocalStrategy, {
    { LocalStrategy::None, "None" },
    { LocalStrategy::RoughnessCondition, "RoughnessCondition" },
    { LocalStrategy::DistanceCondition, "DistanceCondition" },
});
FALCOR_ENUM_REGISTER(LocalStrategy);

enum class ReSTIRMISKind : uint32_t
{
    Constant = 0,
    Talbot = 1,
    Pairwise = 2,
    ConstantBinary = 3,
    ConstantBiased = 4
};

FALCOR_ENUM_INFO(ReSTIRMISKind, {
    { ReSTIRMISKind::Constant, "Constant" },
    { ReSTIRMISKind::Talbot, "Talbot" },
    { ReSTIRMISKind::Pairwise, "Pairwise" },
    { ReSTIRMISKind::ConstantBinary, "ConstantBinary" },
    { ReSTIRMISKind::ConstantBiased, "ConstantBiased" },
});
FALCOR_ENUM_REGISTER(ReSTIRMISKind);

enum class SpatialReusePattern : uint32_t
{
    Default = 0, // same as in ScreenSpaceReSTIR
    SmallWindow = 1,
};

FALCOR_ENUM_INFO(SpatialReusePattern, {
    { SpatialReusePattern::Default, "Default" },
    { SpatialReusePattern::SmallWindow, "SmallWindow" },
});
FALCOR_ENUM_REGISTER(SpatialReusePattern);

enum class PathSamplingMode
{
    ReSTIR,
    PathReuse,
    PathTracing
};

FALCOR_ENUM_INFO(PathSamplingMode, {
    { PathSamplingMode::ReSTIR, "ReSTIR" },
    { PathSamplingMode::PathReuse, "PathReuse" },
    { PathSamplingMode::PathTracing, "PathTracing" },
});
FALCOR_ENUM_REGISTER(PathSamplingMode);

enum class PathReusePattern
{
    Block,
    NRooks,
    NRooksShift
};

FALCOR_ENUM_INFO(PathReusePattern, {
    { PathReusePattern::Block, "Block" },
    { PathReusePattern::NRooks, "NRooks" },
    { PathReusePattern::NRooksShift, "NRooksShift" },
});
FALCOR_ENUM_REGISTER(PathReusePattern);

/// end of ReSTIR PT enums

// Import static specialization constants.
#ifndef HOST_CODE
__exported import StaticParams;
__exported import Utils.Math.BitTricks;
__exported import Scene.Material.TextureSampler;
#endif


/** Path tracer parmeters shared between host/device code.
*/
struct RestirPathTracerParams
{
    int     useFixedSeed = false;       ///< Use fixed random seed. This is useful for debugging.
    uint    seed = 0;                   ///< Random seed. This will get updated from the host depending on settings.
    uint    fixedSeed = 1;              ///< The seed to use when 'useFixedSeed' is enabled.
    float   lodBias = 0.f;              ///< LOD bias applied to secondary hits.

    // Runtime values
    uint2   frameDim = { 0, 0 };        ///< Frame dimension in pixels.
    uint2   screenTiles = { 0, 0 };     ///< Number of screen-tiles. Screen tiles may extend outside the frame.

    uint    frameCount = 0;             ///< Frames rendered. This is used as random seed.
    uint    localStrategyType = (uint)LocalStrategy::RoughnessCondition | (uint)LocalStrategy::DistanceCondition;
    int     rejectShiftBasedOnJacobian = true; // can be helpful for dynamic geometry
    float   jacobianRejectionThreshold = 10.f;

    // params for roughness condition in mixed strategy
    float   specularRoughnessThreshold = 0.2f; ///< Specular reflection events are only classified as specular if the material's roughness value is equal or smaller than this threshold. Otherwise they are classified diffuse.
    // params for distance condition in mixed strategy
    float   nearFieldDistance = 0.1f; //TODO: make this adaptive to spatial reuse size / scene size
    uint2 dummy; // padding

#ifndef HOST_CODE

    uint getReservoirOffset(const uint2 pixel)
    {
        uint2 tileID = pixel >> kScreenTileBits;

        uint stride = kScreenTileDim.x * kScreenTileDim.y;
        uint tileIdx = tileID.y * screenTiles.x + tileID.x;
        uint tileOffset = tileIdx * stride;

        uint tileBits = kScreenTileBits.x + kScreenTileBits.y;
        uint pixelIdx = interleave_16bit(pixel) & ((1 << tileBits) - 1); // TODO: Use interleave_8bit() if kScreenTileBits <= 4.
        return tileOffset + pixelIdx;
    }

    /** Helper to create a texture sampler instance.
    The method for computing texture level-of-detail depends on the configuration.
    \param[in] path Path state.
    \param[in] isPrimaryTriangleHit True if primary hit on a triangle.
    \return Texture sampler instance.
    */
    ITextureSampler createTextureSampler(bool isPrimaryHit)
    {
        float lod = isPrimaryHit ? 0.f : lodBias;
        return ExplicitLodTextureSampler(lod);
    }

#endif
};

END_NAMESPACE_FALCOR
