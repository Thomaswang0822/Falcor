/***************************************************************************
 # Copyright (c) 2015-21, NVIDIA CORPORATION. All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/

#pragma once
#include "Utils/HostDeviceShared.slangh"

#ifdef __INTELLISENSE__

#ifndef LIGHT_SAMPLE_ENV_INDEX_BITS
#define LIGHT_SAMPLE_ENV_INDEX_BITS 26
#endif

#ifndef MAX_HISTORY_LENGTH
#define MAX_HISTORY_LENGTH 20
#endif

#ifndef USE_PAIRWISE_MIS
#define USE_PAIRWISE_MIS 0
#endif

#ifndef UNBIASED
#define UNBIASED 0
#endif

#ifndef DEBUG_OUTPUT
#define DEBUG_OUTPUT 0
#endif

#ifndef NEIGHBOR_OFFSET_COUNT
#define NEIGHBOR_OFFSET_COUNT 0
#endif

#ifndef USE_ALPHA_TEST
#define USE_ALPHA_TEST 1
#endif

#ifndef USE_VISIBILITY
#define USE_VISIBILITY 1
#endif

#ifndef REUSE_VISIBILITY
#define REUSE_VISIBILITY 1
#endif

#endif // __INTELLISENSE__

BEGIN_NAMESPACE_FALCOR

/** Enumeration of available debug outputs.
    Note: Keep in sync with definition in ScreenSpaceReSTIR.h
*/
enum class DebugOutput : uint32_t
{
    Disabled,
    Position,
    Depth,
    Normal,
    FaceNormal,
    DiffuseWeight,
    SpecularWeight,
    SpecularRoughness,
    PackedNormal,
    PackedDepth,
    InitialWeight,
    TemporalReuse,
    SpatialReuse,
    FinalSampleDir,
    FinalSampleDistance,
    FinalSampleLi,
};

FALCOR_ENUM_INFO(
    DebugOutput,
    {
        { DebugOutput::Disabled, "Disabled" },
        { DebugOutput::Position, "Position" },
        { DebugOutput::Depth, "Depth" },
        { DebugOutput::Normal, "Normal" },
        { DebugOutput::FaceNormal, "FaceNormal" },
        { DebugOutput::DiffuseWeight, "DiffuseWeight" },
        { DebugOutput::SpecularWeight, "SpecularWeight" },
        { DebugOutput::SpecularRoughness, "SpecularRoughness" },
        { DebugOutput::PackedNormal, "PackedNormal" },
        { DebugOutput::PackedDepth, "PackedDepth" },
        { DebugOutput::InitialWeight, "InitialWeight" },
        { DebugOutput::TemporalReuse, "TemporalReuse" },
        { DebugOutput::SpatialReuse, "SpatialReuse" },
        { DebugOutput::FinalSampleDir, "FinalSampleDir" },
        { DebugOutput::FinalSampleDistance, "FinalSampleDistance" },
        { DebugOutput::FinalSampleLi, "FinalSampleLi" },
    }
);
FALCOR_ENUM_REGISTER(DebugOutput);

enum class TargetPDF : uint32_t
{
    IncomingRadiance = 0,
    OutgoingRadiance = 1
};

FALCOR_ENUM_INFO(
    TargetPDF,
    {
        { TargetPDF::IncomingRadiance, "IncomingRadiance" },
        { TargetPDF::OutgoingRadiance, "OutgoingRadiance" },
    }
);
FALCOR_ENUM_REGISTER(TargetPDF);

enum class ReSTIRMode : uint32_t
{
    InputOnly = 0,                 ///< Output input samples directly.
    TemporalOnly = 1,              ///< Output temporal reuse result.
    TemporalAndBiasedSpatial = 2,  ///< Output temporal followed by biased spatial reuse result.
    TemporalAndUnbiasedSpatial = 3 ///< Output temporal followed by unbiased spatial reuse result.
};

FALCOR_ENUM_INFO(
    ReSTIRMode,
    {
        { ReSTIRMode::InputOnly, "InputOnly" },
        { ReSTIRMode::TemporalOnly, "TemporalOnly" },
        { ReSTIRMode::TemporalAndBiasedSpatial, "TemporalAndBiasedSpatial" },
        { ReSTIRMode::TemporalAndUnbiasedSpatial, "TemporalAndUnbiasedSpatial" },
    }
);
FALCOR_ENUM_REGISTER(ReSTIRMode);

enum class SpatialReusePattern
#ifdef HOST_CODE
    : uint32_t
#endif
{
    Default = 0, // same as in ScreenSpaceReSTIR
};

END_NAMESPACE_FALCOR
