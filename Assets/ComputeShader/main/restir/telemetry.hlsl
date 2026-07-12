#ifndef RESTIR_TELEMETRY_INCLUDED
#define RESTIR_TELEMETRY_INCLUDED

#define RESTIR_TELEMETRY_MAGIC 0x52535452u
#define RESTIR_TELEMETRY_SCHEMA 1u
#define RESTIR_TELEMETRY_PACKET_WORDS 1024u
#define RESTIR_TELEMETRY_HEADER_WORDS 32u
#define RESTIR_TELEMETRY_COUNTER_BASE 32u
#define RESTIR_TELEMETRY_COUNTER_COUNT 96u
#define RESTIR_TELEMETRY_RECORD_BASE 128u
#define RESTIR_TELEMETRY_RECORD_WORDS 56u
#define RESTIR_TELEMETRY_RECORD_COUNT 16u

#define RESTIR_HEADER_MAGIC 0u
#define RESTIR_HEADER_SCHEMA 1u
#define RESTIR_HEADER_FRAME 2u
#define RESTIR_HEADER_SAMPLE 3u
#define RESTIR_HEADER_GENERATION 4u
#define RESTIR_HEADER_WIDTH 5u
#define RESTIR_HEADER_HEIGHT 6u
#define RESTIR_HEADER_MODE_FLAGS 7u
#define RESTIR_HEADER_DIRECT_INITIAL_SLOT 8u
#define RESTIR_HEADER_DIRECT_FINAL_SLOT 9u
#define RESTIR_HEADER_INDIRECT_INITIAL_SLOT 10u
#define RESTIR_HEADER_INDIRECT_FINAL_SLOT 11u
#define RESTIR_HEADER_SELECTED_PIXEL 12u

#define RESTIR_COUNTER_SAMPLED_PIXELS 0u
#define RESTIR_COUNTER_DI_INITIAL_ACCEPTED 1u
#define RESTIR_COUNTER_DI_INITIAL_INVALID_SURFACE 2u
#define RESTIR_COUNTER_DI_INITIAL_INVALID_PROPOSAL 3u
#define RESTIR_COUNTER_DI_TEMPORAL_INVALID_CURRENT 4u
#define RESTIR_COUNTER_DI_TEMPORAL_NO_HISTORY 5u
#define RESTIR_COUNTER_DI_TEMPORAL_REPROJECTION_OOB 6u
#define RESTIR_COUNTER_DI_TEMPORAL_INVALID_HISTORY 7u
#define RESTIR_COUNTER_DI_TEMPORAL_INCOMPATIBLE 8u
#define RESTIR_COUNTER_DI_TEMPORAL_REEVALUATION_REJECTED 9u
#define RESTIR_COUNTER_DI_TEMPORAL_HISTORY_COMBINED 10u
#define RESTIR_COUNTER_DI_TEMPORAL_HISTORY_SELECTED 11u
#define RESTIR_COUNTER_DI_TEMPORAL_NONFINITE_OUTPUT 12u
#define RESTIR_COUNTER_DI_SHADE_INVALID_RESERVOIR 13u
#define RESTIR_COUNTER_DI_SHADE_VISIBILITY_REJECTED 14u
#define RESTIR_COUNTER_DI_SHADE_POSITIVE_CONTRIBUTION 15u

#define RESTIR_COUNTER_GI_INITIAL_PRIMARY_MISS 16u
#define RESTIR_COUNTER_GI_INITIAL_INVALID_SECONDARY 17u
#define RESTIR_COUNTER_GI_INITIAL_INVALID_PROPOSAL 18u
#define RESTIR_COUNTER_GI_INITIAL_ZERO_THROUGHPUT 19u
#define RESTIR_COUNTER_GI_INITIAL_ZERO_TARGET 20u
#define RESTIR_COUNTER_GI_INITIAL_NONFINITE 21u
#define RESTIR_COUNTER_GI_INITIAL_ACCEPTED 22u

#define RESTIR_COUNTER_GI_TEMPORAL_INVALID_CURRENT 32u
#define RESTIR_COUNTER_GI_TEMPORAL_NO_HISTORY 33u
#define RESTIR_COUNTER_GI_TEMPORAL_REPROJECTION_OOB 34u
#define RESTIR_COUNTER_GI_TEMPORAL_INVALID_HISTORY 35u
#define RESTIR_COUNTER_GI_TEMPORAL_INCOMPATIBLE 36u
#define RESTIR_COUNTER_GI_TEMPORAL_REEVALUATION_REJECTED 37u
#define RESTIR_COUNTER_GI_TEMPORAL_JACOBIAN_REJECTED 38u
#define RESTIR_COUNTER_GI_TEMPORAL_HISTORY_COMBINED 39u
#define RESTIR_COUNTER_GI_TEMPORAL_HISTORY_SELECTED 40u
#define RESTIR_COUNTER_GI_TEMPORAL_M_CAPPED 41u
#define RESTIR_COUNTER_GI_TEMPORAL_NONFINITE_OUTPUT 42u

#define RESTIR_COUNTER_GI_SPATIAL_INVALID_CURRENT 48u
#define RESTIR_COUNTER_GI_SPATIAL_INVALID_NEIGHBOR 49u
#define RESTIR_COUNTER_GI_SPATIAL_INCOMPATIBLE_NEIGHBOR 50u
#define RESTIR_COUNTER_GI_SPATIAL_REEVALUATION_REJECTED 51u
#define RESTIR_COUNTER_GI_SPATIAL_JACOBIAN_REJECTED 52u
#define RESTIR_COUNTER_GI_SPATIAL_NEIGHBOR_COMBINED 53u
#define RESTIR_COUNTER_GI_SPATIAL_NEIGHBOR_SELECTED 54u
#define RESTIR_COUNTER_GI_SPATIAL_ZERO_NORMALIZATION 55u
#define RESTIR_COUNTER_GI_SPATIAL_M_CAPPED 56u
#define RESTIR_COUNTER_GI_SPATIAL_NONFINITE_OUTPUT 57u

#define RESTIR_COUNTER_GI_FINAL_INVALID_PRIMARY 64u
#define RESTIR_COUNTER_GI_FINAL_INVALID_RESERVOIR 65u
#define RESTIR_COUNTER_GI_FINAL_VISIBILITY_REJECTED 66u
#define RESTIR_COUNTER_GI_FINAL_ZERO_RADIANCE 67u
#define RESTIR_COUNTER_GI_FINAL_ZERO_TARGET 68u
#define RESTIR_COUNTER_GI_FINAL_NONFINITE_WEIGHT 69u
#define RESTIR_COUNTER_GI_FINAL_NONFINITE_CONTRIBUTION 70u
#define RESTIR_COUNTER_GI_FINAL_EXCESSIVE_WEIGHT 71u
#define RESTIR_COUNTER_GI_FINAL_EXCESSIVE_CONTRIBUTION 72u
#define RESTIR_COUNTER_GI_FINAL_POSITIVE_CONTRIBUTION 73u

#define RESTIR_COUNTER_CRITICAL_NONFINITE 80u
#define RESTIR_COUNTER_CRITICAL_OUT_OF_RANGE 81u
#define RESTIR_COUNTER_CRITICAL_BUFFER_CONTRACT 82u

#define RESTIR_STAGE_NONE 0u
#define RESTIR_STAGE_DI_INITIAL 1u
#define RESTIR_STAGE_DI_TEMPORAL 2u
#define RESTIR_STAGE_DI_SHADE 3u
#define RESTIR_STAGE_GI_INITIAL 4u
#define RESTIR_STAGE_GI_TEMPORAL 5u
#define RESTIR_STAGE_GI_SPATIAL 6u
#define RESTIR_STAGE_GI_FINAL 7u

#define RESTIR_REASON_NONE 0u
#define RESTIR_REASON_PRIMARY_MISS 1u
#define RESTIR_REASON_INVALID_SURFACE 2u
#define RESTIR_REASON_INVALID_PROPOSAL_PDF 3u
#define RESTIR_REASON_ZERO_THROUGHPUT 4u
#define RESTIR_REASON_ZERO_TARGET 5u
#define RESTIR_REASON_REPROJECTION_OOB 6u
#define RESTIR_REASON_INVALID_HISTORY 7u
#define RESTIR_REASON_INCOMPATIBLE_SURFACE 8u
#define RESTIR_REASON_REEVALUATION_REJECTED 9u
#define RESTIR_REASON_JACOBIAN_REJECTED 10u
#define RESTIR_REASON_ZERO_NORMALIZATION 11u
#define RESTIR_REASON_VISIBILITY_REJECTED 12u
#define RESTIR_REASON_NONFINITE_RESERVOIR 13u
#define RESTIR_REASON_NONFINITE_WEIGHT 14u
#define RESTIR_REASON_NONFINITE_CONTRIBUTION 15u
#define RESTIR_REASON_EXCESSIVE_WEIGHT 16u
#define RESTIR_REASON_EXCESSIVE_CONTRIBUTION 17u

#if defined(RESTIR_TELEMETRY_ENABLED)

RWByteAddressBuffer ReSTIRTelemetry;
uint _RestirTelemetryEnabled;
uint _RestirTelemetrySampleStride;
uint _RestirTelemetrySamplePhase;
uint _RestirTelemetryGeneration;
uint _RestirTelemetrySampleCount;
uint _RestirTelemetryModeFlags;
uint _RestirTelemetryDirectInitialSlot;
uint _RestirTelemetryDirectFinalSlot;
uint _RestirTelemetryIndirectInitialSlot;
uint _RestirTelemetryIndirectFinalSlot;

uint RestirTelemetryByteOffset(uint wordIndex)
{
    return wordIndex * 4u;
}

bool RestirTelemetryShouldSample(uint pixelIndex)
{
    uint stride = max(_RestirTelemetrySampleStride, 1u);
    return _RestirTelemetryEnabled != 0u && pixelIndex % stride == _RestirTelemetrySamplePhase % stride;
}

void RestirTelemetryCount(uint counterIndex, uint pixelIndex)
{
    if (!RestirTelemetryShouldSample(pixelIndex) || counterIndex >= RESTIR_TELEMETRY_COUNTER_COUNT)
        return;

    uint originalValue;
    ReSTIRTelemetry.InterlockedAdd(
        RestirTelemetryByteOffset(RESTIR_TELEMETRY_COUNTER_BASE + counterIndex),
        1u,
        originalValue);
}

void RestirTelemetryCountCritical(uint counterIndex)
{
    if (_RestirTelemetryEnabled == 0u || counterIndex >= RESTIR_TELEMETRY_COUNTER_COUNT)
        return;

    uint originalValue;
    ReSTIRTelemetry.InterlockedAdd(
        RestirTelemetryByteOffset(RESTIR_TELEMETRY_COUNTER_BASE + counterIndex),
        1u,
        originalValue);
}

bool RestirTelemetryClaimSelectedPixel(uint pixelIndex)
{
    if (!RestirTelemetryShouldSample(pixelIndex))
        return false;

    uint originalValue;
    ReSTIRTelemetry.InterlockedCompareExchange(
        RestirTelemetryByteOffset(RESTIR_HEADER_SELECTED_PIXEL),
        0xffffffffu,
        pixelIndex,
        originalValue);
    return originalValue == 0xffffffffu || originalValue == pixelIndex;
}

uint RestirTelemetrySelectedPixel()
{
    return ReSTIRTelemetry.Load(RestirTelemetryByteOffset(RESTIR_HEADER_SELECTED_PIXEL));
}

void RestirTelemetryStoreFloat4(uint wordIndex, float4 value)
{
    ReSTIRTelemetry.Store4(RestirTelemetryByteOffset(wordIndex), asuint(value));
}

void RestirTelemetryWriteRecord(
    uint recordIndex,
    uint stage,
    uint reason,
    uint pixelIndex,
    float4 data0,
    float4 data1,
    float4 data2,
    float4 data3,
    float4 data4,
    float4 data5)
{
    if (!RestirTelemetryShouldSample(pixelIndex) || recordIndex >= RESTIR_TELEMETRY_RECORD_COUNT)
        return;

    uint baseWord = RESTIR_TELEMETRY_RECORD_BASE + recordIndex * RESTIR_TELEMETRY_RECORD_WORDS;
    uint originalValue;
    ReSTIRTelemetry.InterlockedCompareExchange(
        RestirTelemetryByteOffset(baseWord),
        0u,
        1u,
        originalValue);
    if (originalValue != 0u)
        return;

    ReSTIRTelemetry.Store(RestirTelemetryByteOffset(baseWord + 1u), stage);
    ReSTIRTelemetry.Store(RestirTelemetryByteOffset(baseWord + 2u), reason);
    ReSTIRTelemetry.Store(RestirTelemetryByteOffset(baseWord + 3u), pixelIndex);
    RestirTelemetryStoreFloat4(baseWord + 4u, data0);
    RestirTelemetryStoreFloat4(baseWord + 8u, data1);
    RestirTelemetryStoreFloat4(baseWord + 12u, data2);
    RestirTelemetryStoreFloat4(baseWord + 16u, data3);
    RestirTelemetryStoreFloat4(baseWord + 20u, data4);
    RestirTelemetryStoreFloat4(baseWord + 24u, data5);
    ReSTIRTelemetry.Store(RestirTelemetryByteOffset(baseWord + 28u), _RestirTelemetryGeneration);
    ReSTIRTelemetry.Store(RestirTelemetryByteOffset(baseWord + 29u), _RestirTelemetrySampleCount);
}

#else

void RestirTelemetryCount(uint counterIndex, uint pixelIndex) {}
void RestirTelemetryCountCritical(uint counterIndex) {}
bool RestirTelemetryClaimSelectedPixel(uint pixelIndex) { return false; }
uint RestirTelemetrySelectedPixel() { return 0xffffffffu; }
void RestirTelemetryWriteRecord(
    uint recordIndex,
    uint stage,
    uint reason,
    uint pixelIndex,
    float4 data0,
    float4 data1,
    float4 data2,
    float4 data3,
    float4 data4,
    float4 data5) {}

#endif

#endif
