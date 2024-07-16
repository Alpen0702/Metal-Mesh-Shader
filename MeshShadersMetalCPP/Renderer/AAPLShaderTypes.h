/*
See LICENSE folder for this sample’s licensing information.

Abstract:
This file provides definitions that both the app and the shader use.
*/

#pragma once

#ifdef __METAL_VERSION__
#define NSInteger metal::int32_t
#else
#define constant
#include <Foundation/Foundation.hpp>
#endif

#include <simd/simd.h>

typedef enum BufferIndex : int32_t
{
    AAPLBufferIndexMeshVertices = 0,
    AAPLBufferIndexMeshIndices = 1,
    AAPLBufferIndexMeshInfo = 2,
    AAPLBufferViewProjectionMatrix = 3,
    AAPLBufferIndexTransforms = 4,
    AAPLBufferInstanceData = 5
} BufferIndex;

typedef struct AAPLVertex
{
    simd_float4 position;
    simd_float4 normal;
    simd_float2 uv;
} AAPLVertex;

typedef struct AAPLMeshInfo
{
    simd_float4 color;
    
    uint32_t instanceCount{0};
    uint32_t instanceOffset{0};
    
    // This is the first offset into the indices array.
    uint32_t startIndex{0};
    // This is one past the first offset into the indices array.
    uint32_t lastIndex{0};
    // This is the index of the first vertex in the vertex array.
    uint32_t startVertexIndex{0};
    uint32_t vertexCount{0};
    uint32_t primitiveCount{0};

} AAPLMeshInfo;

typedef struct AAPLInstanceData
{
    simd_float3 instancePos;
    simd_float3 instanceRot;
    float instanceScale;
    uint16_t instanceIndex;
} AAPLInstanceData;

/// Declare the constant data for the entire frame in this structure.
typedef struct
{
    simd_float4x4 viewProjectionMatrix;
    simd_float4x4 inverseTransform;
} AAPLFrameData;

using AAPLIndexType = uint16_t;

static constexpr constant uint32_t AAPLNumTasks = 140;

static constexpr constant uint32_t AAPLMaxMeshletVertexCount = 183;
static constexpr constant uint32_t AAPLMaxMeshletIndicesCount = 306;
static constexpr constant uint32_t AAPLMaxPrimitiveCount = 126;

static constexpr constant uint32_t AAPLMaxTotalThreadsPerObjectThreadgroup = 1;
static constexpr constant uint32_t AAPLMaxTotalThreadsPerMeshThreadgroup = AAPLMaxPrimitiveCount;
static constexpr constant uint32_t AAPLMaxThreadgroupsPerMeshGrid = 8;

static constexpr constant uint32_t AAPL_FUNCTION_CONSTANT_TOPOLOGY = 0;
