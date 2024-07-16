#include <metal_stdlib>
#include "AAPLShaderTypes.h"

using namespace metal;

struct vertexOut
{
    float4 position [[position]];
    float3 normal;
};

struct payload_t
{
    AAPLVertex vertices[AAPLMaxMeshletVertexCount];
    float4x4 transform;
    float3 color;

    uint32_t primitiveCount;
    uint32_t vertexCount;
    uint32_t instanceOffset;

    /// The array of vertex indices for the meshlet into the vertices array.
    ///
    /// The object stage uses this to copy indices into the payload.
    /// The mesh stage uses this to set the indices for the geometry.
    uint8_t indices[1024];
};

// Per-vertex primitive data.
struct primOut
{
    float3 color;
};

struct fragmentIn
{
    vertexOut v;
    primOut p;
};

/// Defines a mesh declaration type that supports triangles.
using AAPLTriangleMeshType = metal::mesh<vertexOut, primOut, AAPLMaxMeshletVertexCount, AAPLMaxPrimitiveCount, metal::topology::triangle>;

/// The object stage that generates one submesh group.
[[object, max_total_threads_per_threadgroup(AAPLMaxTotalThreadsPerObjectThreadgroup), max_total_threadgroups_per_mesh_grid(AAPLMaxThreadgroupsPerMeshGrid)]]
void meshShaderObjectStageFunction(object_data payload_t& payload            [[payload]],
                                   mesh_grid_properties meshGridProperties,
                                   constant AAPLMeshInfo& meshInfo           [[buffer(AAPLBufferIndexMeshInfo)]],
                                   constant AAPLVertex* vertices             [[buffer(AAPLBufferIndexMeshVertices)]],
                                   constant AAPLIndexType* indices           [[buffer(AAPLBufferIndexMeshIndices)]],
                                   constant float4x4*   transforms           [[buffer(AAPLBufferIndexTransforms)]],
                                   constant float4x4&   viewProjectionMatrix [[buffer(AAPLBufferViewProjectionMatrix)]],
                                   uint3                positionInGrid       [[threadgroup_position_in_grid]])
{

    payload.color = meshInfo.color.xyz;
    
    uint startIndex = meshInfo.startIndex;
    uint startVertexIndex = meshInfo.startVertexIndex;
    
    payload.primitiveCount = meshInfo.primitiveCount;
    payload.vertexCount = meshInfo.vertexCount;
    payload.instanceOffset = meshInfo.instanceOffset + positionInGrid.x;


    // Copy the triangle indices into the payload.
    for (uint i = 0; i < payload.primitiveCount * 3; i++)
    {
        payload.indices[i] = indices[startIndex + i] - startVertexIndex;
    }

    // Copy the vertex data into the payload.
    for (size_t i = 0; i < payload.vertexCount; i++)
    {
        payload.vertices[i] = vertices[startVertexIndex + i];
    }
    
    // Concatenate the view projection matrix to the model transform matrix.
    payload.transform = viewProjectionMatrix * transforms[0];

    // Set the output submesh count for the mesh shader.
    // Because the mesh shader is only producing one mesh, the threadgroup grid size is 1 x 1 x 1.
    meshGridProperties.set_threadgroups_per_grid(uint3(1, 1, 1));
    //if(meshInfo.instanceCount==0)
        //meshGridProperties.set_threadgroups_per_grid(uint3(1500, 1, 1));

}

/// The mesh stage function that generates a triangle mesh.
[[mesh, max_total_threads_per_threadgroup(AAPLMaxPrimitiveCount)]]
void meshShaderMeshStageFunction(AAPLTriangleMeshType output,
                                 const object_data payload_t& payload [[payload]],
                                 constant AAPLInstanceData* instanceData   [[buffer(AAPLBufferInstanceData)]],
                                 uint lid [[thread_index_in_threadgroup]],
                                 uint tid [[threadgroup_position_in_grid]])
{
    // Set the number of primitives for the entire mesh.
    output.set_primitive_count(payload.primitiveCount);
    
    // Apply the transformation matrix to all the vertex data.
    // For performance, place the vertices common to all LODs in the first part of the buffer and then limit the vertex count.
    if (lid < payload.vertexCount)
    {
        vertexOut v;
        int instanceID = payload.instanceOffset;
        float4 position = float4(payload.vertices[lid].position.xyz * instanceData[instanceID].instanceScale + instanceData[instanceID].instancePos, 1.0f);

        v.position = payload.transform * position;
        v.normal = normalize(payload.vertices[lid].normal.xyz);
        output.set_vertex(lid, v);
    }
    
    // Set the constant data for the entire primitive.
    if (lid < payload.primitiveCount)
    {
        primOut p;
        p.color = payload.color;
        output.set_primitive(lid, p);

        // Set the output indices.
        uint i = (3*lid);
        output.set_index(i+0, payload.indices[i+0]);
        output.set_index(i+1, payload.indices[i+1]);
        output.set_index(i+2, payload.indices[i+2]);
    }
}

/// The fragment shader that blends a bit of Lambertian light with the meshlet's normal and mesh color.
fragment float4 fragmentShader(fragmentIn in [[stage_in]])
{
    float3 N = normalize(in.v.normal);
    float3 L = normalize(float3(1, 1, 1));

    // The dot product of the normal and light direction isn't clamped and makes
    // the intensity of the mesh color range between `0.5` and `1.0`.
    float3 ambientIntensity = float3(1.0);
    float3 lightIntensity = float3(100.0);
    float3 colorIntensity = ambientIntensity + lightIntensity * (0.5 + 0.5 * dot(N, L));

    // Reduce the dynamic range by a simple tone-mapping operator.
    colorIntensity = colorIntensity / (1.0 + colorIntensity);

    // Change the range of the normal from [-1, 1] to [0, 1] to use it as a color.
    float3 normalColor = N * 0.5 + 0.5;

    // Add a 20% mix of the normal color with the shaded meshlet color.
    return float4(mix(colorIntensity * in.p.color, normalColor, 0.2), 1.0f);
}
