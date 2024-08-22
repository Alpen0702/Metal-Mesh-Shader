#include <metal_stdlib>
#include "AAPLShaderTypes.h"

using namespace metal;

struct RasterizerData
{
    float4 position [[position]];
    float3 normal;
    float2 texCoord;
};

vertex RasterizerData indirectDrawVertStageFunction(uint vertexID [[vertex_id]],
                                                    uint instanceID [[instance_id]],
                                                    constant AAPLVertex* vertices [[buffer(AAPLBufferIndexMeshVertices)]],
                                                    constant AAPLInstanceData* instanceData [[buffer(AAPLBufferInstanceData)]],
                                                    constant float4x4* transforms [[buffer(AAPLBufferIndexTransforms)]],
                                                    constant float4x4* viewProjectionMatrix [[buffer(AAPLBufferViewProjectionMatrix)]])
{
    RasterizerData out;
    //out.position = viewProjectionMatrix * transforms[0] * float4(vertices[vertexID].position.xyz * instanceData[instanceID].instanceScale + instanceData[instanceID].instancePos, 1.0f);
    out.normal = vertices[vertexID].normal.xyz;
    out.texCoord = vertices[vertexID].uv;
    out.position = viewProjectionMatrix[0] * transforms[0] * float4(vertices[vertexID].position.xyz * instanceData[instanceID].instanceScale + instanceData[instanceID].instancePos, 1.0f);
    return out;
}

fragment float4 indirectDrawFragStageFunction(RasterizerData in [[stage_in]])
                                              //texture2d<float> texture [[texture(TextureIndexBaseColor)]])
{
    //constexpr sampler linearSampler(mip_filter::linear, mag_filter::linear, min_filter::linear);
    float4 color = float4(1,1,1,1);//texture.sample(linearSampler, in.texCoord);
    //color.rgb = pow(color.rgb, 1.0 / 2.2);
    return color;
}
