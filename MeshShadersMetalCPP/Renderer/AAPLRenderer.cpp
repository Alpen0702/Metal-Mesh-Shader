/*
See LICENSE folder for this sample’s licensing information.

Abstract:
The renderer's mesh shader implementation that draws bicubic Bezier patches.
*/

#include <MetalKit/MetalKit.hpp>
#include <simd/simd.h>
#include <vector>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include "load_gltf.h"
#include "AAPLRenderer.hpp"

constexpr bool _useMultisampleAntialiasing = true;

#pragma mark - Matrix Math Utilities

/// Returns a scaling matrix.
matrix_float4x4 matrix4x4_scaling(float s)
{
    return simd_matrix_from_rows(
        simd_make_float4(s, 0, 0, 0),  // Row 1
        simd_make_float4(0, s, 0, 0),  // Row 2
        simd_make_float4(0, 0, s, 0),  // Row 3
        simd_make_float4(0, 0, 0, 1)); // Row 4
}

/// Returns a translation matrix.
matrix_float4x4 matrix4x4_translation(float tx, float ty, float tz)
{
    return simd_matrix_from_rows(
        simd_make_float4(1, 0, 0, tx), // Row 1
        simd_make_float4(0, 1, 0, ty), // Row 2
        simd_make_float4(0, 0, 1, tz), // Row 3
        simd_make_float4(0, 0, 0, 1)); // Row 4
}

/// Returns a rotation matrix.
matrix_float4x4 matrix4x4_ZRotate(float angleRadians)
{
    const float a = angleRadians;
    return simd_matrix_from_rows(
        simd_make_float4(cosf(a), sinf(a), 0.0f, 0.0f),  // Row 1
        simd_make_float4(-sinf(a), cosf(a), 0.0f, 0.0f), // Row 2
        simd_make_float4(0.0f, 0.0f, 1.0f, 0.0f),        // Row 3
        simd_make_float4(0.0f, 0.0f, 0.0f, 1.0f));       // Row 4
}

/// Returns a rotation matrix.
matrix_float4x4 matrix4x4_YRotate(float angleRadians)
{
    const float a = angleRadians;
    return simd_matrix_from_rows(
        simd_make_float4(cosf(a), 0.0f, sinf(a), 0.0f),  // Row 1
        simd_make_float4(0.0f, 1.0f, 0.0f, 0.0f),        // Row 2
        simd_make_float4(-sinf(a), 0.0f, cosf(a), 0.0f), // Row 3
        simd_make_float4(0.0f, 0.0f, 0.0f, 1.0f));       // Row 4
}

/// Returns a perspective transform matrix.
matrix_float4x4 matrix_perspective_right_hand(float fovyRadians, float aspect, float nearZ, float farZ)
{
    float ys = 1 / tanf(fovyRadians * 0.5);
    float xs = ys / aspect;
    float zs = farZ / (nearZ - farZ);
    return simd_matrix_from_rows(
        simd_make_float4(xs, 0, 0, 0),          // Row 1
        simd_make_float4(0, ys, 0, 0),          // Row 2
        simd_make_float4(0, 0, zs, nearZ * zs), // Row 3
        simd_make_float4(0, 0, -1, 0)           // Row 4
    );
}

/// Calculates the Berstein basis function with index 3, and subindex i.
float bernsteinBasisCubic(float u, int i)
{
    constexpr float n_choose_i[4] = {1, 3, 3, 1};
    return n_choose_i[i] * powf(u, float(i)) * powf(1.0f - u, float(3 - i));
}

/// Returns the bicubic patch point where k contains 16 elements.
simd_float4 bicubicPoint(float u, float v, std::vector<simd_float3> controlPoints)
{
    simd_float3 p = simd_make_float3(0, 0, 0);
    int k = 0;
    for (int i = 0; i <= 3; i++) {
        for (int j = 0; j <= 3; j++) {
            float Bni_u = bernsteinBasisCubic(u, i);
            float Bmj_v = bernsteinBasisCubic(v, j);
            p += Bni_u * Bmj_v * controlPoints[k];
            k++;
        }
    }
    return simd_make_float4(p.x, p.y, p.z, 1.0f);
}

/// Returns a point on a bicubic patch.
///
/// The bicubic patch point is in parametric coordinates (u, v).
simd_float4 bicubicPatch(int shape, float u, float v)
{
    static int gshape = -1;
    static std::vector<simd_float3> controlPoints;
    if (gshape != shape)
    {
        // Generate control points if this object wasn't used before.
        controlPoints.resize(16);
        int k = 0;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                controlPoints[k].x = i / 3.0f - 0.5f;
                controlPoints[k].y = j / 3.0f - 0.5f;
                controlPoints[k].z = -0.5 + 0.5 * drand48();
                k++;
            }
        }
        gshape = shape;
    }
    return bicubicPoint(u, v, controlPoints);
}

/// Returns a point on a bicubic patch with coordinates (u, v) for one of the bicubic patches.
simd_float3 bicubicPatch3(int shape, float u, float v)
{
    simd_float4 p = bicubicPatch(shape, u, v);
    return simd_make_float3(p.x, p.y, p.z);
}

/// Check for errors with the given NSError and reset the pointer.
void handleError(NS::Error** pError)
{
    if (*pError)
    {
        fprintf(stderr, "%s", (*pError)->localizedDescription()->utf8String());

        assert(false);
    }
    *pError = nullptr;
}

#pragma mark - AAPLRenderer implementation

/// Initializes the renderer with a view.
AAPLRenderer::AAPLRenderer(MTK::View& view)
{
    degree = 0.0f;
    _pDevice = view.device()->retain();
    view.setDepthStencilPixelFormat(MTL::PixelFormatDepth32Float_Stencil8);
    if (_useMultisampleAntialiasing)
        view.setSampleCount(4);

    _pCommandQueue = _pDevice->newCommandQueue();
    for (size_t i = 0; i < AAPLMaxFramesInFlight; i++) {
        _pTransformsBuffer[i] = _pDevice->newBuffer(AAPLNumObjectsXYZ * sizeof(matrix_float4x4), MTL::ResourceStorageModeShared);
    }
    _pMeshColorsBuffer = _pDevice->newBuffer(AAPLNumObjectsXYZ * sizeof(vector_float3), MTL::ResourceStorageModeShared);
    const size_t LODCount = 3;
    _pMeshVerticesBuffer = _pDevice->newBuffer(AAPLNumObjectsXYZ * sizeof(AAPLVertex) * AAPLMaxMeshletVertexCount * LODCount, MTL::ResourceStorageModeShared);
    _pMeshIndicesBuffer = _pDevice->newBuffer(AAPLNumObjectsXYZ * sizeof(AAPLIndexType) * AAPLMaxPrimitiveCount * 6 * LODCount, MTL::ResourceStorageModeShared);
    _pMeshInfoBuffer = _pDevice->newBuffer(AAPLNumObjectsXYZ * sizeof(AAPLMeshInfo), MTL::ResourceStorageModeShared);
    _pInstanceDataBuffer = _pDevice->newBuffer(AAPLNumObjectsXYZ * sizeof(AAPLInstanceData), MTL::ResourceStorageModeShared);
    
    
    //loadGLTF("/Users/liyangyang/Desktop/leiluo/Git\ Mesh\ Shader/Metal-Mesh-Shader/assets/scenes/simpRocks.gltf", meshVertices, meshIndices);
    loadGLTF("assets/scenes/simpRocks.gltf", meshVertices, meshIndices);

    
    
    buildShaders();
    makeMeshlets();
    makeMeshletColors();
    prepareInstanceData();
}

/// Releases the renderer's GPU resources, including buffers and pipeline states.
AAPLRenderer::~AAPLRenderer()
{
    _pDevice->release();
    _pCommandQueue->release();
    _pRenderPipelineState->release();
    _pDepthStencilState->release();
    _pMeshVerticesBuffer->release();
    _pMeshIndicesBuffer->release();
    _pMeshInfoBuffer->release();
    _pInstanceDataBuffer->release();
    _pMeshColorsBuffer->release();
    for (size_t i = 0; i < AAPLMaxFramesInFlight; i++) {
        _pTransformsBuffer[i]->release();
    }
}

/// Builds the vertex, fragment, and mesh shader pipelines.
void AAPLRenderer::buildShaders()
{
    using NS::StringEncoding::UTF8StringEncoding;

    NS::Error* pError = nullptr;
    MTL::Library* pLibrary = _pDevice->newDefaultLibrary();
    handleError(&pError);

    // Set up the mesh shading pipeline.
    MTL::MeshRenderPipelineDescriptor* pMeshDesc = MTL::MeshRenderPipelineDescriptor::alloc()->init();
    MTL::Function* pFragFn = pLibrary->newFunction(NS::String::string("fragmentShader", UTF8StringEncoding));

    // All three mesh shaders use the following common properties.
    pMeshDesc->setFragmentFunction(pFragFn);
    pMeshDesc->colorAttachments()->object(0)->setPixelFormat(MTL::PixelFormat::PixelFormatBGRA8Unorm);
    pMeshDesc->setDepthAttachmentPixelFormat(MTL::PixelFormat::PixelFormatDepth16Unorm);
    if (_useMultisampleAntialiasing)
        pMeshDesc->setRasterSampleCount(4);

    pMeshDesc->setMaxTotalThreadsPerObjectThreadgroup(AAPLMaxTotalThreadsPerObjectThreadgroup);
    pMeshDesc->setMaxTotalThreadsPerMeshThreadgroup(AAPLMaxTotalThreadsPerMeshThreadgroup);


    int topology = 0;
    MTL::FunctionConstantValues* pConstantValues = MTL::FunctionConstantValues::alloc()->init();
    pConstantValues->setConstantValue(&topology, MTL::DataTypeInt, AAPL_FUNCTION_CONSTANT_TOPOLOGY);

    MTL::Function* pMeshFn = pLibrary->newFunction(NS::String::string("meshShaderMeshStageFunction", UTF8StringEncoding), pConstantValues, &pError);
    handleError(&pError);
    pMeshDesc->setMeshFunction(pMeshFn);

    MTL::Function* pObjectFn = pLibrary->newFunction(NS::String::string("meshShaderObjectStageFunction", UTF8StringEncoding), pConstantValues, &pError);
    handleError(&pError);
    pMeshDesc->setObjectFunction(pObjectFn);

    _pRenderPipelineState = _pDevice->newRenderPipelineState(pMeshDesc, MTL::PipelineOptionNone, nullptr, &pError);
    pObjectFn->release();
    pMeshFn->release();
    pConstantValues->release();

    // Set up the depth-stencil state object.
    MTL::DepthStencilDescriptor* depthStencilDesc = MTL::DepthStencilDescriptor::alloc()->init();
    depthStencilDesc->setDepthCompareFunction(MTL::CompareFunctionLess);
    depthStencilDesc->setDepthWriteEnabled(true);
    _pDepthStencilState = _pDevice->newDepthStencilState(depthStencilDesc);

    handleError(&pError);

    pFragFn->release();
    pMeshDesc->release();
    pLibrary->release();
    depthStencilDesc->release();
}

/// Initializes the meshlet vertex data for all the bicubic patches.
void AAPLRenderer::makeMeshlets()
{
    size_t segX = AAPLNumPatchSegmentsX;
    size_t segY = AAPLNumPatchSegmentsY;

    meshInfo.resize(AAPLNumObjectsXYZ);
    for (int i = 0; i < AAPLNumObjectsXYZ; i++)
    {
        AAPLMeshInfo& mesh = meshInfo[i];
        mesh.patchIndex = i;
        mesh.color = simd_make_float4(1.0, 0.0, 1.0, 1.0);
        mesh.numLODs = 1;
        

        mesh.lod1.startVertexIndex = 0;
        mesh.lod1.startIndex = 0;
        mesh.lod1.lastIndex = 6;
        mesh.lod1.vertexCount = 4;
            
        // Set the number of triangles.
        mesh.lod1.primitiveCount = (mesh.lod1.lastIndex - mesh.lod1.startIndex) / 3;

    }
    
    // Tell Metal when the buffer contents change.
    assert(_pMeshVerticesBuffer->length() >= meshVertices.size() * sizeof(AAPLVertex));
    assert(_pMeshIndicesBuffer->length() >= meshIndices.size() * sizeof(AAPLIndexType));
    memcpy(_pMeshVerticesBuffer->contents(), meshVertices.data(), sizeof(AAPLVertex) * meshVertices.size());
    memcpy(_pMeshIndicesBuffer->contents(), meshIndices.data(), sizeof(AAPLIndexType) * meshIndices.size());
    memcpy(_pMeshInfoBuffer->contents(), meshInfo.data(), sizeof(AAPLMeshInfo) * meshInfo.size());
}

/// Sets up the color for each bicubic patch.
void AAPLRenderer::makeMeshletColors()
{
    simd_float3* meshColors = reinterpret_cast<simd_float3*>(_pMeshColorsBuffer->contents());

    int count = 0;

    float x_div = 1.0f / (AAPLNumObjectsX + 1);
    float y_div = 1.0f / AAPLNumObjectsY;
    float z_div = 1.0f / AAPLNumObjectsZ;

    for (size_t z = 0; z < AAPLNumObjectsZ; ++z)
    {
        for (size_t y = 0; y < AAPLNumObjectsY; ++y)
        {
            for (size_t x = 0; x < AAPLNumObjectsX; ++x)
            {
                meshColors[count] = simd_make_float3((x + 1.0f) * x_div, y * y_div, (1.0f + z) * z_div);
                meshColors[count] = simd_normalize(meshColors[count]) * 0.75f;
                count++;
            }
        }
    }
}

void AAPLRenderer::prepareInstanceData()
{
    int instanceCount = 1000;
    std::vector<InstanceData> instance_data(instanceCount);

    std::default_random_engine              rnd_generator(static_cast<unsigned>(time(nullptr)));
    std::uniform_real_distribution<float>   uniform_dist(0.0, 1.0);

    // Distribute rocks randomly on two different rings
    for (auto i = 0; i < instanceCount / 2; i++)
    {
        simd_float2 ring0{1.0f, 2.0f};
        simd_float2 ring1{2, 3.5f};

        float rho, theta;
        // Inner ring

        rho                       = sqrt((pow(ring0[1], 2.0f) - pow(ring0[0], 2.0f)) * uniform_dist(rnd_generator) + pow(ring0[0], 2.0f));
        theta                     = 2.0f * 3.14 * uniform_dist(rnd_generator);
        instance_data[i].pos      = simd_float3(rho * cos(theta), uniform_dist(rnd_generator) * 0.5f - 0.25f, rho * sin(theta));
        instance_data[i].rot      = simd_float3(0, 0, 0);
        instance_data[i].scale    = 1.5f + uniform_dist(rnd_generator) - uniform_dist(rnd_generator);
        instance_data[i].instanceIndex = i;
        instance_data[i].scale *= 0.75f;

        // Outer ring
        rho                                                                 = sqrt((pow(ring1[1], 2.0f) - pow(ring1[0], 2.0f)) * uniform_dist(rnd_generator) + pow(ring1[0], 2.0f));
        theta                                                               = 2.0f * 3.14 * uniform_dist(rnd_generator);
        instance_data[static_cast<size_t>(i + instanceCount / 2)].pos      = simd_float3(rho * cos(theta), uniform_dist(rnd_generator) * 0.5f - 0.25f, rho * sin(theta));
        instance_data[static_cast<size_t>(i + instanceCount / 2)].rot      =  simd_float3(0, 0, 0);
        instance_data[static_cast<size_t>(i + instanceCount / 2)].scale    = 1.5f + uniform_dist(rnd_generator) - uniform_dist(rnd_generator);
        instance_data[static_cast<size_t>(i + instanceCount / 2)].instanceIndex = static_cast<size_t>(i + instanceCount / 2);
        instance_data[static_cast<size_t>(i + instanceCount / 2)].scale *= 0.75f;
    }
    memcpy(_pInstanceDataBuffer->contents(), instance_data.data(), sizeof(AAPLInstanceData) * instanceCount);
}

/// Updates the object transform matrix state before other methods encode any render commands.
void AAPLRenderer::updateStage()
{
    // Get the array pointers for the buffers.
    matrix_float4x4* transforms = reinterpret_cast<matrix_float4x4*>(_pTransformsBuffer[_curFrameInFlight]->contents());

    int count = 0;
    degree += rotationSpeed * M_PI / 180.0f;

    for (size_t z = 0; z < AAPLNumObjectsZ; ++z)
    {
        float z_pos = -12.0f - z * 2.0f;
        for (size_t y = 0; y < AAPLNumObjectsY; ++y)
        {
            float y_pos = 2 * (y - (float(AAPLNumObjectsY - 1) / 2));
            for (size_t x = 0; x < AAPLNumObjectsX; ++x)
            {
                float x_pos = 2 * (x - (float(AAPLNumObjectsX - 1) / 2));
                transforms[count] = matrix_multiply(matrix4x4_translation(x_pos, y_pos, z_pos), matrix4x4_YRotate(degree));
                count++;
            }
        }
    }
}

/// Draws the mesh shaders scene.
void AAPLRenderer::draw(MTK::View* pView)
{
    _curFrameInFlight = (_curFrameInFlight + 1) % AAPLMaxFramesInFlight;

    NS::AutoreleasePool* pPool = NS::AutoreleasePool::alloc()->init();

    // Get a command buffer and start a render command encoder.
    MTL::CommandBuffer* pCommandBuffer = _pCommandQueue->commandBuffer();
    MTL::RenderPassDescriptor* pRenderPassDesc = pView->currentRenderPassDescriptor();
    pRenderPassDesc->colorAttachments()->object(0)->setClearColor(MTL::ClearColor(0.65f, 0.75f, 0.85f, 1.0f));
    MTL::RenderCommandEncoder* pRenderEncoder = pCommandBuffer->renderCommandEncoder(pRenderPassDesc);

    matrix_float4x4 viewMatrix = matrix4x4_translation(0, offsetY, -10 + 10 * offsetZ);
    matrix_float4x4 viewProjectionMatrix = matrix_multiply(_projectionMatrix, viewMatrix);

    // Update the object positions.
    updateStage();

    pRenderEncoder->setFrontFacingWinding(MTL::Winding::WindingCounterClockwise);
    pRenderEncoder->setRenderPipelineState(_pRenderPipelineState);
    pRenderEncoder->setDepthStencilState(_pDepthStencilState);

    // Pass data to the object stage.
    pRenderEncoder->setObjectBuffer(_pMeshVerticesBuffer, 0, AAPLBufferIndexMeshVertices);
    pRenderEncoder->setObjectBuffer(_pMeshIndicesBuffer, 0, AAPLBufferIndexMeshIndices);
    pRenderEncoder->setObjectBuffer(_pMeshInfoBuffer, 0, AAPLBufferIndexMeshInfo);
    
    pRenderEncoder->setObjectBuffer(_pTransformsBuffer[_curFrameInFlight], 0, AAPLBufferIndexTransforms);
    pRenderEncoder->setObjectBuffer(_pMeshColorsBuffer, 0, AAPLBufferIndexMeshColor);
    pRenderEncoder->setObjectBytes(&viewProjectionMatrix, sizeof(viewProjectionMatrix), AAPLBufferViewProjectionMatrix);
    pRenderEncoder->setObjectBytes(&lodChoice, 4, AAPLBufferIndexLODChoice);

    pRenderEncoder->setObjectBuffer(_pInstanceDataBuffer, 0, AAPLBufferInstanceData);

    // Pass data to the mesh stage.
    pRenderEncoder->setMeshBytes(&viewProjectionMatrix, sizeof(viewProjectionMatrix), AAPLBufferViewProjectionMatrix);

    /// Draw objects using the mesh shaders.
    /// Parameter 1: threadgroupsPerGrid ... X=`AAPLNumObjectsX`, Y=`AAPLNumObjectsY`, ...
    /// Parameter 2: threadsPerObjectThreadgroup ... `AAPLMaxTotalThreadsPerObjectThreadgroup`
    /// Parameter 3: threadsPerMeshGroup ... X = `AAPLMaxTotalThreadsPerMeshThreadgroup` has a limit of 64 vertices per meshlet in this sample.
    ///
    /// The object shader copies vertices, indices, and other relevant data to the payload and generates the submesh groups.
    /// The parameter `positionInGrid` (`threadgroup_position_in_grid`) in the shader addresses the submesh.
    /// This tells the object shader the index of the transform for the submesh.
    /// The mesh shader uses the payload to generate the primitives (points, lines, or triangles).
    pRenderEncoder->drawMeshThreadgroups(MTL::Size(AAPLNumObjectsX, AAPLNumObjectsY, AAPLNumObjectsZ),
                                         MTL::Size(AAPLMaxTotalThreadsPerObjectThreadgroup, 1, 1),
                                         MTL::Size(AAPLMaxTotalThreadsPerMeshThreadgroup, 1, 1));

    pRenderEncoder->endEncoding();
    pCommandBuffer->presentDrawable(pView->currentDrawable());
    pCommandBuffer->commit();

    pPool->release();
}

/// Responds to changes in the drawable's size or the device's orientation.
void AAPLRenderer::drawableSizeWillChange(CGSize size)
{
    float aspect = size.width / (float)size.height;
    _projectionMatrix = matrix_perspective_right_hand(65.0f * (M_PI / 180.0f), aspect, 0.1f, 100.0f);
}
