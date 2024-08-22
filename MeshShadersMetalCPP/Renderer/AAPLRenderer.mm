/*
See LICENSE folder for this sample’s licensing information.

Abstract:
The renderer's mesh shader implementation that draws bicubic Bezier patches.
*/

#include <MetalKit/MetalKit.hpp>
#include <Metal/Metal.h>
#include <simd/simd.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <random>
#include "load_gltf.h"
#include "texture_loader_bridge.hpp"
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

/// Returns a rotation matrix.
matrix_float4x4 matrix4x4_XRotate(float angleRadians)
{
    const float a = angleRadians;
    return simd_matrix_from_rows(
        simd_make_float4(1.0f, 0.0f, 0.0f, 0.0f),  // Row 1
        simd_make_float4(0.0f, cosf(a), sinf(a), 0.0f),        // Row 2
        simd_make_float4(0.0f, -sinf(a), cosf(a), 0.0f), // Row 3
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
        _pTransformsBuffer[i] = _pDevice->newBuffer(sizeof(matrix_float4x4), MTL::ResourceStorageModeShared);
    }

    _pMeshVerticesBuffer = _pDevice->newBuffer(sizeof(AAPLVertex) * AAPLMaxMeshletVertexCount, MTL::ResourceStorageModeShared);
    _pMeshIndicesBuffer = _pDevice->newBuffer(sizeof(AAPLIndexType) * AAPLMaxMeshletIndicesCount, MTL::ResourceStorageModeShared);
    _pInstanceDataBuffer = _pDevice->newBuffer(sizeof(AAPLInstanceData) * instanceCount, MTL::ResourceStorageModeShared);
    _pViewProjectionBuffer = _pDevice->newBuffer(sizeof(matrix_float4x4), MTL::ResourceStorageModeShared);
    
    
    //loadGLTF("/Users/liyangyang/Desktop/leiluo/Git\ Mesh\ Shader/Metal-Mesh-Shader/assets/scenes/simpRocks.gltf", meshVertices, meshIndices);
    loadGLTF("assets/scenes/simpRocks.gltf", meshVertices, meshIndices);
    
    void* loader = createTextureLoader();
    MTL::Device* device = _pDevice;
    texture = loadTexture(loader, "single_astc", device);
//    for (int i = 0; i < 300; i++)
//        texArray[i] = loadTexture(loader, "single_astc", device);
    
    buildShaders();
    makeMeshlets();
    prepareInstanceData();
    prepareIndirectCmdBuffer();
}

/// Releases the renderer's GPU resources, including buffers and pipeline states.
AAPLRenderer::~AAPLRenderer()
{
    _pDevice->release();
    _pCommandQueue->release();
    _pRenderPipelineStateIndirectDraw->release();
    _pRenderPipelineStateMeshShader->release();
    _pDepthStencilState->release();
    _pMeshVerticesBuffer->release();
    _pMeshIndicesBuffer->release();
    _pInstanceDataBuffer->release();
    _pViewProjectionBuffer->release();
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

    // Set up the indirect draw pipeline.
    MTL::RenderPipelineDescriptor* pIndirectDesc = MTL::RenderPipelineDescriptor::alloc()->init();

    pIndirectDesc->colorAttachments()->object(0)->setPixelFormat(MTL::PixelFormat::PixelFormatBGRA8Unorm);
    pIndirectDesc->setDepthAttachmentPixelFormat(MTL::PixelFormatDepth32Float_Stencil8);
    pIndirectDesc->setStencilAttachmentPixelFormat(MTL::PixelFormatDepth32Float_Stencil8);
    if (_useMultisampleAntialiasing)
        pIndirectDesc->setRasterSampleCount(4);
    pIndirectDesc->setSupportIndirectCommandBuffers(true);
    
    MTL::Function* pVertFnIndirect = pLibrary->newFunction(NS::String::string("indirectDrawVertStageFunction", UTF8StringEncoding));
    pIndirectDesc->setVertexFunction(pVertFnIndirect);
    
    MTL::Function* pFragFnIndirect = pLibrary->newFunction(NS::String::string("indirectDrawFragStageFunction", UTF8StringEncoding));
    pIndirectDesc->setFragmentFunction(pFragFnIndirect);
    
    _pRenderPipelineStateIndirectDraw = _pDevice->newRenderPipelineState(pIndirectDesc, MTL::PipelineOptionNone, nullptr, &pError);
    handleError(&pError);

    pVertFnIndirect->release();
    pFragFnIndirect->release();
    pIndirectDesc->release();
    
    
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

    _pRenderPipelineStateMeshShader = _pDevice->newRenderPipelineState(pMeshDesc, MTL::PipelineOptionNone, nullptr, &pError);
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

    meshInfo.resize(AAPLNumTasks);
    for (int i = 0; i < AAPLNumTasks; i++)
    {
        AAPLMeshInfo& mesh = meshInfo[i];
        mesh.color = simd_make_float4(1.0, 0.0, 1.0, 1.0);

        mesh.startVertexIndex = 4;
        mesh.startIndex = 6;
        mesh.lastIndex = 198;
        mesh.vertexCount = 103;

        if (i < 50)
        {
            mesh.instanceCount = 300;
            mesh.instanceOffset = 300 * i;
            mesh.color = simd_make_float4(0.0, 0.0, 1.0, 1.0);
        }
        else if (i < 100)
        {
            mesh.startVertexIndex = 0;
            mesh.startIndex = 0;
            mesh.lastIndex = 6;
            mesh.vertexCount = 4;
            
            mesh.instanceCount = 300;
            mesh.instanceOffset = 300 * i;
            mesh.color = simd_make_float4(0.0, 1.0, 1.0, 1.0);

        }
        else if (i < 107)
        {
            mesh.instanceCount = 1;
            mesh.instanceOffset = 30000 + i;
            mesh.color = simd_make_float4(0.0, 0.0, 0.0, 1.0);

        }
        else if (i < 122)
        {
            mesh.instanceCount = 3;
            mesh.instanceOffset = 30007 + (i - 107) * 3;
            mesh.color = simd_make_float4(0.0, 1.0, 0.0, 1.0);

        }
        else if (i < 137)
        {
            mesh.instanceCount = 30;
            mesh.instanceOffset = 30052 + (i - 122) * 30;
            mesh.color = simd_make_float4(1.0, 0.0, 1.0, 1.0);

        }
        else if (i < 138)
        {
            mesh.instanceCount = 300;
            mesh.instanceOffset = 30502;
            mesh.color = simd_make_float4(1.0, 0.0, 0.0, 1.0);

        }
        else if (i < 139)
        {
            mesh.instanceCount = 3000;
            mesh.instanceOffset = 30802;
            mesh.color = simd_make_float4(1.0, 1.0, 1.0, 1.0);

        }
        else if (i < 140)
        {
            mesh.instanceCount = 10000;
            mesh.instanceOffset = 33802;
            mesh.color = simd_make_float4(1.0, 1.0, 0.0, 1.0);

        }
            
        // Set the number of triangles.
        mesh.primitiveCount = (mesh.lastIndex - mesh.startIndex) / 3;

    }
    
    // Tell Metal when the buffer contents change.
    assert(_pMeshVerticesBuffer->length() >= meshVertices.size() * sizeof(AAPLVertex));
    assert(_pMeshIndicesBuffer->length() >= meshIndices.size() * sizeof(AAPLIndexType));
    memcpy(_pMeshVerticesBuffer->contents(), meshVertices.data(), sizeof(AAPLVertex) * meshVertices.size());
    memcpy(_pMeshIndicesBuffer->contents(), meshIndices.data(), sizeof(AAPLIndexType) * meshIndices.size());
}

void AAPLRenderer::prepareInstanceData()
{
    std::vector<AAPLInstanceData> instance_data(instanceCount);

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
        instance_data[i].instancePos      = simd_make_float3(rho * cos(theta), uniform_dist(rnd_generator) * 0.5f - 0.25f, rho * sin(theta));
        instance_data[i].instanceRot      = simd_make_float3(0, 0, 0);
        instance_data[i].instanceScale    = 1.5f + uniform_dist(rnd_generator) - uniform_dist(rnd_generator);
        instance_data[i].instanceIndex = i;
        instance_data[i].instanceScale *= 0.75f;

        // Outer ring
        rho                                                                 = sqrt((pow(ring1[1], 2.0f) - pow(ring1[0], 2.0f)) * uniform_dist(rnd_generator) + pow(ring1[0], 2.0f));
        theta                                                               = 2.0f * 3.14 * uniform_dist(rnd_generator);
        instance_data[static_cast<size_t>(i + instanceCount / 2)].instancePos      = simd_make_float3(rho * cos(theta), uniform_dist(rnd_generator) * 0.5f - 0.25f, rho * sin(theta));
        instance_data[static_cast<size_t>(i + instanceCount / 2)].instanceRot      =  simd_make_float3(0, 0, 0);
        instance_data[static_cast<size_t>(i + instanceCount / 2)].instanceScale    = 1.5f + uniform_dist(rnd_generator) - uniform_dist(rnd_generator);
        instance_data[static_cast<size_t>(i + instanceCount / 2)].instanceIndex = static_cast<size_t>(i + instanceCount / 2);
        instance_data[static_cast<size_t>(i + instanceCount / 2)].instanceScale *= 0.75f;
    }
    memcpy(_pInstanceDataBuffer->contents(), instance_data.data(), sizeof(AAPLInstanceData) * instanceCount);
}

//- (void)prepareIndirectCmdBuffer
void AAPLRenderer::prepareIndirectCmdBuffer()
{
    MTLIndirectCommandBufferDescriptor* icbDescriptor = [MTLIndirectCommandBufferDescriptor new];

    // Indicate that the only draw commands will be indexed draw commands.
    icbDescriptor.commandTypes = MTLIndirectCommandTypeDraw;

    // Indicate that buffers will be set for each command IN the indirect command buffer.
    icbDescriptor.inheritBuffers = NO;

    // Indicate that a max of 3 buffers will be set for each command.
    icbDescriptor.maxVertexBufferBindCount = 6;
    icbDescriptor.maxFragmentBufferBindCount = 0;

#if defined TARGET_MACOS || defined(__IPHONE_13_0)
    // Indicate that the render pipeline state object will be set in the render command encoder
    // (not by the indirect command buffer).
    // On iOS, this property only exists on iOS 13 and later.  It defaults to YES in earlier
    // versions
    if (@available(iOS 13.0, *)) {
        icbDescriptor.inheritPipelineState = YES;
    }
#endif

    id<MTLIndirectCommandBuffer> _indirectCommandBuffer;

    _indirectCommandBuffer = [(__bridge id<MTLDevice>)_pDevice newIndirectCommandBufferWithDescriptor:icbDescriptor
                                                             maxCommandCount:1
                                                                     options:0];

    _indirectCommandBuffer.label = @"Scene ICB";

    //  Encode a draw command for each object drawn in the indirect command buffer.
    for (int objIndex = 0; objIndex < 1; objIndex++)
    {
        id<MTLIndirectRenderCommand> ICBCommand =
            [_indirectCommandBuffer indirectRenderCommandAtIndex:objIndex];

        [ICBCommand setVertexBuffer:(__bridge id<MTLBuffer>)_pMeshVerticesBuffer
                             offset:0
                            atIndex:AAPLBufferIndexMeshVertices];
        [ICBCommand setVertexBuffer:(__bridge id<MTLBuffer>)_pInstanceDataBuffer
                             offset:0
                            atIndex:AAPLBufferInstanceData];
        [ICBCommand setVertexBuffer:(__bridge id<MTLBuffer>)_pTransformsBuffer[_curFrameInFlight]
                             offset:0
                            atIndex:AAPLBufferIndexTransforms];
        [ICBCommand setVertexBuffer:(__bridge id<MTLBuffer>)_pViewProjectionBuffer
                             offset:0
                            atIndex:AAPLBufferViewProjectionMatrix];
        
//        [ICBCommand drawIndexedPrimitives:MTLPrimitiveTypeTriangle
//                               indexCount:6
//                                indexType:MTLIndexTypeUInt16
//                              indexBuffer:(__bridge id<MTLBuffer>)_pMeshIndicesBuffer
//                        indexBufferOffset:0
//                            instanceCount:1
//                               baseVertex:0
//                             baseInstance:0];
        
        [ICBCommand drawPrimitives:MTLPrimitiveTypeTriangle
                       vertexStart:0
                       vertexCount:4
                     instanceCount:300
                      baseInstance:objIndex];

    }
    
    _pIndirectCommandBuffer = (__bridge_retained MTL::IndirectCommandBuffer*)_indirectCommandBuffer;
}

/// Updates the object transform matrix state before other methods encode any render commands.
void AAPLRenderer::updateStage()
{
    // Get the array pointers for the buffers.
    matrix_float4x4* transforms = reinterpret_cast<matrix_float4x4*>(_pTransformsBuffer[_curFrameInFlight]->contents());

    int count = 0;
    degree += rotationSpeed * M_PI / 180.0f;

    for (size_t z = 0; z < 1; ++z)
    {
        float z_pos = -12.0f - z * 2.0f;
        for (size_t y = 0; y < 1; ++y)
        {
            float y_pos = 2 * (y - (float(1 - 1) / 2));
            for (size_t x = 0; x < 2; ++x)
            {
                float x_pos = 2 * (x - (float(2 - 1) / 2));
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

    //matrix_float4x4 viewMatrix = matrix_multiply(matrix4x4_XRotate(0.9), matrix4x4_translation(0, 4, 9));
    matrix_float4x4 viewMatrix = matrix_multiply(matrix4x4_XRotate(1.5), matrix4x4_translation(0, 4, 12));

    std::vector<matrix_float4x4> viewProjectionMatrix;
    viewProjectionMatrix.push_back(matrix_multiply(_projectionMatrix, viewMatrix));

    memcpy(_pViewProjectionBuffer->contents(), viewProjectionMatrix.data(), sizeof(matrix_float4x4));


    // Update the object positions.
    updateStage();


    // Indirect Draw
    if (methodChoice == 1)
    {
        pRenderEncoder->setFrontFacingWinding(MTL::Winding::WindingCounterClockwise);
        pRenderEncoder->setRenderPipelineState(_pRenderPipelineStateIndirectDraw);
        pRenderEncoder->setDepthStencilState(_pDepthStencilState);
        
        pRenderEncoder->setVertexBuffer(_pMeshVerticesBuffer, 0, AAPLBufferIndexMeshVertices);
        pRenderEncoder->setVertexBuffer(_pInstanceDataBuffer, 0, AAPLBufferInstanceData);
        pRenderEncoder->setVertexBuffer(_pTransformsBuffer[_curFrameInFlight], 0, AAPLBufferIndexTransforms);
        //pRenderEncoder->setVertexBytes(&viewProjectionMatrix, sizeof(viewProjectionMatrix), AAPLBufferViewProjectionMatrix);
        pRenderEncoder->setVertexBuffer(_pViewProjectionBuffer, 0, AAPLBufferViewProjectionMatrix);

        pRenderEncoder->useResource(_pMeshVerticesBuffer, MTLResourceUsageRead);
        pRenderEncoder->useResource(_pInstanceDataBuffer, MTLResourceUsageRead);
        pRenderEncoder->useResource(_pTransformsBuffer[_curFrameInFlight], MTLResourceUsageRead);
        pRenderEncoder->useResource(_pViewProjectionBuffer, MTLResourceUsageRead);
        
        pRenderEncoder->executeCommandsInBuffer(_pIndirectCommandBuffer, {0, 1});
    }
    
    // Mesh Shader
    if (methodChoice == 2)
    {
        pRenderEncoder->setFrontFacingWinding(MTL::Winding::WindingCounterClockwise);
        pRenderEncoder->setRenderPipelineState(_pRenderPipelineStateMeshShader);
        pRenderEncoder->setDepthStencilState(_pDepthStencilState);
        
        // Pass data to the object stage.
        pRenderEncoder->setObjectBuffer(_pMeshVerticesBuffer, 0, AAPLBufferIndexMeshVertices);
        pRenderEncoder->setObjectBuffer(_pMeshIndicesBuffer, 0, AAPLBufferIndexMeshIndices);
        
        pRenderEncoder->setObjectBuffer(_pTransformsBuffer[_curFrameInFlight], 0, AAPLBufferIndexTransforms);
        pRenderEncoder->setObjectBuffer(_pViewProjectionBuffer, 0, AAPLBufferViewProjectionMatrix);

        //pRenderEncoder->setObjectBytes(&viewProjectionMatrix, sizeof(viewProjectionMatrix), AAPLBufferViewProjectionMatrix);

        pRenderEncoder->setMeshBuffer(_pInstanceDataBuffer, 0, AAPLBufferInstanceData);

        // Pass data to the mesh stage.
        //pRenderEncoder->setMeshBytes(&viewProjectionMatrix, sizeof(viewProjectionMatrix), AAPLBufferViewProjectionMatrix);

        //pRenderEncoder->setFragmentTextures(texArray, TextureIndexBaseColor);
        pRenderEncoder->setFragmentTexture(texture, TextureIndexBaseColor);
        
    //    // 第一批处理前128个纹理
    //    pRenderEncoder->setFragmentTextures(texArray, {0, 128});
    //
    //    // 第二批处理接下来的128个纹理
    //    pRenderEncoder->setFragmentTextures(texArray + 128, {128, 128});
    //
    //    // 处理剩余的44个纹理
    //    pRenderEncoder->setFragmentTextures(texArray + 256, {256, 44});
        //pRenderEncoder->setFragmentTextures(texArray, NS::Range(0, 300));

        
        /// Draw objects using the mesh shaders.
        /// Parameter 1: threadgroupsPerGrid ... X=`AAPLNumObjectsX`, Y=`AAPLNumObjectsY`, ...
        /// Parameter 2: threadsPerObjectThreadgroup ... `AAPLMaxTotalThreadsPerObjectThreadgroup`
        /// Parameter 3: threadsPerMeshGroup ... X = `AAPLMaxTotalThreadsPerMeshThreadgroup` has a limit of 64 vertices per meshlet in this sample.
        ///
        /// The object shader copies vertices, indices, and other relevant data to the payload and generates the submesh groups.
        /// The parameter `positionInGrid` (`threadgroup_position_in_grid`) in the shader addresses the submesh.
        /// This tells the object shader the index of the transform for the submesh.
        /// The mesh shader uses the payload to generate the primitives (points, lines, or triangles).
        for (int i = 0; i < AAPLNumTasks; i++)
        {
            pRenderEncoder->setObjectBytes(&meshInfo[i], sizeof(AAPLMeshInfo), AAPLBufferIndexMeshInfo);
            pRenderEncoder->drawMeshThreadgroups(MTL::Size(meshInfo[i].instanceCount, 1, 1),
                                                 MTL::Size(1, 1, 1),
                                                 MTL::Size(126, 1, 1));
        }
    }
    

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
