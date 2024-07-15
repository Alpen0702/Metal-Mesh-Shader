#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tiny_gltf.h"
#include "AAPLShaderTypes.h"

#include <cstdio>
#include <fstream>
#include <iostream>

static std::string GetFilePathExtension(const std::string& FileName) {
	if (FileName.find_last_of(".") != std::string::npos)
		return FileName.substr(FileName.find_last_of(".") + 1);
	return "";
}


void loadGLTF(const char* path, std::vector<AAPLVertex>& vertices, std::vector<AAPLIndexType>& indices)
{
	tinygltf::Model model;
	tinygltf::TinyGLTF gltf_ctx;
	std::string err;
	std::string warn;
	std::string ext = GetFilePathExtension(path);

	//gltf_ctx.SetStoreOriginalJSONForExtrasAndExtensions(
	//	store_original_json_for_extras_and_extensions);

	bool ret = false;
	if (ext.compare("glb") == 0) {
		std::cout << "Reading binary glTF" << std::endl;
		// assume binary glTF.
		ret = gltf_ctx.LoadBinaryFromFile(&model, &err, &warn, path);
	}
	else {
		std::cout << "Reading ASCII glTF" << std::endl;
		// assume ascii glTF.
		ret = gltf_ctx.LoadASCIIFromFile(&model, &err, &warn, path);
	}
    
    
    if (path) {
        std::cout << "File exists." << std::endl;
    } else {
        std::cout << "File does not exist." << std::endl;
    }
    
	for (const auto& mesh : model.meshes) {
		for (const auto& primitive : mesh.primitives) {
			const tinygltf::Accessor& positionAccessor = model.accessors[primitive.attributes.find("POSITION")->second];
			const tinygltf::BufferView& positionBufferView = model.bufferViews[positionAccessor.bufferView];
			const tinygltf::Buffer& positionBuffer = model.buffers[positionBufferView.buffer];

			const float* positions = reinterpret_cast<const float*>(&positionBuffer.data[positionBufferView.byteOffset + positionAccessor.byteOffset]);

			// 获取顶点法线
			const tinygltf::Accessor& normalAccessor = model.accessors[primitive.attributes.find("NORMAL")->second];
			const tinygltf::BufferView& normalBufferView = model.bufferViews[normalAccessor.bufferView];
			const tinygltf::Buffer& normalBuffer = model.buffers[normalBufferView.buffer];

			const float* normals = reinterpret_cast<const float*>(&normalBuffer.data[normalBufferView.byteOffset + normalAccessor.byteOffset]);

			// 获取纹理坐标
			const tinygltf::Accessor& texcoordAccessor = model.accessors[primitive.attributes.find("TEXCOORD_0")->second];
			const tinygltf::BufferView& texcoordBufferView = model.bufferViews[texcoordAccessor.bufferView];
			const tinygltf::Buffer& texcoordBuffer = model.buffers[texcoordBufferView.buffer];

			const float* texcoords = reinterpret_cast<const float*>(&texcoordBuffer.data[texcoordBufferView.byteOffset + texcoordAccessor.byteOffset]);

			// 读取顶点数据
			for (size_t i = 0; i < positionAccessor.count; ++i) {
				float px = positions[i * 3 + 0]; // x 坐标
				float py = positions[i * 3 + 1]; // y 坐标
				float pz = positions[i * 3 + 2]; // z 坐标

				float nx = normals[i * 3 + 0]; // 法线 x
				float ny = normals[i * 3 + 1]; // 法线 y
				float nz = normals[i * 3 + 2]; // 法线 z

				float tx = texcoords[i * 2 + 0]; // 纹理坐标 u
				float ty = texcoords[i * 2 + 1]; // 纹理坐标 v
				
				AAPLVertex vtx;
				
				vtx.position = simd_make_float4(px, py, pz, 1);
				vtx.normal = simd_make_float4(nx, ny, nz, 0);
				vtx.uv = simd_make_float2(tx, ty);

				vertices.push_back(vtx);
			}

			if (primitive.indices >= 0) {
				const tinygltf::Accessor& indexAccessor = model.accessors[primitive.indices];
				const tinygltf::BufferView& indexBufferView = model.bufferViews[indexAccessor.bufferView];
				const tinygltf::Buffer& indexBuffer = model.buffers[indexBufferView.buffer];

				const void* _indices = &indexBuffer.data[indexBufferView.byteOffset + indexAccessor.byteOffset];
				size_t count = indexAccessor.count;

				std::vector<unsigned int> indexData;
				if (indexAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) {
					const unsigned short* ushort_indices = reinterpret_cast<const unsigned short*>(_indices);
					indexData.assign(ushort_indices, ushort_indices + count);
				}
				else if (indexAccessor.componentType ==	TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) {
					const unsigned int* uint_indices = reinterpret_cast<const unsigned int*>(_indices);
					indexData.assign(uint_indices, uint_indices + count);
				}

				// 处理索引数据
				for (size_t i = 0; i < indexData.size(); i += 3) {
					indices.push_back(indexData[i]);
					indices.push_back(indexData[i + 1]);
					indices.push_back(indexData[i + 2]);
				}
			}
		}
	}
}
