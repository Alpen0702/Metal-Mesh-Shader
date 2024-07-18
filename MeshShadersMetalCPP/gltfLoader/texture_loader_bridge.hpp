#include <Metal/Metal.hpp>

void* createTextureLoader();
MTL::Texture* loadTexture(void* loader, const char* url, MTL::Device* device);
