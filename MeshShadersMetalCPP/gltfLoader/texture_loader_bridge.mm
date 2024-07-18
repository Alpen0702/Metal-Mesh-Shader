#import "load_texture.h"
#import "texture_loader_bridge.hpp"

void* createTextureLoader() {
    TextureLoader* loader = [[TextureLoader alloc] init];
    return (__bridge_retained void*)loader;
}

MTL::Texture* loadTexture(void* loader, const char* url, MTL::Device* device) {
    TextureLoader* bridge = (__bridge TextureLoader*)loader;
    NSURL* nsUrl = [[NSBundle mainBundle] URLForResource:@(url) withExtension:@"ktx"];
    id<MTLDevice> metalDevice = (__bridge id<MTLDevice>)device;
    id<MTLTexture> texture = [bridge loadTextureUsingMetalKit:nsUrl device:metalDevice];
    return (__bridge_retained MTL::Texture*)texture;
}
