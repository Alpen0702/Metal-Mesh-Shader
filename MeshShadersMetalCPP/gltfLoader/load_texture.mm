#import <MetalKit/MetalKit.h>
#include "load_texture.h"

@implementation TextureLoader

- (id<MTLTexture>)loadTextureUsingMetalKit: (NSURL *)url device: (id<MTLDevice>)device {
    
    NSError *error;
    MTKTextureLoader *loader = [[MTKTextureLoader alloc] initWithDevice:device];
    NSDictionary *textureLoaderOptions = @{
        MTKTextureLoaderOptionTextureUsage : @(MTLTextureUsageShaderRead),
        MTKTextureLoaderOptionTextureStorageMode : @(MTLStorageModePrivate)
    };
    id<MTLTexture> texture = [loader newTextureWithContentsOfURL:url options:textureLoaderOptions error:&error];

    if (!texture) {
        NSLog(@"Error creating the texture from %@: %@", url.absoluteString, error.localizedDescription);
        return nil;
    }
    else
        NSLog(@"Texture loaded successfully.");

    return texture;
}

@end

