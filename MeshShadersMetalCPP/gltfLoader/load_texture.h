#import <Metal/Metal.h>
#import <Foundation/Foundation.h>

@interface TextureLoader : NSObject
- (id<MTLTexture>)loadTextureUsingMetalKit: (NSURL *)url device: (id<MTLDevice>)device;
@end
