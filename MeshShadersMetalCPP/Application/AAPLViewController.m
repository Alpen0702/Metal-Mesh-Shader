/*
See LICENSE folder for this sample’s licensing information.

Abstract:
The cross-platform view controller.
*/
#import "AAPLViewController.h"
#import "AAPLRendererAdapter.h"

@import MetalKit;

@implementation AAPLViewController
{
    MTKView* _view;
    AAPLRendererAdapter* _renderer;

#if TARGET_OS_OSX

    __weak IBOutlet NSSegmentedControl* _methodChoice;
    __weak IBOutlet NSSegmentedControl* _topologyChoice;
    __weak IBOutlet NSSegmentedControl* _rotationChoice;
    __weak IBOutlet NSSlider* _translationAmount;
    __weak IBOutlet NSTextField* _methodChoiceLabel;
    __weak IBOutlet NSTextField* _topologyChoiceLabel;
    __weak IBOutlet NSTextField* _rotationChoiceLabel;
    __weak IBOutlet NSTextField* _translationAmountLabel;

#elif TARGET_OS_IPHONE

    __weak IBOutlet UISegmentedControl* _methodChoice;
    __weak IBOutlet UISegmentedControl* _topologyChoice;
    __weak IBOutlet UISegmentedControl* _rotationChoice;
    __weak IBOutlet UISlider* _translationAmount;

#endif
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    _view = (MTKView *)self.view;
    _view.device = MTLCreateSystemDefaultDevice();
    NSAssert(_view.device, @"The app couldn't get a GPU device.");
    NSLog(@"Device: %@", _view.device.name);
    
    // Check whether the GPU device supports mesh shaders.
    if (![_view.device supportsFamily:MTLGPUFamilyMac2] &&
        ![_view.device supportsFamily:MTLGPUFamilyApple7])
    {
        NSLog(@"This GPU device doesn't support mesh shaders.");

        // Create an empty view and return.
#if TARGET_OS_OSX
        self.view = [[NSView alloc] initWithFrame:_view.frame];
#elif TARGET_OS_IPHONE
        self.view = [[UIView alloc] initWithFrame:_view.frame];
        self.view.backgroundColor = [[UIColor alloc] initWithRed:1.0 green:0.0 blue:0.0 alpha:1.0];
#endif
        return;
    }

    _renderer = [[AAPLRendererAdapter alloc] initWithMtkView:_view];
    NSAssert(_renderer, @"Renderer failed initialization");

    [_renderer drawableSizeWillChange:_view.bounds.size];

    // Configure the view to use this class instance to handle the draw and resize events.
    _view.delegate = self;
}

- (void)mtkView:(MTKView *)view drawableSizeWillChange:(CGSize)size
{
    [_renderer drawableSizeWillChange:_view.bounds.size];
}

- (void)drawInMTKView:(nonnull MTKView *)view
{
#if TARGET_OS_OSX
    const BOOL demoMode = NO;
    const BOOL invisibleControls = NO;
    float offsetY = invisibleControls ? 0.0f : -1.5f;
    
    if ((demoMode))
    {
        if (invisibleControls)
        {
            [_rotationChoice setHidden:YES]; [_rotationChoiceLabel setHidden:YES];
            [_topologyChoice setHidden:YES]; [_topologyChoiceLabel setHidden:YES];
            [_methodChoice setHidden:YES]; [_methodChoiceLabel setHidden:YES];
            [_translationAmount setHidden:YES]; [_translationAmountLabel setHidden:YES];
        }
        
        static double time0 = 0;
        if (time0 == 0)
            time0 = CACurrentMediaTime() + 10;
        double time1 = CACurrentMediaTime();
        double timeRunning = time1 - time0;
        
        [_rotationChoice setSelectedSegment:2];
        if (timeRunning < 20.0f)
            [_translationAmount setFloatValue:sin(timeRunning / 16.0f)];
        if (timeRunning >= 24) {
            [_methodChoice setSelectedSegment:2]; // High
            [_topologyChoice setSelectedSegment:2]; // Triangles
        }
        else if (timeRunning >= 22) {
            [_methodChoice setSelectedSegment:1]; // Medium
            [_topologyChoice setSelectedSegment:2]; // Triangles
        }
        else if (timeRunning >= 20) {
            [_methodChoice setSelectedSegment:0]; // Low
            [_topologyChoice setSelectedSegment:2]; // Triangles
        }
        else if (timeRunning >= 18) {
            [_methodChoice setSelectedSegment:0]; // Low
            [_topologyChoice setSelectedSegment:1]; // Lines
        }
        else if (timeRunning >= 16) {
            [_methodChoice setSelectedSegment:1]; // Medium
            [_topologyChoice setSelectedSegment:1]; // Lines
        }
        else if (timeRunning >= 14) {
            [_methodChoice setSelectedSegment:2]; // High
            [_topologyChoice setSelectedSegment:1]; // Lines
        }
        else if (timeRunning >= 12) {
            [_methodChoice setSelectedSegment:2]; // High
            [_topologyChoice setSelectedSegment:0]; // Points
        }
        else if (timeRunning >= 10) {
            [_methodChoice setSelectedSegment:1]; // Medium
            [_topologyChoice setSelectedSegment:0]; // Points
        }
        else if (timeRunning >= 8) {
            [_methodChoice setSelectedSegment:0]; // Low
            [_topologyChoice setSelectedSegment:0]; // Points
        }
        else if (timeRunning >= 6) {
            [_methodChoice setSelectedSegment:1]; // Medium
            [_topologyChoice setSelectedSegment:0]; // Points
        }
        else if (timeRunning >= 4) {
            [_methodChoice setSelectedSegment:2]; // High
            [_topologyChoice setSelectedSegment:0]; // Points
        }
        else if (timeRunning >= 2) {
            [_methodChoice setSelectedSegment:2]; // High
            [_topologyChoice setSelectedSegment:1]; // Lines
        }
        else {
            [_methodChoice setSelectedSegment:2]; // High
            [_topologyChoice setSelectedSegment:2]; // Triangles
        }
    }
#elif TARGET_OS_IPHONE
    float offsetY = 0.0f;
#endif

#if TARGET_OS_OSX
    NSInteger methodChoice = _methodChoice.selectedSegment;
    NSInteger topologyChoice = _topologyChoice.selectedSegment;
    NSInteger rotationChoice = _rotationChoice.selectedSegment;
    float translationAmount = _translationAmount.floatValue;
#elif TARGET_OS_IPHONE
    NSInteger methodChoice = _methodChoice.selectedSegmentIndex;
    NSInteger topologyChoice = _topologyChoice.selectedSegmentIndex;
    NSInteger rotationChoice = _rotationChoice.selectedSegmentIndex;
    float translationAmount = _translationAmount.value;
#endif

    [_renderer setTranslation:translationAmount offsetY:offsetY];

    switch (methodChoice)
    {
        case -1:
        case 0: [_renderer setMethodChoice:0]; break; // Low
        case 1: [_renderer setMethodChoice:1]; break; // Medium
        case 2: [_renderer setMethodChoice:2]; break; // Mesh Shader
    }
    
    switch(topologyChoice)
    {
        case -1:
        case 0: [_renderer setTopologyChoice:0]; break; // Points
        case 1: [_renderer setTopologyChoice:1]; break; // Lines
        case 2: [_renderer setTopologyChoice:2]; break; // Triangles
    }

    switch(rotationChoice) {
        case -1:
        case 0: [_renderer setRotationSpeed:0.0f]; break; // Off
        case 1: [_renderer setRotationSpeed:0.1f]; break; // Slow
        case 2: [_renderer setRotationSpeed:0.25f]; break; // Normal
    }

    [_renderer drawInMTKView:view];
}

@end
