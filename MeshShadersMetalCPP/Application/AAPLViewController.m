/*
See LICENSE folder for this sample’s licensing information.

Abstract:
This file provides a Metal view controller for a Cocoa app.
*/
#import "AAPLViewController.h"
#import "AAPLRendererAdapter.h"
#import <MetalKit/MetalKit.h>

@implementation AAPLViewController
{
    MTKView* _view;
    AAPLRendererAdapter* _renderer;
    __weak IBOutlet NSSegmentedControl* _lodDetailChoice;
    __weak IBOutlet NSSegmentedControl* _topologyChoice;
    __weak IBOutlet NSSegmentedControl* _rotationChoice;
    __weak IBOutlet NSSlider* _translationAmount;
    __weak IBOutlet NSTextField* _lodDetailChoiceLabel;
    __weak IBOutlet NSTextField* _topologyChoiceLabel;
    __weak IBOutlet NSTextField* _rotationChoiceLabel;
    __weak IBOutlet NSTextField* _translationAmountLabel;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    _view = (MTKView *)self.view;
    _view.device = MTLCreateSystemDefaultDevice();

    // If the device doesn't support mesh shaders, print a log message and don't initialize the renderer.
    if (!_view.device)
    {
        NSLog(@"Mesh shaders are not supported on this device");
    }
    else
    {
        _renderer = [[AAPLRendererAdapter alloc] initWithMtkView:_view];
        NSAssert(_renderer, @"Renderer failed initialization");
        
        [_renderer drawableSizeWillChange:_view.bounds.size];
        
        // Configure the view to use this class instance to handle the draw and resize events.
        _view.delegate = self;
    }
}

- (void)mtkView:(MTKView *)view drawableSizeWillChange:(CGSize)size
{
    [_renderer drawableSizeWillChange:_view.bounds.size];
}

- (void)drawInMTKView:(nonnull MTKView *)view
{
    const BOOL demoMode = NO;
    const BOOL invisibleControls = NO;
    float offsetY = invisibleControls ? 0.0f : -1.5f;
    
    if ((demoMode))
    {
        if (invisibleControls)
        {
            [_rotationChoice setHidden:YES]; [_rotationChoiceLabel setHidden:YES];
            [_topologyChoice setHidden:YES]; [_topologyChoiceLabel setHidden:YES];
            [_lodDetailChoice setHidden:YES]; [_lodDetailChoiceLabel setHidden:YES];
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
            [_lodDetailChoice setSelectedSegment:2]; // High
            [_topologyChoice setSelectedSegment:2]; // Triangles
        }
        else if (timeRunning >= 22) {
            [_lodDetailChoice setSelectedSegment:1]; // Medium
            [_topologyChoice setSelectedSegment:2]; // Triangles
        }
        else if (timeRunning >= 20) {
            [_lodDetailChoice setSelectedSegment:0]; // Low
            [_topologyChoice setSelectedSegment:2]; // Triangles
        }
        else if (timeRunning >= 18) {
            [_lodDetailChoice setSelectedSegment:0]; // Low
            [_topologyChoice setSelectedSegment:1]; // Lines
        }
        else if (timeRunning >= 16) {
            [_lodDetailChoice setSelectedSegment:1]; // Medium
            [_topologyChoice setSelectedSegment:1]; // Lines
        }
        else if (timeRunning >= 14) {
            [_lodDetailChoice setSelectedSegment:2]; // High
            [_topologyChoice setSelectedSegment:1]; // Lines
        }
        else if (timeRunning >= 12) {
            [_lodDetailChoice setSelectedSegment:2]; // High
            [_topologyChoice setSelectedSegment:0]; // Points
        }
        else if (timeRunning >= 10) {
            [_lodDetailChoice setSelectedSegment:1]; // Medium
            [_topologyChoice setSelectedSegment:0]; // Points
        }
        else if (timeRunning >= 8) {
            [_lodDetailChoice setSelectedSegment:0]; // Low
            [_topologyChoice setSelectedSegment:0]; // Points
        }
        else if (timeRunning >= 6) {
            [_lodDetailChoice setSelectedSegment:1]; // Medium
            [_topologyChoice setSelectedSegment:0]; // Points
        }
        else if (timeRunning >= 4) {
            [_lodDetailChoice setSelectedSegment:2]; // High
            [_topologyChoice setSelectedSegment:0]; // Points
        }
        else if (timeRunning >= 2) {
            [_lodDetailChoice setSelectedSegment:2]; // High
            [_topologyChoice setSelectedSegment:1]; // Lines
        }
        else {
            [_lodDetailChoice setSelectedSegment:2]; // High
            [_topologyChoice setSelectedSegment:2]; // Triangles
        }
    }
    
    switch([_rotationChoice selectedSegment]) {
        case -1:
        case 0: [_renderer setRotationSpeed:0.0f]; break; // Off
        case 1: [_renderer setRotationSpeed:0.1f]; break; // Slow
        case 2: [_renderer setRotationSpeed:0.25f]; break; // Normal
    }
    
    [_renderer setTranslation:[_translationAmount floatValue] offsetY:offsetY];
    
    switch ([_lodDetailChoice selectedSegment])
    {
        case -1:
        case 0: [_renderer setLODChoice:2]; break; // Low
        case 1: [_renderer setLODChoice:1]; break; // Medium
        case 2: [_renderer setLODChoice:0]; break; // High
    }
    
    switch([_topologyChoice selectedSegment])
    {
        case -1:
        case 0: [_renderer setTopologyChoice:0]; break; // Points
        case 1: [_renderer setTopologyChoice:1]; break; // Lines
        case 2: [_renderer setTopologyChoice:2]; break; // Triangles
    }
    
    [_renderer drawInMTKView:view];
}

@end
