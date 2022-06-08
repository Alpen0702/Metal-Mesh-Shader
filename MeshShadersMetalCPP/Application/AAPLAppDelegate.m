/*
See LICENSE folder for this sample’s licensing information.

Abstract:
This file contains the app delegate for a Cocoa app.
*/

#import "AAPLAppDelegate.h"

@implementation AAPLAppDelegate

/// Tell Cocoa that the app needs to terminate when the window closes.
- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication *)sender
{
    return YES;
}

@end
