#import <Foundation/Foundation.h>

FOUNDATION_EXPORT double OpenXRKitVersionNumber;

FOUNDATION_EXPORT const unsigned char OpenXRKitVersionString[];

/* 
    This umbrella header does not contain any imports, because all public symbols for OpenXR are declared
    in the OpenXR headers, which should be included in your target via an OpenXR SDK. The runtime provided
    by OpenXRKit can be accessed either by statically linking the OpenXR loader (as an XCFramework or static
    library) and configuring it to dynamically load the runtime via an environment variable and a JSON runtime
    manifest file that points to a copy of the runtime embedded in your app bundle, or by linking directly to
    OpenXRKit, which also defines the public symbols declared by the OpenXR SDK.

    The reason for this unusual linking requirement is that iOS and visionOS apps cannot install libraries
    or other files into system directories, so all OpenXR runtime-related content must be in the app bundle.
    Furthermore, passing non-literal arguments to dlopen() is strongly discouraged by App Store policy, so
    linking directly against OpenXRKit (bypassing the loader but also forgoing any opportunity to activate
    layers) is the preferred way, unless you are hacking on macOS or building apps for your own use outside
    the App Store, where these restriction do not apply.

    Note that the SDK headers provided with this project declare certain symbols that belong to extensions not
    yet ratified by the Khronos Group, so if you are already using another OpenXR SDK, you will either need to
    use the provided headers to build and link your target, or else patch your own SDK to include these symbols.
*/
