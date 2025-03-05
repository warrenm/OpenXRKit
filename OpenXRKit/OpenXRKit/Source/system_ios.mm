/*
** Copyright 2024-2025, Warren Moore
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

#import "runtime_apple.hpp"

#if defined (OS_APPLE_IOS)

#include <vector>

#include <sys/utsname.h>

#import <ARKit/ARKit.h>
#import <Metal/Metal.h>

namespace {
const char *GetSystemModelName(void) {
    static char modelName[256];
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        utsname systemInfo;
        uname(&systemInfo);
        strcpy(modelName, systemInfo.machine);
    });
    return modelName;
}
}

class System_iOS final : public xr::System {
public:
    System_iOS(XrSystemId systemId) : xr::System(systemId)
    {
        // Since instances can query views before creating a session, we can't rely
        // on there being an existing set of graphics bindings during such queries,
        // so we just assume the application will wind up using the default Metal
        // device. This is a pretty safe assumption nowadays, since no iOS, iPadOS,
        // or visionOS device reports more than one Metal device. Back in the short-
        // lived era of VR on macOS, this would have been more troubling, but those
        // days are long gone.
        _device = MTLCreateSystemDefaultDevice();
    }

    XrResult GetSystemProperties(XrSystemProperties *properties) override {
        // All iOS devices supporting our minimum required GPU family (Apple4)
        // have a maximum 2D texture dimension of 16K.
        // Ref. https://developer.apple.com/metal/Metal-Feature-Set-Tables.pdf
        XrSystemGraphicsProperties graphicsProperties {
            .maxSwapchainImageHeight = 16384,
            .maxSwapchainImageWidth = 16384,
            .maxLayerCount = XR_MIN_COMPOSITION_LAYERS_SUPPORTED,
        };

        XrBool32 has3DoF = [AROrientationTrackingConfiguration isSupported];
        XrBool32 has6DoF = [ARWorldTrackingConfiguration isSupported];

        XrSystemTrackingProperties trackingProperties {
            .orientationTracking = has3DoF,
            .positionTracking = has3DoF && has6DoF,
        };

        properties->type = XR_TYPE_SYSTEM_PROPERTIES;
        properties->next = nullptr;
        properties->systemId = GetSystemId();
        properties->vendorId = 0x05ac; // https://devicehunt.com/view/type/usb/vendor/05AC
        // Apple doesn't provide API to get the marketing name of a device, so we just use the raw machine name.
        strcpy(properties->systemName, GetSystemModelName());
        memcpy(&properties->graphicsProperties, &graphicsProperties, sizeof(XrSystemGraphicsProperties));
        memcpy(&properties->trackingProperties, &trackingProperties, sizeof(XrSystemTrackingProperties));

        XrBaseOutStructure *next = (XrBaseOutStructure *)properties->next;
        while (next) {
            if (next->type == XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT) {
                auto handTracking = (XrSystemHandTrackingPropertiesEXT *)next;
                handTracking->supportsHandTracking = SupportsHandTracking();
            }
            next = next->next;
        }

        return XR_SUCCESS;
    }

    bool SupportsFormFactor(XrFormFactor formFactor) override {
        return formFactor == XrFormFactor::XR_FORM_FACTOR_HANDHELD_DISPLAY;
    }

    XrResult GetViewConfigurations(std::vector<XrViewConfigurationType> &configurations) override {
        // We only support "mobile AR" configurations, not Google Cardboard-style VR.
        configurations = { XrViewConfigurationType::XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO };
        return XR_SUCCESS;
    }

    XrResult GetPropertiesForViewConfiguration(XrViewConfigurationType configuration,
                                               XrViewConfigurationProperties *properties) override
    {
        if (configuration != XrViewConfigurationType::XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO) {
            return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
        }

        properties->type = XR_TYPE_VIEW_CONFIGURATION_PROPERTIES;
        properties->next = nullptr;
        properties->viewConfigurationType = configuration;
        properties->fovMutable = XR_TRUE;

        return XR_SUCCESS;
    }

    XrResult GetViewsForViewConfiguration(XrViewConfigurationType configuration,
                                          std::vector<XrViewConfigurationView> &views) override
    {
        assert(_device != nil);

        if (configuration != XrViewConfigurationType::XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO) {
            return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
        }

        views.clear();
        views.emplace_back(XrViewConfigurationView{});

        XrSystemProperties system;
        GetSystemProperties(&system);

        NSUInteger maxSampleCount = 16;
        while (![_device supportsTextureSampleCount:maxSampleCount] && maxSampleCount > 1) {
            // Current Metal devices only have power-of-two sample counts, so do a log scan rather than linear.
            maxSampleCount >>= 1;
        }

        UIScreen *mainScreen = UIScreen.mainScreen;
        // We use bounds and not nativeBounds for a few reasons. Primarily,
        // bounds changes based on device orientation, and if we happen never
        // to change orientations, swap chain resources might not have to be
        // discarded. Furthermore, although native bounds is handily expressed
        // in pixels, it also represents the physical pixel size of the display
        // rather than the resolution at which content is drawn (prior to being
        // downscaled on some devices). By drawing at the expected resolution of
        // the system, we hope to play a little nicer with the compositor.
        // In any event, all of this is probably irrelevant because the end user
        // should create whatever render target size they want and let us and the
        // system handle any necessary scaling.
        CGSize mainScreenSizePoints = mainScreen.bounds.size;
        CGFloat displayWidth = mainScreenSizePoints.width * mainScreen.scale;
        CGFloat displayHeight = mainScreenSizePoints.height * mainScreen.scale;

        XrViewConfigurationView &view = views.back();
        view.type = XR_TYPE_VIEW_CONFIGURATION_VIEW;
        view.next = nullptr;
        view.recommendedImageRectWidth = static_cast<uint32_t>(displayWidth);
        view.maxImageRectWidth = system.graphicsProperties.maxSwapchainImageWidth;
        view.recommendedImageRectHeight = static_cast<uint32_t>(displayHeight);
        view.maxImageRectHeight = system.graphicsProperties.maxSwapchainImageHeight;
        view.recommendedSwapchainSampleCount = 1;
        view.maxSwapchainSampleCount = static_cast<uint32_t>(maxSampleCount);

        return XR_SUCCESS;
    }

    XrResult GetBlendModesForViewConfiguration(XrViewConfigurationType configuration,
                                               std::vector<XrEnvironmentBlendMode> &blendModes) override
    {
        if (configuration != XrViewConfigurationType::XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO) {
            return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
        }

        blendModes = {
            XrEnvironmentBlendMode::XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND,
            XrEnvironmentBlendMode::XR_ENVIRONMENT_BLEND_MODE_OPAQUE,
            XrEnvironmentBlendMode::XR_ENVIRONMENT_BLEND_MODE_ADDITIVE,
        };
        return XR_SUCCESS;
    }

    XrResult GetMetalGraphicsRequirementsKHR(XrGraphicsRequirementsMetalKHR *graphicsRequirements) override {
        #if defined(OS_APPLE_MACOS)
        graphicsRequirements->requiredMetalGPUFamily = static_cast<int64_t>(MTLGPUFamilyMetal3);
        #elif defined(OS_APPLE_IOS) // iOS and iPadOS
        graphicsRequirements->requiredMetalGPUFamily = static_cast<int64_t>(MTLGPUFamilyApple4); // A11 or newer
        #elif defined(OS_APPLE_VISIONOS)
        graphicsRequirements->requiredMetalGPUFamily = static_cast<int64_t>(MTLGPUFamilyApple8);
        #else
        graphicsRequirements->requiredMetalGPUFamily = 0; // Unsupported platform (tvOS, watchOS, etc.)
        #endif

        return XR_SUCCESS;
    }

    /* When recentering occurs, the runtime must queue the XrEventDataReferenceSpaceChangePending event,
     with the recentered LOCAL space origin only taking effect for xrLocateSpace or xrLocateViews calls
     whose XrTime parameter is greater than or equal to the changeTime provided in that event.
     */

private:
    id<MTLDevice> _device;
};

std::unique_ptr<xr::System> CreateSystem_iOS(XrSystemId systemId) {
    return std::make_unique<System_iOS>(systemId);
}

#endif
