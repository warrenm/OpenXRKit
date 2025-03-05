/*
** Copyright 2024-2025, Warren Moore
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

#import "runtime_apple.hpp"

#if defined (OS_APPLE_VISIONOS)

#include <vector>

#include <sys/utsname.h>

#import <ARKit/ARKit.h>
#import <Metal/Metal.h>

using namespace xr;

namespace {
const char *GetSystemModelName(void) {
    static char modelName[] = "Apple Vision Pro";
    return modelName;
}

template <typename Type, XrStructureType Tag>
Type *_Nullable FindInNextChain(void *root) {
    XrBaseOutStructure *base = (XrBaseOutStructure *)root;
    while (base) {
        if (base->type == Tag) {
            return reinterpret_cast<Type *>(base);
        }
        base = (XrBaseOutStructure *)base->next;
    }
    return nullptr;
}
}

class System_visionOS final : public System {
public:
    System_visionOS(XrSystemId systemId) : System(systemId)
    {
        // Since instances can query views before creating a session, we can't rely
        // on there being an existing set of graphics bindings during such queries,
        // so we just assume the application will wind up using the default Metal
        // device.
        _device = MTLCreateSystemDefaultDevice();
    }

    XrResult GetSystemProperties(XrSystemProperties *properties) override {
        if (properties == nullptr || properties->type != XR_TYPE_SYSTEM_PROPERTIES) {
            return XR_ERROR_VALIDATION_FAILURE;
        }

        // All visionOS devices support the Apple8 GPU family, which has a
        // maximum 2D texture dimension of 16K.
        // Ref. https://developer.apple.com/metal/Metal-Feature-Set-Tables.pdf
        XrSystemGraphicsProperties graphicsProperties {
            .maxSwapchainImageHeight = 16384,
            .maxSwapchainImageWidth = 16384,
            .maxLayerCount = XR_MIN_COMPOSITION_LAYERS_SUPPORTED,
        };

        XrBool32 has3DoF = XR_TRUE;
        XrBool32 has6DoF = XR_TRUE;

        XrSystemTrackingProperties trackingProperties {
            .orientationTracking = has3DoF,
            .positionTracking = has3DoF && has6DoF,
        };

        properties->systemId = GetSystemId();
        properties->vendorId = 0x05ac; // https://devicehunt.com/view/type/usb/vendor/05AC
        // Apple doesn't provide API to get the marketing name of a device, so we just use the raw machine name.
        strcpy(properties->systemName, GetSystemModelName());
        memcpy(&properties->graphicsProperties, &graphicsProperties, sizeof(XrSystemGraphicsProperties));
        memcpy(&properties->trackingProperties, &trackingProperties, sizeof(XrSystemTrackingProperties));

        auto handTracking = FindInNextChain<XrSystemHandTrackingPropertiesEXT, XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT>(properties);
        if (handTracking) {
            handTracking->supportsHandTracking = ar_hand_tracking_provider_is_supported();
        }

        return XR_SUCCESS;
    }

    bool SupportsFormFactor(XrFormFactor formFactor) override {
        return formFactor == XrFormFactor::XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    }

    XrResult GetViewConfigurations(std::vector<XrViewConfigurationType> &configurations) override {
        #if TARGET_OS_SIMULATOR
        configurations = { XrViewConfigurationType::XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO };
        #else
        configurations = { XrViewConfigurationType::XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO };
        #endif
        return XR_SUCCESS;
    }

    XrResult GetPropertiesForViewConfiguration(XrViewConfigurationType configuration,
                                               XrViewConfigurationProperties *properties) override
    {
        #if TARGET_OS_SIMULATOR
        XrViewConfigurationType supportedConfiguration = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO;
        #else
        XrViewConfigurationType supportedConfiguration = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        #endif

        if (configuration != supportedConfiguration) {
            return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
        }

        properties->type = XR_TYPE_VIEW_CONFIGURATION_PROPERTIES;
        properties->next = nullptr;
        properties->viewConfigurationType = configuration;
        properties->fovMutable = XR_FALSE; // FoV is supplied by CompositorServices

        return XR_SUCCESS;
    }

    XrResult GetViewsForViewConfiguration(XrViewConfigurationType configuration,
                                          std::vector<XrViewConfigurationView> &views) override
    {
        assert(_device != nil);

        size_t viewCount = 1;
        switch (configuration) {
            case XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO:
                viewCount = 1;
                break;
            case XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO:
                viewCount = 2;
                break;
            default:
                return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
        }

        views.clear();

        XrSystemProperties system {
            .type = XR_TYPE_SYSTEM_PROPERTIES,
            .next = nullptr
        };
        GetSystemProperties(&system);

        NSUInteger maxSampleCount = 16;
        while (![_device supportsTextureSampleCount:maxSampleCount] && maxSampleCount > 1) {
            // Current Metal devices only have power-of-two sample counts, so do a log scan rather than linear.
            maxSampleCount >>= 1;
        }

        #if TARGET_OS_SIMULATOR
        const CGSize compositorServicesResolution = CGSizeMake(2732, 2048);
        #else
        const CGSize compositorServicesResolution = CGSizeMake(1920, 1824);
        #endif

        for (size_t i = 0; i < viewCount; ++i) {
            XrViewConfigurationView view;
            view.type = XR_TYPE_VIEW_CONFIGURATION_VIEW;
            view.next = nullptr;
            view.recommendedImageRectWidth = static_cast<uint32_t>(compositorServicesResolution.width);
            view.maxImageRectWidth = system.graphicsProperties.maxSwapchainImageWidth;
            view.recommendedImageRectHeight = static_cast<uint32_t>(compositorServicesResolution.height);
            view.maxImageRectHeight = system.graphicsProperties.maxSwapchainImageHeight;
            view.recommendedSwapchainSampleCount = 1;
            view.maxSwapchainSampleCount = static_cast<uint32_t>(maxSampleCount);
            views.push_back(view);
        }

        return XR_SUCCESS;
    }

    XrResult GetBlendModesForViewConfiguration(XrViewConfigurationType configuration,
                                               std::vector<XrEnvironmentBlendMode> &blendModes) override
    {
        if (configuration != XrViewConfigurationType::XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO) {
            return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
        }

        blendModes = {
            XrEnvironmentBlendMode::XR_ENVIRONMENT_BLEND_MODE_OPAQUE
        };

        if (@available(visionOS 2, *)) {
            // Metal-based passthrough became available with visionOS 2.0.
            blendModes.push_back(XrEnvironmentBlendMode::XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND);
        }
        return XR_SUCCESS;
    }

    XrResult GetMetalGraphicsRequirementsKHR(XrGraphicsRequirementsMetalKHR *graphicsRequirements) override {
        graphicsRequirements->requiredMetalGPUFamily = static_cast<int64_t>(MTLGPUFamilyApple8);
        return XR_SUCCESS;
    }

private:
    id<MTLDevice> _device;
};

std::unique_ptr<System> CreateSystem_visionOS(XrSystemId systemId) {
    return std::make_unique<System_visionOS>(systemId);
}

#endif
