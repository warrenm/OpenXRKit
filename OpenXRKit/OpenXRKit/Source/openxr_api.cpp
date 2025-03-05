/*
** Copyright 2024-2025, Warren Moore
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

#include "openxr_api.h"
#include "runtime_api.hpp"
#include "runtime_apple.hpp"

#include <cstring>
#include <string>
#include <unordered_map>

#define OPENXRKIT_RUNTIME_API_VERSION XR_MAKE_VERSION(1, 0, 34)

using namespace xr;

XrResult xrNegotiateLoaderRuntimeInterface(const XrNegotiateLoaderInfo *loaderInfo,
                                           XrNegotiateRuntimeRequest *runtimeRequest)
{
    if (loaderInfo->structType != XR_LOADER_INTERFACE_STRUCT_LOADER_INFO ||
        loaderInfo->structVersion != XR_LOADER_INFO_STRUCT_VERSION ||
        loaderInfo->structSize != sizeof(XrNegotiateLoaderInfo))
    {
        return XR_ERROR_INITIALIZATION_FAILED;
    }

    if (runtimeRequest->structType != XR_LOADER_INTERFACE_STRUCT_RUNTIME_REQUEST ||
        runtimeRequest->structVersion != XR_RUNTIME_INFO_STRUCT_VERSION ||
        runtimeRequest->structSize != sizeof(XrNegotiateRuntimeRequest))
    {
        return XR_ERROR_INITIALIZATION_FAILED;
    }

    bool supportedAPIVersion = (loaderInfo->minApiVersion <= OPENXRKIT_RUNTIME_API_VERSION) &&
    (loaderInfo->maxApiVersion >= OPENXRKIT_RUNTIME_API_VERSION);

    if (!supportedAPIVersion) {
        return XR_ERROR_INITIALIZATION_FAILED;
    }

    runtimeRequest->structType = XR_LOADER_INTERFACE_STRUCT_RUNTIME_REQUEST;
    runtimeRequest->structVersion = XR_LOADER_INFO_STRUCT_VERSION;
    runtimeRequest->structSize = sizeof(XrNegotiateRuntimeRequest);
    runtimeRequest->runtimeInterfaceVersion = XR_CURRENT_LOADER_API_LAYER_VERSION;
    runtimeRequest->runtimeApiVersion = OPENXRKIT_RUNTIME_API_VERSION;
    runtimeRequest->getInstanceProcAddr = xrGetInstanceProcAddr;

    return XR_SUCCESS;
}

XrResult xrEnumerateApiLayerProperties(uint32_t propertyCapacityInput,
                                       uint32_t *propertyCountOutput,
                                       XrApiLayerProperties *properties)
{
    *propertyCountOutput = 0;
    return XR_SUCCESS;
}

XrResult xrEnumerateInstanceExtensionProperties(const char *layerName, uint32_t propertyCapacityInput,
                                                uint32_t *propertyCountOutput, XrExtensionProperties *properties)
{
    int availableExtensionCount = 3;
    if (layerName == nullptr) {
        *propertyCountOutput = availableExtensionCount;
        if (propertyCapacityInput >= availableExtensionCount) {
            XrExtensionProperties &metalEnableProps = properties[0];
            metalEnableProps.type = XR_TYPE_EXTENSION_PROPERTIES;
            metalEnableProps.next = nullptr;
            strcpy(metalEnableProps.extensionName, XR_KHR_METAL_ENABLE_EXTENSION_NAME);
            metalEnableProps.extensionVersion = 1;

            XrExtensionProperties &depthLayerProps = properties[1];
            depthLayerProps.type = XR_TYPE_EXTENSION_PROPERTIES;
            depthLayerProps.next = nullptr;
            strcpy(depthLayerProps.extensionName, XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME);
            depthLayerProps.extensionVersion = 1;

            XrExtensionProperties &handTrackingProps = properties[2];
            handTrackingProps.type = XR_TYPE_EXTENSION_PROPERTIES;
            handTrackingProps.next = nullptr;
            strcpy(handTrackingProps.extensionName, XR_EXT_HAND_TRACKING_EXTENSION_NAME);
            handTrackingProps.extensionVersion = 1;

            return XR_SUCCESS;
        } else if (propertyCapacityInput > 0) {
            return XR_ERROR_SIZE_INSUFFICIENT;
        } else {
            return XR_SUCCESS;
        }
    } else {
        return XR_ERROR_API_LAYER_NOT_PRESENT;
    }
}

XrResult xrCreateInstance(const XrInstanceCreateInfo *createInfo, XrInstance *instance) {
    if (strlen(createInfo->applicationInfo.applicationName) == 0) {
        return XR_ERROR_NAME_INVALID;
    }

    *instance = xr::Instance::RegisterInstance(CreateInstance_Apple());
    return XR_SUCCESS;
}

XrResult xrDestroyInstance(XrInstance instance) {
    return xr::Instance::DestroyInstance(instance);
}

XrResult xrGetInstanceProperties(XrInstance instance, XrInstanceProperties *instanceProperties) {
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->GetInstanceProperties(instanceProperties);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrPollEvent(XrInstance instance, XrEventDataBuffer *eventData) {
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->PollEvent(eventData);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrResultToString(XrInstance instance, XrResult value, char buffer[XR_MAX_RESULT_STRING_SIZE]) {
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->ResultToString(value, buffer);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrStructureTypeToString(XrInstance instance, XrStructureType value, char buffer[XR_MAX_STRUCTURE_NAME_SIZE]) {
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->StructureTypeToString(value, buffer);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrGetSystem(XrInstance instance, const XrSystemGetInfo *getInfo, XrSystemId *systemId) {
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->GetSystem(getInfo, systemId);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrGetSystemProperties(XrInstance instance, XrSystemId systemId, XrSystemProperties *properties) {
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->GetSystemProperties(systemId, properties);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrEnumerateEnvironmentBlendModes(XrInstance instance, XrSystemId systemId, 
                                          XrViewConfigurationType viewConfigurationType,
                                          uint32_t environmentBlendModeCapacityInput, 
                                          uint32_t *environmentBlendModeCountOutput,
                                          XrEnvironmentBlendMode *environmentBlendModes)
{
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->EnumerateEnvironmentBlendModes(systemId, viewConfigurationType, environmentBlendModeCapacityInput,
                                                    environmentBlendModeCountOutput, environmentBlendModes);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrCreateSession(XrInstance instance, const XrSessionCreateInfo *createInfo, XrSession *session) {
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->CreateSession(createInfo, session);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrDestroySession(XrSession session) {
    if (auto impl = Session::FromHandle(session)) {
        XrInstance instanceHandle = impl->GetInstance();
        if (auto instance = Instance::FromHandle(instanceHandle)) {
            return instance->DestroySession(session);
        }
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrEnumerateReferenceSpaces(XrSession session, uint32_t spaceCapacityInput, 
                                    uint32_t *spaceCountOutput, XrReferenceSpaceType *spaces)
{
    if (auto impl = Session::FromHandle(session)) {
        return impl->EnumerateReferenceSpaces(spaceCapacityInput, spaceCountOutput, spaces);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrCreateReferenceSpace(XrSession session, const XrReferenceSpaceCreateInfo *createInfo, XrSpace *space) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->CreateReferenceSpace(createInfo, space);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrGetReferenceSpaceBoundsRect(XrSession session, XrReferenceSpaceType referenceSpaceType, XrExtent2Df *bounds) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->GetReferenceSpaceBoundsRect(referenceSpaceType, bounds);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrCreateActionSpace(XrSession session, const XrActionSpaceCreateInfo *createInfo, XrSpace *space) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->CreateActionSpace(createInfo, space);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrLocateSpace(XrSpace space, XrSpace baseSpace, XrTime time, XrSpaceLocation *location) {
    if (auto impl = Space::FromHandle(space)) {
        XrSession sessionHandle = impl->GetSession();
        auto session = xr::Session::FromHandle(sessionHandle);
        if (session == nullptr) {
            return XR_ERROR_HANDLE_INVALID;
        }
        return session->LocateSpace(space, baseSpace, time, location);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrDestroySpace(XrSpace space) {
    if (auto impl = Space::FromHandle(space)) {
        XrSession session = impl->GetSession();
        if (auto sessionImpl = Session::FromHandle(session)) {
            return sessionImpl->DestroySpace(space);
        }
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrEnumerateViewConfigurations(XrInstance instance, XrSystemId systemId, 
                                       uint32_t viewConfigurationTypeCapacityInput,
                                       uint32_t *viewConfigurationTypeCountOutput, 
                                       XrViewConfigurationType *viewConfigurationTypes)
{
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->EnumerateViewConfigurations(systemId, viewConfigurationTypeCapacityInput,
                                                 viewConfigurationTypeCountOutput, viewConfigurationTypes);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrGetViewConfigurationProperties(XrInstance instance, XrSystemId systemId,
                                          XrViewConfigurationType viewConfigurationType, 
                                          XrViewConfigurationProperties *configurationProperties)
{
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->GetViewConfigurationProperties(systemId, viewConfigurationType, configurationProperties);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrEnumerateViewConfigurationViews(XrInstance instance, XrSystemId systemId, 
                                           XrViewConfigurationType viewConfigurationType,
                                           uint32_t viewCapacityInput, uint32_t *viewCountOutput, 
                                           XrViewConfigurationView *views)
{
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->EnumerateViewConfigurationViews(systemId, viewConfigurationType, viewCapacityInput, viewCountOutput, views);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrEnumerateSwapchainFormats(XrSession session, uint32_t formatCapacityInput, 
                                     uint32_t *formatCountOutput, int64_t *formats)
{
    if (auto impl = Session::FromHandle(session)) {
        return impl->EnumerateSwapchainFormats(formatCapacityInput, formatCountOutput, formats);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrCreateSwapchain(XrSession session, const XrSwapchainCreateInfo *createInfo, XrSwapchain *swapchain) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->CreateSwapchain(createInfo, swapchain);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrDestroySwapchain(XrSwapchain swapchain) {
    if (auto impl = Swapchain::FromHandle(swapchain)) {
        XrSession session = impl->GetSession();
        if (auto sessionImpl = Session::FromHandle(session)) {
            return sessionImpl->DestroySwapchain(swapchain);
        }
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrEnumerateSwapchainImages(XrSwapchain swapchain, uint32_t imageCapacityInput,
                                    uint32_t *imageCountOutput, XrSwapchainImageBaseHeader *images)
{
    if (auto impl = Swapchain::FromHandle(swapchain)) {
        return impl->EnumerateSwapchainImages(imageCapacityInput, imageCountOutput, images);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrAcquireSwapchainImage(XrSwapchain swapchain, const XrSwapchainImageAcquireInfo *acquireInfo, uint32_t *index) {
    if (auto impl = Swapchain::FromHandle(swapchain)) {
        return impl->AcquireSwapchainImage(acquireInfo, index);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrWaitSwapchainImage(XrSwapchain swapchain, const XrSwapchainImageWaitInfo *waitInfo) {
    if (auto impl = Swapchain::FromHandle(swapchain)) {
        return impl->WaitSwapchainImage(waitInfo);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrReleaseSwapchainImage(XrSwapchain swapchain, const XrSwapchainImageReleaseInfo *releaseInfo) {
    if (auto impl = Swapchain::FromHandle(swapchain)) {
        return impl->ReleaseSwapchainImage(releaseInfo);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrBeginSession(XrSession session, const XrSessionBeginInfo *beginInfo) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->BeginSession(beginInfo);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrEndSession(XrSession session) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->EndSession();
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrRequestExitSession(XrSession session) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->RequestExitSession();
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrWaitFrame(XrSession session, const XrFrameWaitInfo *frameWaitInfo, XrFrameState *frameState) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->WaitFrame(frameWaitInfo, frameState);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrBeginFrame(XrSession session, const XrFrameBeginInfo *frameBeginInfo) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->BeginFrame(frameBeginInfo);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrEndFrame(XrSession session, const XrFrameEndInfo *frameEndInfo) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->EndFrame(frameEndInfo);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrLocateViews(XrSession session, const XrViewLocateInfo *viewLocateInfo, XrViewState *viewState,
                       uint32_t viewCapacityInput, uint32_t *viewCountOutput, XrView *views)
{
    if (auto impl = Session::FromHandle(session)) {
        return impl->LocateViews(viewLocateInfo, viewState, viewCapacityInput, viewCountOutput, views);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrStringToPath(XrInstance instance, const char *pathString, XrPath *path) {
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->StringToPath(pathString, path);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrPathToString(XrInstance instance, XrPath path, uint32_t bufferCapacityInput, 
                        uint32_t *bufferCountOutput, char *buffer)
{
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->PathToString(path, bufferCapacityInput, bufferCountOutput, buffer);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrCreateActionSet(XrInstance instance, const XrActionSetCreateInfo *createInfo, XrActionSet *actionSet) {
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->CreateActionSet(createInfo, actionSet);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrDestroyActionSet(XrActionSet actionSet) {
    if (auto impl = ActionSet::FromHandle(actionSet)) {
        XrInstance instance = impl->GetInstance();
        if (auto instanceImpl = Instance::FromHandle(instance)) {
            instanceImpl->DestroyActionSet(actionSet);
        }
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrCreateAction(XrActionSet actionSet, const XrActionCreateInfo *createInfo, XrAction *action) {
    if (auto impl = ActionSet::FromHandle(actionSet)) {
        return impl->CreateAction(createInfo, action);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrDestroyAction(XrAction action) {
    if (auto impl = Action::FromHandle(action)) {
        XrActionSet actionSet = impl->GetActionSet();
        if (auto actionSetImpl = ActionSet::FromHandle(actionSet)) {
            actionSetImpl->DestroyAction(action);
        }
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrSuggestInteractionProfileBindings(XrInstance instance, 
                                             const XrInteractionProfileSuggestedBinding *suggestedBindings)
{
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->SuggestInteractionProfileBindings(suggestedBindings);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrAttachSessionActionSets(XrSession session, const XrSessionActionSetsAttachInfo *attachInfo) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->AttachSessionActionSets(attachInfo);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrGetCurrentInteractionProfile(XrSession session, XrPath topLevelUserPath, 
                                        XrInteractionProfileState *interactionProfile)
{
    if (auto impl = Session::FromHandle(session)) {
        return impl->GetCurrentInteractionProfile(topLevelUserPath, interactionProfile);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrGetActionStateBoolean(XrSession session, const XrActionStateGetInfo *getInfo, XrActionStateBoolean *state) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->GetActionStateBoolean(getInfo, state);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrGetActionStateFloat(XrSession session, const XrActionStateGetInfo *getInfo, XrActionStateFloat *state) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->GetActionStateFloat(getInfo, state);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrGetActionStateVector2f(XrSession session, const XrActionStateGetInfo *getInfo, XrActionStateVector2f *state) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->GetActionStateVector2f(getInfo, state);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrGetActionStatePose(XrSession session, const XrActionStateGetInfo *getInfo, XrActionStatePose *state) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->GetActionStatePose(getInfo, state);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrSyncActions(XrSession session, const XrActionsSyncInfo *syncInfo) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->SyncActions(syncInfo);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrEnumerateBoundSourcesForAction(XrSession session, const XrBoundSourcesForActionEnumerateInfo *enumerateInfo,
                                          uint32_t sourceCapacityInput, uint32_t *sourceCountOutput, XrPath *sources)
{
    if (auto impl = Session::FromHandle(session)) {
        return impl->EnumerateBoundSourcesForAction(enumerateInfo, sourceCapacityInput, sourceCountOutput, sources);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrGetInputSourceLocalizedName(XrSession session, const XrInputSourceLocalizedNameGetInfo *getInfo,
                                       uint32_t bufferCapacityInput, uint32_t *bufferCountOutput, char *buffer)
{
    if (auto impl = Session::FromHandle(session)) {
        return impl->GetInputSourceLocalizedName(getInfo, bufferCapacityInput, bufferCountOutput, buffer);
    }
    return XR_ERROR_RUNTIME_FAILURE;
}

XrResult xrApplyHapticFeedback(XrSession session, const XrHapticActionInfo *hapticActionInfo, 
                               const XrHapticBaseHeader *hapticFeedback) 
{
    if (auto impl = Session::FromHandle(session)) {
        return impl->ApplyHapticFeedback(hapticActionInfo, hapticFeedback);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrStopHapticFeedback(XrSession session, const XrHapticActionInfo *hapticActionInfo) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->StopHapticFeedback(hapticActionInfo);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrGetMetalGraphicsRequirementsKHR(XrInstance instance,
                                           XrSystemId systemId,
                                           XrGraphicsRequirementsMetalKHR *graphicsRequirements)
{
    if (auto impl = Instance::FromHandle(instance)) {
        return impl->GetMetalGraphicsRequirementsKHR(systemId, graphicsRequirements);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrCreateHandTrackerEXT(XrSession session, const XrHandTrackerCreateInfoEXT* createInfo, XrHandTrackerEXT* handTracker) {
    if (auto impl = Session::FromHandle(session)) {
        return impl->CreateHandTrackerEXT(createInfo, handTracker);
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrDestroyHandTrackerEXT(XrHandTrackerEXT handTracker) {
    if (auto impl = HandTrackerEXT::FromHandle(handTracker)) {
        XrSession session = impl->GetSession();
        if (auto sessionImpl = Session::FromHandle(session)) {
            return sessionImpl->DestroyHandTrackerEXT(handTracker);
        }
    }
    return XR_ERROR_HANDLE_INVALID;
}

XrResult xrLocateHandJointsEXT(XrHandTrackerEXT handTracker, const XrHandJointsLocateInfoEXT* locateInfo, XrHandJointLocationsEXT* locations) {
    if (auto impl = HandTrackerEXT::FromHandle(handTracker)) {
        XrSession session = impl->GetSession();
        if (auto sessionImpl = Session::FromHandle(session)) {
            return sessionImpl->LocateHandJointsEXT(handTracker, locateInfo, locations);
        }
    }
    return XR_ERROR_HANDLE_INVALID;
}

#define INSTANCE_PROC_TABLE_ENTRY(x) std::make_pair(#x, reinterpret_cast<PFN_xrVoidFunction>(x))

XrResult xrGetInstanceProcAddr(XrInstance instance, const char *name, PFN_xrVoidFunction *function) {
    static std::unordered_map<std::string, PFN_xrVoidFunction> procLookupTable {
        // Core 1.0 Functions
        INSTANCE_PROC_TABLE_ENTRY(xrEnumerateInstanceExtensionProperties),
        INSTANCE_PROC_TABLE_ENTRY(xrCreateInstance),
        INSTANCE_PROC_TABLE_ENTRY(xrDestroyInstance),
        INSTANCE_PROC_TABLE_ENTRY(xrGetInstanceProperties),
        INSTANCE_PROC_TABLE_ENTRY(xrPollEvent),
        INSTANCE_PROC_TABLE_ENTRY(xrResultToString),
        INSTANCE_PROC_TABLE_ENTRY(xrStructureTypeToString),
        INSTANCE_PROC_TABLE_ENTRY(xrGetSystem),
        INSTANCE_PROC_TABLE_ENTRY(xrGetSystemProperties),
        INSTANCE_PROC_TABLE_ENTRY(xrEnumerateEnvironmentBlendModes),
        INSTANCE_PROC_TABLE_ENTRY(xrCreateSession),
        INSTANCE_PROC_TABLE_ENTRY(xrDestroySession),
        INSTANCE_PROC_TABLE_ENTRY(xrEnumerateReferenceSpaces),
        INSTANCE_PROC_TABLE_ENTRY(xrCreateReferenceSpace),
        INSTANCE_PROC_TABLE_ENTRY(xrGetReferenceSpaceBoundsRect),
        INSTANCE_PROC_TABLE_ENTRY(xrCreateActionSpace),
        INSTANCE_PROC_TABLE_ENTRY(xrLocateSpace),
        INSTANCE_PROC_TABLE_ENTRY(xrDestroySpace),
        INSTANCE_PROC_TABLE_ENTRY(xrEnumerateViewConfigurations),
        INSTANCE_PROC_TABLE_ENTRY(xrGetViewConfigurationProperties),
        INSTANCE_PROC_TABLE_ENTRY(xrEnumerateViewConfigurationViews),
        INSTANCE_PROC_TABLE_ENTRY(xrEnumerateSwapchainFormats),
        INSTANCE_PROC_TABLE_ENTRY(xrCreateSwapchain),
        INSTANCE_PROC_TABLE_ENTRY(xrDestroySwapchain),
        INSTANCE_PROC_TABLE_ENTRY(xrEnumerateSwapchainImages),
        INSTANCE_PROC_TABLE_ENTRY(xrAcquireSwapchainImage),
        INSTANCE_PROC_TABLE_ENTRY(xrWaitSwapchainImage),
        INSTANCE_PROC_TABLE_ENTRY(xrReleaseSwapchainImage),
        INSTANCE_PROC_TABLE_ENTRY(xrBeginSession),
        INSTANCE_PROC_TABLE_ENTRY(xrEndSession),
        INSTANCE_PROC_TABLE_ENTRY(xrRequestExitSession),
        INSTANCE_PROC_TABLE_ENTRY(xrWaitFrame),
        INSTANCE_PROC_TABLE_ENTRY(xrBeginFrame),
        INSTANCE_PROC_TABLE_ENTRY(xrEndFrame),
        INSTANCE_PROC_TABLE_ENTRY(xrLocateViews),
        INSTANCE_PROC_TABLE_ENTRY(xrStringToPath),
        INSTANCE_PROC_TABLE_ENTRY(xrPathToString),
        INSTANCE_PROC_TABLE_ENTRY(xrCreateActionSet),
        INSTANCE_PROC_TABLE_ENTRY(xrDestroyActionSet),
        INSTANCE_PROC_TABLE_ENTRY(xrCreateAction),
        INSTANCE_PROC_TABLE_ENTRY(xrDestroyAction),
        INSTANCE_PROC_TABLE_ENTRY(xrSuggestInteractionProfileBindings),
        INSTANCE_PROC_TABLE_ENTRY(xrAttachSessionActionSets),
        INSTANCE_PROC_TABLE_ENTRY(xrGetCurrentInteractionProfile),
        INSTANCE_PROC_TABLE_ENTRY(xrGetActionStateBoolean),
        INSTANCE_PROC_TABLE_ENTRY(xrGetActionStateFloat),
        INSTANCE_PROC_TABLE_ENTRY(xrGetActionStateVector2f),
        INSTANCE_PROC_TABLE_ENTRY(xrGetActionStatePose),
        INSTANCE_PROC_TABLE_ENTRY(xrSyncActions),
        INSTANCE_PROC_TABLE_ENTRY(xrEnumerateBoundSourcesForAction),
        INSTANCE_PROC_TABLE_ENTRY(xrGetInputSourceLocalizedName),
        INSTANCE_PROC_TABLE_ENTRY(xrApplyHapticFeedback),
        INSTANCE_PROC_TABLE_ENTRY(xrStopHapticFeedback),
        // XR_KHR_metal_enable
        #if defined(XR_USE_GRAPHICS_API_METAL)
        INSTANCE_PROC_TABLE_ENTRY(xrGetMetalGraphicsRequirementsKHR),
        #endif
        // XR_EXT_hand_interaction
        INSTANCE_PROC_TABLE_ENTRY(xrCreateHandTrackerEXT),
        INSTANCE_PROC_TABLE_ENTRY(xrDestroyHandTrackerEXT),
        INSTANCE_PROC_TABLE_ENTRY(xrLocateHandJointsEXT),
    };
    auto const& proc = procLookupTable.find(name);
    if (proc != procLookupTable.end()) {
        *function = proc->second;
        return XR_SUCCESS;
    } else {
        *function = nullptr;
        return XR_ERROR_FUNCTION_UNSUPPORTED;
    }
}

#undef INSTANCE_PROC_TABLE_ENTRY
