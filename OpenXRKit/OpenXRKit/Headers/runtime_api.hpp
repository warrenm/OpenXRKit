/*
** Copyright 2024-2025, Warren Moore
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

#pragma once

#include "openxr_api.h"

#include <openxr/openxr_reflection.h>

#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#if defined(__APPLE__)
#define OS_APPLE
#include <TargetConditionals.h>
#if TARGET_OS_OSX
#define OS_APPLE_MACOS
#elif TARGET_OS_IOS
#define OS_APPLE_IOS
#elif TARGET_OS_VISION
#define OS_APPLE_VISIONOS
#endif
#endif

#if __cplusplus < 202002L
template <class T, class Alloc, class U>
typename std::vector<T, Alloc>::size_type erase(std::vector<T, Alloc>& c, const U& value) {
    auto it = std::remove(begin(c), end(c), value);
    auto r = c.end() - it;
    c.erase(it, c.end());
    return r;
}
#endif

namespace xr {

class Action;
class ActionSet;

struct Event {
public:
    XrStructureType type;
    XrSession session = XR_NULL_HANDLE;
    XrTime time = 0;
    union {
        XrSessionState sessionState = XrSessionState::XR_SESSION_STATE_IDLE;
    };
};

template <typename HandleType, typename InstanceType>
class RuntimeBase {
public:
    static HandleType RegisterInstance(std::unique_ptr<InstanceType> instance) {
        auto handle = instance->GetHandle();
        _instances.insert(std::make_pair(handle, std::move(instance)));
        return handle;
    }

    static InstanceType *FromHandle(HandleType handle) {
        auto const& it = _instances.find(handle);
        if (it == _instances.end()) {
            return nullptr;
        } else {
            return it->second.get();
        }
    }

    static XrResult DestroyInstance(HandleType handle) {
        auto const &it = _instances.find(handle);
        if (it == _instances.end()) {
            return XR_ERROR_HANDLE_INVALID;
        }
        _instances.erase(it);
        return XR_SUCCESS;
    }

    RuntimeBase() {
        uint64_t objectIndex = static_cast<uint64_t>(_nextUniqueObjectIndex++);
        uint64_t typeTag = static_cast<uint64_t>(InstanceType::GetTypeTag());
        _handle = reinterpret_cast<HandleType>((typeTag << 32) | objectIndex);
    }

    virtual ~RuntimeBase() = default;

    HandleType GetHandle() const {
        return _handle;
    }

private:
    static inline std::unordered_map<HandleType, std::unique_ptr<InstanceType>> _instances;
    static inline uint32_t _nextUniqueObjectIndex = 1;
    HandleType _handle;
};

class Instance : public RuntimeBase<XrInstance, Instance> {
public:
    Instance() = default;

    static uint32_t GetTypeTag() { return XR_OBJECT_TYPE_INSTANCE; }

    virtual XrResult GetInstanceProperties(XrInstanceProperties *outInstanceProperties) = 0;

    virtual XrResult ResultToString(XrResult value, char *buffer);

    virtual XrResult StructureTypeToString(XrStructureType value, char *buffer);

    virtual XrResult GetSystem(const XrSystemGetInfo* getInfo, XrSystemId* systemId) = 0;
    virtual XrResult GetSystemProperties(XrSystemId systemId, XrSystemProperties* properties) = 0;
    virtual XrResult StringToPath(const char* pathString, XrPath* path) = 0;
    virtual XrResult PathToString(XrPath path, uint32_t bufferCapacityInput,
                                  uint32_t* bufferCountOutput, char* buffer) = 0;
    virtual XrResult EnumerateViewConfigurations(XrSystemId systemId, uint32_t viewConfigurationTypeCapacityInput,
                                                 uint32_t* viewConfigurationTypeCountOutput,
                                                 XrViewConfigurationType* viewConfigurationTypes) = 0;
    virtual XrResult GetViewConfigurationProperties(XrSystemId systemId, XrViewConfigurationType viewConfigurationType,
                                                    XrViewConfigurationProperties* configurationProperties) = 0;
    virtual XrResult EnumerateViewConfigurationViews(XrSystemId systemId, XrViewConfigurationType viewConfigurationType,
                                                     uint32_t viewCapacityInput, uint32_t* viewCountOutput,
                                                     XrViewConfigurationView* views) = 0;
    virtual XrResult EnumerateEnvironmentBlendModes(XrSystemId systemId, XrViewConfigurationType viewConfigurationType,
                                                    uint32_t environmentBlendModeCapacityInput,
                                                    uint32_t* environmentBlendModeCountOutput,
                                                    XrEnvironmentBlendMode* environmentBlendModes) = 0;
    virtual XrResult CreateSession(const XrSessionCreateInfo* createInfo, XrSession* session) = 0;
    virtual XrResult DestroySession(XrSession session) = 0;
    virtual XrResult SuggestInteractionProfileBindings(const XrInteractionProfileSuggestedBinding* suggestedBindings) = 0;
    virtual XrResult CreateActionSet(const XrActionSetCreateInfo* createInfo, XrActionSet* actionSet) = 0;
    virtual XrResult DestroyActionSet(XrActionSet actionSet) = 0;

#if defined(XR_USE_GRAPHICS_API_METAL)
    virtual XrResult GetMetalGraphicsRequirementsKHR(XrSystemId systemId,
                                                     XrGraphicsRequirementsMetalKHR *graphicsRequirements) = 0;
#endif

    virtual XrResult PollEvent(XrEventDataBuffer *eventData);
    virtual void PostEvent(Event event);

    virtual void GetSuggestedInteractionProfiles(std::vector<XrPath> &interactionProfiles) = 0;
    virtual void GetSuggestedActionBindings(XrPath interactionProfile, std::vector<XrActionSuggestedBinding> &bindings) = 0;

protected:
    std::deque<Event> _eventQueue;
};

class System {
public:
    explicit System(XrSystemId systemId) :
        _systemId { systemId }
    {}

    virtual ~System() = default;

    XrSystemId GetSystemId() {
        return _systemId;
    }

    virtual bool SupportsFormFactor(XrFormFactor formFactor) = 0;

    virtual XrResult GetSystemProperties(XrSystemProperties *properties) = 0;
    virtual XrResult GetViewConfigurations(std::vector<XrViewConfigurationType> &configurations) = 0;
    virtual XrResult GetPropertiesForViewConfiguration(XrViewConfigurationType configuration,
                                                       XrViewConfigurationProperties *properties) = 0;
    virtual XrResult GetViewsForViewConfiguration(XrViewConfigurationType configuration,
                                                  std::vector<XrViewConfigurationView> &views) = 0;
    virtual XrResult GetBlendModesForViewConfiguration(XrViewConfigurationType configuration,
                                                       std::vector<XrEnvironmentBlendMode> &blendModes) = 0;
    virtual XrResult GetMetalGraphicsRequirementsKHR(XrGraphicsRequirementsMetalKHR *graphicsRequirements) = 0;

private:
    XrSystemId _systemId;
};

class InteractionManager {
public:
    virtual ~InteractionManager() = default;

    virtual void GetSupportedInteractionProfiles(std::vector<XrPath> &interactionProfiles) = 0;
    virtual bool BindInteractionProfile(XrPath interactionProfile) = 0;

    virtual XrResult AttachSessionActionSets(const XrSessionActionSetsAttachInfo* attachInfo) = 0;
    virtual XrResult GetCurrentInteractionProfile(XrPath topLevelUserPath, XrInteractionProfileState* interactionProfile) = 0;
    virtual XrResult GetActionStateBoolean(const XrActionStateGetInfo* getInfo, XrActionStateBoolean* state) = 0;
    virtual XrResult GetActionStateFloat(const XrActionStateGetInfo* getInfo, XrActionStateFloat* state) = 0;
    virtual XrResult GetActionStateVector2f(const XrActionStateGetInfo* getInfo, XrActionStateVector2f* state) = 0;
    virtual XrResult GetActionStatePose(const XrActionStateGetInfo* getInfo, XrActionStatePose* state) = 0;
    virtual XrResult SyncActions(const XrActionsSyncInfo* syncInfo) = 0;
    virtual XrResult EnumerateBoundSourcesForAction(const XrBoundSourcesForActionEnumerateInfo* enumerateInfo,
                                                    uint32_t sourceCapacityInput, uint32_t* sourceCountOutput,
                                                    XrPath* sources) = 0;
    virtual XrResult GetInputSourceLocalizedName(const XrInputSourceLocalizedNameGetInfo* getInfo,
                                                 uint32_t bufferCapacityInput, uint32_t* bufferCountOutput, char* buffer) = 0;
    virtual XrResult ApplyHapticFeedback(const XrHapticActionInfo* hapticActionInfo,
                                         const XrHapticBaseHeader* hapticFeedback) = 0;
    virtual XrResult StopHapticFeedback(const XrHapticActionInfo* hapticActionInfo) = 0;
};

class Session : public RuntimeBase<XrSession, Session> {
public:
    static uint32_t GetTypeTag() { return XR_OBJECT_TYPE_SESSION; }

    Session(XrInstance parent) : _instance(parent) {}

    XrInstance GetInstance() const { return _instance; }

    virtual XrResult EnumerateReferenceSpaces(uint32_t spaceCapacityInput,
                                              uint32_t* spaceCountOutput, XrReferenceSpaceType* spaces) = 0;
    virtual XrResult CreateReferenceSpace(const XrReferenceSpaceCreateInfo* createInfo, XrSpace* space) = 0;
    virtual XrResult GetReferenceSpaceBoundsRect(XrReferenceSpaceType referenceSpaceType, XrExtent2Df* bounds) = 0;
    virtual XrResult CreateActionSpace(const XrActionSpaceCreateInfo* createInfo, XrSpace* space) = 0;
    virtual XrResult DestroySpace(XrSpace space) = 0;
    virtual XrResult LocateViews(const XrViewLocateInfo* viewLocateInfo, XrViewState* viewState,
                                 uint32_t viewCapacityInput, uint32_t* viewCountOutput, XrView* views) = 0;
    virtual XrResult LocateSpace(XrSpace space, XrSpace baseSpace, XrTime time, XrSpaceLocation *location) = 0;

    virtual XrResult EnumerateSwapchainFormats(uint32_t formatCapacityInput,
                                               uint32_t* formatCountOutput, int64_t* formats) = 0;
    virtual XrResult CreateSwapchain( const XrSwapchainCreateInfo* createInfo, XrSwapchain* swapchain) = 0;
    virtual XrResult DestroySwapchain(XrSwapchain swapchain) = 0;

    virtual XrResult BeginSession(const XrSessionBeginInfo* beginInfo) = 0;
    virtual XrResult EndSession() = 0;
    virtual XrResult RequestExitSession() = 0;

    virtual XrResult WaitFrame(const XrFrameWaitInfo* frameWaitInfo, XrFrameState* frameState) = 0;
    virtual XrResult BeginFrame(const XrFrameBeginInfo* frameBeginInfo) = 0;
    virtual XrResult EndFrame(const XrFrameEndInfo* frameEndInfo) = 0;

    virtual XrResult AttachSessionActionSets(const XrSessionActionSetsAttachInfo* attachInfo) = 0;
    virtual XrResult GetCurrentInteractionProfile(XrPath topLevelUserPath, XrInteractionProfileState* interactionProfile) = 0;
    virtual XrResult GetActionStateBoolean(const XrActionStateGetInfo* getInfo, XrActionStateBoolean* state) = 0;
    virtual XrResult GetActionStateFloat(const XrActionStateGetInfo* getInfo, XrActionStateFloat* state) = 0;
    virtual XrResult GetActionStateVector2f(const XrActionStateGetInfo* getInfo, XrActionStateVector2f* state) = 0;
    virtual XrResult GetActionStatePose(const XrActionStateGetInfo* getInfo, XrActionStatePose* state) = 0;
    virtual XrResult SyncActions(const XrActionsSyncInfo* syncInfo) = 0;
    virtual XrResult EnumerateBoundSourcesForAction(const XrBoundSourcesForActionEnumerateInfo* enumerateInfo,
                                                    uint32_t sourceCapacityInput, uint32_t* sourceCountOutput,
                                                    XrPath* sources) = 0;
    virtual XrResult GetInputSourceLocalizedName(const XrInputSourceLocalizedNameGetInfo* getInfo,
                                                 uint32_t bufferCapacityInput, uint32_t* bufferCountOutput, char* buffer) = 0;
    virtual XrResult ApplyHapticFeedback(const XrHapticActionInfo* hapticActionInfo,
                                         const XrHapticBaseHeader* hapticFeedback) = 0;
    virtual XrResult StopHapticFeedback(const XrHapticActionInfo* hapticActionInfo) = 0;

    virtual XrResult CreateHandTrackerEXT(const XrHandTrackerCreateInfoEXT *createInfo, XrHandTrackerEXT *handTracker) {
        return XR_ERROR_FEATURE_UNSUPPORTED;
    }
    virtual XrResult DestroyHandTrackerEXT(XrHandTrackerEXT handTracker) {
        return XR_ERROR_FUNCTION_UNSUPPORTED;
    }
    virtual XrResult LocateHandJointsEXT(XrHandTrackerEXT handTracker, const XrHandJointsLocateInfoEXT *locateInfo, XrHandJointLocationsEXT *locations) {
        return XR_ERROR_FUNCTION_UNSUPPORTED;
    }

    virtual void OnSuggestedActionBindingsChanged() {}

private:
    XrInstance _instance = XR_NULL_HANDLE;
};

class ActionSet : public RuntimeBase<XrActionSet, ActionSet> {
public:
    static uint32_t GetTypeTag() { return XR_OBJECT_TYPE_ACTION_SET; }

    ActionSet(XrInstance instance, const XrActionSetCreateInfo &createInfo) :
        _instance { instance },
        _name { createInfo.actionSetName },
        _localizedName { createInfo.localizedActionSetName },
        _priority { createInfo.priority }
    {}

    ~ActionSet();
    
    XrInstance GetInstance() const {
        return _instance;
    }

    virtual XrResult CreateAction(const XrActionCreateInfo *createInfo, XrAction *action);
    virtual XrResult DestroyAction(XrAction action);

    std::string const& GetName() const {
        return _name;
    }

    std::string const& GetLocalizedName() const {
        return _localizedName;
    }

    std::vector<XrAction> const& GetActions() const {
        return _actions;
    }

    bool IsAttached() const {
        return _attached;
    }

    void SetAttached() {
        // It's not valid to attach an already-attached action set, but we don't enforce that here.
        _attached = true;
    }

private:
    std::vector<XrAction> _actions;
    std::string _name;
    std::string _localizedName;
    uint32_t _priority;
    XrInstance _instance = XR_NULL_HANDLE;
    bool _attached = false;
};

class Action : public RuntimeBase<XrAction, Action> {
public:
    static uint32_t GetTypeTag() { return XR_OBJECT_TYPE_ACTION; }

    Action(XrActionSet actionSet, const XrActionCreateInfo &createInfo) :
        _actionSet { actionSet }
    {
        _name = createInfo.actionName;
        _localizedName = createInfo.localizedActionName;
        _actionType = createInfo.actionType;
        for (uint32_t i = 0; i < createInfo.countSubactionPaths; ++i) {
            _subactionPaths.push_back(createInfo.subactionPaths[i]);
        }
    }

    XrActionSet GetActionSet() const {
        return _actionSet;
    }

    std::string const& GetName() const {
        return _name;
    }

    std::string const& GetLocalizedName() const {
        return _localizedName;
    }

    XrActionType GetActionType() const {
        return _actionType;
    }

    std::vector<XrPath> const& GetSubactionPaths() const {
        return _subactionPaths;
    }

    virtual bool GetBoolValue() { return false; }
    virtual float GetFloatValue() { return 0.0f; }
    virtual XrVector2f GetVector2fValue() { return XrVector2f {}; }
    virtual XrPosef GetPosefValue() { return XrPosef{}; }

private:
    XrActionSet _actionSet;
    std::string _name;
    std::string _localizedName;
    XrActionType _actionType;
    std::vector<XrPath> _subactionPaths;
};

class Space : public RuntimeBase<XrSpace, Space> {
public:
    static uint32_t GetTypeTag() { return XR_OBJECT_TYPE_SPACE; }

    Space(XrSession session) : _session(session) {}

    XrSession GetSession() const {
        return _session;
    }

private:
    XrSession _session = XR_NULL_HANDLE;
};

class Swapchain : public RuntimeBase<XrSwapchain, Swapchain> {
public:
    static uint32_t GetTypeTag() { return XR_OBJECT_TYPE_SWAPCHAIN; }

    Swapchain(XrSession session) : _session(session) {}

    XrSession GetSession() const {
        return _session;
    }

    virtual XrResult EnumerateSwapchainImages(uint32_t imageCapacityInput,
                                              uint32_t* imageCountOutput, XrSwapchainImageBaseHeader* images) = 0;
    virtual XrResult AcquireSwapchainImage(const XrSwapchainImageAcquireInfo* acquireInfo, uint32_t* index) = 0;
    virtual XrResult WaitSwapchainImage(const XrSwapchainImageWaitInfo* waitInfo) = 0;
    virtual XrResult ReleaseSwapchainImage(const XrSwapchainImageReleaseInfo* releaseInfo) = 0;

private:
    XrSession _session = XR_NULL_HANDLE;
};

class HandTrackerEXT : public RuntimeBase<XrHandTrackerEXT, HandTrackerEXT> {
public:
    static uint32_t GetTypeTag() { return XR_OBJECT_TYPE_HAND_TRACKER_EXT; }

    HandTrackerEXT(XrSession session, XrHandEXT hand, XrHandJointSetEXT handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT) :
        _session { session },
        _hand { hand },
        _handJointSet { handJointSet }
    {}

    XrSession GetSession() const {
        return _session;
    }

    XrHandEXT GetHand() const {
        return _hand;
    }

private:
    XrSession _session = XR_NULL_HANDLE;
    XrHandEXT _hand;
    XrHandJointSetEXT _handJointSet;
};

} // namespace xr
