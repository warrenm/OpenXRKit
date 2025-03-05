/*
** Copyright 2024-2025, Warren Moore
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

#include "runtime_api.hpp"

#import <GameController/GameController.h>

namespace {

class ButtonInputSource {
public:
    struct StateSample {
        XrTime time = 0;
        float analog = 0.0f;
        bool digital = false;
    };

    ButtonInputSource(XrInstance instance, XrPath rootInputSourcePath) :
        _instance { instance },
        _rootInputSourcePath { rootInputSourcePath }
    {
    }

    bool BindToLogicalInput(GCControllerButtonInput *buttonInput) {
        _buttonInput = buttonInput;
        _buttonInput.valueChangedHandler = ^(GCControllerButtonInput *button, float value, BOOL pressed) {
            _mostRecentState.time = XrTime{}; // TODO: Get this from a well-known clock source
            _mostRecentState.analog = value;
            _mostRecentState.digital = pressed;
        };
        return true;
    }

    void CopyBooleanState(XrActionStateBoolean &outState) {
        outState.isActive = _active;
        if (_active) {
            outState.currentState = _mostRecentState.digital;
            outState.lastChangeTime = _mostRecentState.time;
            outState.changedSinceLastSync = (_lastSyncState.digital != _mostRecentState.digital) || (_lastSyncState.time == 0);
        }
    }

    void CopyFloatState(XrActionStateFloat &outState) {
        outState.isActive = _active;
        if (_active) {
            outState.currentState = _mostRecentState.analog;
            outState.lastChangeTime = _mostRecentState.time;
            outState.changedSinceLastSync = (_lastSyncState.digital != _mostRecentState.digital) || (_lastSyncState.time == 0);
        }
    }

    void SyncAtTime(XrTime time) {
        if (_active) {
            _lastSyncState = { time, _mostRecentState.analog, _mostRecentState.digital };
        }
    }

private:
    XrInstance _instance = XR_NULL_HANDLE;
    GCControllerButtonInput *_buttonInput = nil;
    XrPath _rootInputSourcePath;
    StateSample _mostRecentState {};
    StateSample _lastSyncState {};
    bool _active = false;
};

class GamepadAxis {
};

class GamepadDPadComponent {
};

class ExtendedGamepadProfile {
public:
    explicit ExtendedGamepadProfile(XrInstance instance) :
        _instance { instance }
    {
    }

    void BindToLogicalController(GCExtendedGamepad *gamepad) {
        //_aButton.BindToLogicalInput(gamepad.buttonA);
    }

    XrResult MapInputPathToComponentPath(XrPath inputSource, XrActionType actionType, XrPath &outComponent) {
        char inputString[XR_MAX_PATH_LENGTH]; uint32_t strLen;
        xrPathToString(_instance, inputSource, XR_MAX_PATH_LENGTH, &strLen, inputString);

        std::array inputPathStrings = {
            "/user/gamepad/input/menu"
            "/user/gamepad/input/view"
            "/user/gamepad/input/a"
            "/user/gamepad/input/b"
            "/user/gamepad/input/x"
            "/user/gamepad/input/y"
            "/user/gamepad/input/dpad_down"
            "/user/gamepad/input/dpad_right"
            "/user/gamepad/input/dpad_up"
            "/user/gamepad/input/dpad_left"
            "/user/gamepad/input/shoulder_left"
            "/user/gamepad/input/shoulder_right"
            "/user/gamepad/input/thumbstick_left"
            "/user/gamepad/input/thumbstick_right"
            "/user/gamepad/input/trigger_left"
            "/user/gamepad/input/trigger_right"
            "/user/gamepad/output/haptic_left"
            "/user/gamepad/output/haptic_right"
            "/user/gamepad/output/haptic_left_trigger"
            "/user/gamepad/output/haptic_right_trigger"
        };
        return XR_SUCCESS;
    }

private:
    XrInstance _instance;

    //ButtonInputSource _menuButton;
    //ButtonInputSource _optionsButton; // optional; not present on all supported controllers
    //ButtonInputSource _homeButton;    // optional; not present on all supported controllers
    //ButtonInputSource _aButton;
    //ButtonInputSource _bButton;
    //ButtonInputSource _xButton;
    //ButtonInputSource _yButton;
    //GamepadDPadComponent _dpad;
    //GamepadDPadComponent _leftThumbstick;
    //GamepadDPadComponent _rightThumbstick;
    //ButtonInputSource _leftThumbstickButton;  // optional; not present on all supported controllers
    //ButtonInputSource _rightThumbstickButton; // optional; not present on all supported controllers
    //ButtonInputSource _leftShoulder;
    //ButtonInputSource _rightShoulder;
    //ButtonInputSource _leftTrigger;
    //ButtonInputSource _rightTrigger;
};

class InteractionManager_GameController : public xr::InteractionManager {
public:
    explicit InteractionManager_GameController(XrInstance instance) :
        _instance { instance }
    {
        (void)xrStringToPath(_instance, "/user/gamepad", &_userGamepadPath);
        (void)xrStringToPath(_instance, "/interaction_profiles/microsoft/xbox_controller", &_msftXboxControllerPath);

        _observers = [NSMutableArray array];
        RegisterForControllerNotifications();
    }

    XrInstance GetInstance() const {
        return _instance;
    }

    void GetSupportedInteractionProfiles(std::vector<XrPath> &interactionProfiles) override {
        interactionProfiles.clear();
        interactionProfiles.push_back(_msftXboxControllerPath);
    }

    bool BindInteractionProfile(XrPath interactionProfile) override {
        if (_boundInteractionProfile == XR_NULL_PATH) {
            _boundInteractionProfile = interactionProfile;
            return true;
        } else {
            return false;
        }
    }

    XrResult AttachSessionActionSets(const XrSessionActionSetsAttachInfo *attachInfo) override {
        auto instance = xr::Instance::FromHandle(GetInstance());
        if (instance == nullptr) {
            return XR_ERROR_INSTANCE_LOST;
        }
        for (int i = 0; i < attachInfo->countActionSets; ++i) {
            XrActionSet setHandle = attachInfo->actionSets[i];
            xr::ActionSet *actionSet = xr::ActionSet::FromHandle(setHandle);
            if (actionSet == nullptr) {
                return XR_ERROR_HANDLE_INVALID;
            }
            if (actionSet->IsAttached()) {
                return XR_ERROR_ACTIONSETS_ALREADY_ATTACHED;
            }
            for (auto const &actionHandle : actionSet->GetActions()) {
                xr::Action *action = xr::Action::FromHandle(actionHandle);
                if (action == nullptr) {
                    return XR_ERROR_HANDLE_INVALID;
                }
                //AttachAction(actionHandle);
                //printf("Would have tried to attach action %s in action set %s\n", action->GetName().c_str(), actionSet->GetLocalizedName().c_str());
            }
            actionSet->SetAttached();
        }
        //_hasAttachedActionSets = true;
        return XR_SUCCESS;
    }

    XrResult GetCurrentInteractionProfile(XrPath topLevelUserPath, XrInteractionProfileState *interactionProfile) override {
        auto instance = xr::Instance::FromHandle(GetInstance());
        if (instance == nullptr) {
            return XR_ERROR_INSTANCE_LOST;
        }
        //if (!_hasAttachedActionSets) {
        //    return XR_ERROR_ACTIONSET_NOT_ATTACHED;
        //}
        if (interactionProfile == nullptr || interactionProfile->type != XR_TYPE_INTERACTION_PROFILE_STATE) {
            return XR_ERROR_VALIDATION_FAILURE;
        }
        if (topLevelUserPath == _userGamepadPath) {
            interactionProfile->interactionProfile = _boundInteractionProfile;
            return XR_SUCCESS;
        } else {
            interactionProfile->interactionProfile = XR_NULL_PATH;
            return XR_ERROR_PATH_UNSUPPORTED;
        }
    }

    XrResult GetActionStateBoolean(const XrActionStateGetInfo *getInfo, XrActionStateBoolean *state) override {
        XrAction actionHandle = getInfo->action;
        xr::Action *action = xr::Action::FromHandle(actionHandle);
        if (action == nullptr) {
            return XR_ERROR_HANDLE_INVALID;
        }

        state->isActive = false;
        state->changedSinceLastSync = false;
        state->currentState = action->GetBoolValue();
        state->lastChangeTime = 0;

        return XR_SUCCESS;
    }

    XrResult GetActionStateFloat(const XrActionStateGetInfo *getInfo, XrActionStateFloat *state) override {
        XrAction actionHandle = getInfo->action;
        xr::Action *action = xr::Action::FromHandle(actionHandle);
        if (action == nullptr) {
            return XR_ERROR_HANDLE_INVALID;
        }

        state->isActive = false;
        state->changedSinceLastSync = false;
        state->currentState = action->GetFloatValue();
        state->lastChangeTime = 0;

        return XR_SUCCESS;
    }

    XrResult GetActionStateVector2f(const XrActionStateGetInfo *getInfo, XrActionStateVector2f *state) override {
        XrAction actionHandle = getInfo->action;
        auto action = xr::Action::FromHandle(actionHandle);
        if (action == nullptr) {
            return XR_ERROR_HANDLE_INVALID;
        }

        state->isActive = false;
        state->changedSinceLastSync = false;
        state->currentState = action->GetVector2fValue();
        state->lastChangeTime = 0;

        return XR_SUCCESS;
    }

    XrResult GetActionStatePose(const XrActionStateGetInfo *getInfo, XrActionStatePose *state) override {
        XrAction actionHandle = getInfo->action;
        auto action = xr::Action::FromHandle(actionHandle);
        if (action == nullptr) {
            return XR_ERROR_HANDLE_INVALID;
        }

        state->isActive = false;

        return XR_SUCCESS;
    }

    XrResult SyncActions(const XrActionsSyncInfo *syncInfo) override {
        for (int i = 0; i < syncInfo->countActiveActionSets; ++i) {
            const XrActiveActionSet &activeActionSet = syncInfo->activeActionSets[i];
            XrActionSet actionSetHandle = activeActionSet.actionSet;
            XrPath subactionPath = activeActionSet.subactionPath;
            auto actionSet = xr::ActionSet::FromHandle(actionSetHandle);
            if (actionSet == nullptr) {
                return XR_ERROR_HANDLE_INVALID;
            }
            if (!actionSet->IsAttached()) {
                return XR_ERROR_ACTIONSET_NOT_ATTACHED;
            }
        }
        return XR_SUCCESS;
    }

    XrResult EnumerateBoundSourcesForAction(const XrBoundSourcesForActionEnumerateInfo *enumerateInfo,
                                            uint32_t sourceCapacityInput, 
                                            uint32_t *sourceCountOutput, XrPath *sources) override
    {
        if (enumerateInfo->type != XR_TYPE_BOUND_SOURCES_FOR_ACTION_ENUMERATE_INFO) {
            return XR_ERROR_VALIDATION_FAILURE;
        }
        auto action = xr::Action::FromHandle(enumerateInfo->action);
        *sourceCountOutput = 0;
        return XR_SUCCESS;
    }

    XrResult GetInputSourceLocalizedName(const XrInputSourceLocalizedNameGetInfo *getInfo, uint32_t bufferCapacityInput,
                                         uint32_t *bufferCountOutput, char *buffer) override
    {
        return XR_ERROR_RUNTIME_FAILURE;
    }

    XrResult ApplyHapticFeedback(const XrHapticActionInfo *hapticActionInfo,
                                 const XrHapticBaseHeader *hapticFeedback) override
    {
        return XR_ERROR_RUNTIME_FAILURE;
    }

    XrResult StopHapticFeedback(const XrHapticActionInfo *hapticActionInfo) override {
        return XR_ERROR_RUNTIME_FAILURE;
    }

private:

    void RegisterForControllerNotifications() {
        NSNotificationCenter *notificationCenter = [NSNotificationCenter defaultCenter];
        NSOperationQueue *queue = NSOperationQueue.mainQueue;
        id ob = nil;
        ob = [notificationCenter addObserverForName:GCControllerDidConnectNotification object:nil queue:queue usingBlock:^(NSNotification *notification) {
            this->ControllerDidConnect((GCController *)notification.object);
        }];
        [_observers addObject:ob];
        ob = [notificationCenter addObserverForName:GCControllerDidBecomeCurrentNotification object:nil queue:queue usingBlock:^(NSNotification *notification) {
            this->ControllerDidBecomeCurrent((GCController *)notification.object);
        }];
        [_observers addObject:ob];
        ob = [notificationCenter addObserverForName:GCControllerDidDisconnectNotification object:nil queue:queue usingBlock:^(NSNotification *notification) {
            this->ControllerDidDisconnect((GCController *)notification.object);
        }];
        [_observers addObject:ob];
        ob = [notificationCenter addObserverForName:GCControllerDidStopBeingCurrentNotification  object:nil queue:queue usingBlock:^(NSNotification *notification) {
            this->ControllerDidStopBeingCurrent((GCController *)notification.object);
        }];
        [_observers addObject:ob];
    }

    void SelectPreferredController() {
        if (_preferredController == nil || _preferredController != GCController.current) {
            _preferredController = GCController.current;
            //if (_preferredController) {
            //    GCMicroGamepad *microGamepad = _preferredController.microGamepad;
            //    GCExtendedGamepad *extendedGamepad = _preferredController.extendedGamepad;
            //    if (extendedGamepad != nil) {
            //        AddStateChangeHandlersToExtendedGamepad(extendedGamepad);
            //    } else if (microGamepad != nil) {
            //        AddStateChangeHandlersToMicroGamepad(microGamepad);
            //    } else {
            //        _preferredController = nil; // No usable profiles available
            //    }
            //}
        }
    }

    void DeselectPreferredController() {
        if (_preferredController != nil) {
            _preferredController = nil;
        }
    }

    void ControllerDidConnect(GCController *controller) {
        SelectPreferredController();
    }

    void ControllerDidBecomeCurrent(GCController *controller) {
        SelectPreferredController();
    }

    void ControllerDidDisconnect(GCController *controller) {
        if (controller == _preferredController) {
            DeselectPreferredController();
        }
    }

    void ControllerDidStopBeingCurrent(GCController *controller) {
        if (controller == _preferredController) {
            DeselectPreferredController();
        }
    }

private:
    XrInstance _instance = XR_NULL_PATH;
    NSMutableArray *_observers = nil;
    GCController *_preferredController = nil;
    XrPath _userGamepadPath = XR_NULL_PATH; // "/user/gamepad"
    XrPath _msftXboxControllerPath = XR_NULL_PATH; // "/interaction_profiles/microsoft/xbox_controller"
    XrPath _boundInteractionProfile = XR_NULL_PATH;
};

} // end anonymous namespace

std::unique_ptr<xr::InteractionManager> CreateInteractionManager_Apple(XrInstance instance) {
    return std::make_unique<InteractionManager_GameController>(instance);
}
