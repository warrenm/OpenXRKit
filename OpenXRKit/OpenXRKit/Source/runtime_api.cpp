/*
** Copyright 2024-2025, Warren Moore
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

#include "runtime_api.hpp"

#define ENUM_CASE_STRING(name, val) case name: return #name;
#define DEFINE_STRING_FROM_ENUMERATOR(enumType)       \
    const char *string_from_enumerator(enumType e) {  \
        switch (e) {                                  \
            XR_LIST_ENUM_##enumType(ENUM_CASE_STRING) \
            default: return nullptr;                  \
        }                                             \
    }

namespace {
DEFINE_STRING_FROM_ENUMERATOR(XrResult)
DEFINE_STRING_FROM_ENUMERATOR(XrStructureType)
}

namespace xr {

XrResult Instance::ResultToString(XrResult value, char *buffer) {
    const char *knownResult = string_from_enumerator(value);
    if (knownResult) {
        strcpy(buffer, knownResult);
    } else {
        if (value > 0) {
            snprintf(buffer, XR_MAX_RESULT_STRING_SIZE, "XR_UNKNOWN_SUCCESS_%d", (int)value);
        } else {
            snprintf(buffer, XR_MAX_RESULT_STRING_SIZE, "XR_UNKNOWN_FAILURE_%d", (int)-value);
        }
    }
    return XR_SUCCESS;
}

XrResult Instance::StructureTypeToString(XrStructureType value, char *buffer) {
    const char *knownResult = string_from_enumerator(value);
    if (knownResult) {
        strcpy(buffer, knownResult);
    } else {
        snprintf(buffer, XR_MAX_STRUCTURE_NAME_SIZE, "XR_UNKNOWN_STRUCTURE_TYPE_%d", (int)value);
    }
    return XR_SUCCESS;
}

XrResult Instance::PollEvent(XrEventDataBuffer *eventData) {
    if (_eventQueue.empty()) {
        return XR_EVENT_UNAVAILABLE;
    }

    Event event = _eventQueue.front();
    _eventQueue.erase(_eventQueue.begin());

    switch (event.type) {
        case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
            XrEventDataSessionStateChanged sessionStateChanged {
                .type = XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED,
                .next = nullptr,
                .session = event.session,
                .state = event.sessionState,
                .time = event.time
            };
            memcpy(eventData, &sessionStateChanged, sizeof(XrEventDataSessionStateChanged));
            return XR_SUCCESS;
        }
        case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED: {
            XrEventDataInteractionProfileChanged interactionProfileChanged {
                .type = XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED,
                .next = nullptr,
                .session = event.session,
            };
            memcpy(eventData, &interactionProfileChanged, sizeof(XrEventDataInteractionProfileChanged));
            return XR_SUCCESS;
        }
        default:
            printf("Dequeued an event of unknown type %d\n", (int)event.type);
            return PollEvent(eventData);
    }
}

void Instance::PostEvent(Event event) {
    _eventQueue.push_back(std::move(event));
}

ActionSet::~ActionSet() {
    for (auto it : _actions) {
        xr::Action::DestroyInstance(it);
    }
}

XrResult ActionSet::CreateAction(const XrActionCreateInfo *createInfo, XrAction *action) {
    if (createInfo == nullptr) {
        return XR_ERROR_VALIDATION_FAILURE;
    }

    *action = Action::RegisterInstance(std::make_unique<Action>(GetHandle(), *createInfo));
    _actions.push_back(*action);
    return XR_SUCCESS;
}

XrResult ActionSet::DestroyAction(XrAction action) {
    erase(_actions, action);
    return Action::DestroyInstance(action);
}

}
