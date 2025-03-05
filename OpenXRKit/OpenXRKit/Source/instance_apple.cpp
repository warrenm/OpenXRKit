/*
** Copyright 2024-2025, Warren Moore
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

#include "runtime_apple.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#define OPENXRKIT_RUNTIME_NAME "OpenXRKit"
#define OPENXRKIT_RUNTIME_VERSION XR_MAKE_VERSION(0, 0, 1)

using namespace xr;

namespace {

class Instance_Apple : public Instance {
public:
    Instance_Apple() {
#if defined(OS_APPLE_IOS)
        XrSystemId systemID = _nextSystemAtom++;
        _systems.insert(std::make_pair(systemID, CreateSystem_iOS(systemID)));
#elif defined(OS_APPLE_VISIONOS)
        XrSystemId systemID = _nextSystemAtom++;
        _systems.insert(std::make_pair(systemID, CreateSystem_visionOS(systemID)));
#else
#error No known OpenXR system implementations for this platform.
#endif
    }

    ~Instance_Apple() {
        for (auto it : _sessions) {
            Session::DestroyInstance(it);
        }
        for (auto it : _actionSets) {
            ActionSet::DestroyInstance(it);
        }
    }

    XrResult GetInstanceProperties(XrInstanceProperties *outInstanceProperties) override {
        outInstanceProperties->type = XR_TYPE_INSTANCE_PROPERTIES;
        outInstanceProperties->next = nullptr;
        outInstanceProperties->runtimeVersion = OPENXRKIT_RUNTIME_VERSION;
        strcpy(outInstanceProperties->runtimeName, OPENXRKIT_RUNTIME_NAME);
        return XR_SUCCESS;
    }

    XrResult GetSystem(const XrSystemGetInfo *getInfo, XrSystemId *systemId) override {
        for (auto const& system : _systems) {
            if (system.second->SupportsFormFactor(getInfo->formFactor)) {
                *systemId = static_cast<XrSystemId>(system.first);
                return XR_SUCCESS;
            }
        }

        // Apple platforms don't support pluggable HMDs, so if our built-in system doesn't
        // support the requested form factor, we have no fallback.
        return XR_ERROR_FORM_FACTOR_UNSUPPORTED;
    }

    XrResult GetSystemProperties(XrSystemId systemId, XrSystemProperties *properties) override {
        auto const& system = _systems.find(systemId);
        if (system == _systems.end()) {
            return XR_ERROR_SYSTEM_INVALID;
        }

        return system->second->GetSystemProperties(properties);
    }

    XrResult StringToPath(const char *pathString, XrPath *path) override {
        std::string key { pathString };
        auto const &it = _pathAtoms.find(key);
        if (it != _pathAtoms.end()) {
            *path = it->second;
        } else {
            XrPath atom = _nextPathAtom++;
            _pathAtoms[key] = atom;
            *path = atom;
        }
        return XR_SUCCESS;
    }

    XrResult PathToString(XrPath path, uint32_t bufferCapacityInput, uint32_t *bufferCountOutput, char *buffer) override {
        if (buffer == nullptr || bufferCountOutput == nullptr) {
            return XR_ERROR_VALIDATION_FAILURE;
        }
        for (auto const &p : _pathAtoms) {
            if (p.second == path) {
                const std::string &key = p.first;
                size_t length = key.length();
                if (length + 1 <= bufferCapacityInput) {
                    strcpy(buffer, key.c_str());
                    *bufferCountOutput = uint32_t(length + 1);
                    return XR_SUCCESS;
                } else {
                    return XR_ERROR_SIZE_INSUFFICIENT;
                }
            }
        }
        return XR_ERROR_PATH_INVALID;
    }

    XrResult EnumerateViewConfigurations(XrSystemId systemId, uint32_t viewConfigurationTypeCapacityInput,
                                         uint32_t *viewConfigurationTypeCountOutput,
                                         XrViewConfigurationType *viewConfigurationTypes) override
    {
        auto const& system = _systems.find(systemId);
        if (system == _systems.end()) {
            return XR_ERROR_SYSTEM_INVALID;
        }

        std::vector<XrViewConfigurationType> types;
        system->second->GetViewConfigurations(types);

        *viewConfigurationTypeCountOutput = static_cast<uint32_t>(types.size());
        if (viewConfigurationTypeCapacityInput >= types.size()) {
            memcpy(viewConfigurationTypes, types.data(), sizeof(XrViewConfigurationType) * types.size());
            return XR_SUCCESS;
        } else if (viewConfigurationTypeCapacityInput > 0) {
            return XR_ERROR_SIZE_INSUFFICIENT;
        } else {
            return XR_SUCCESS;
        }
    }

    XrResult GetViewConfigurationProperties(XrSystemId systemId, XrViewConfigurationType viewConfigurationType,
                                            XrViewConfigurationProperties *configurationProperties) override
    {
        auto const& system = _systems.find(systemId);
        if (system == _systems.end()) {
            return XR_ERROR_SYSTEM_INVALID;
        }
        return system->second->GetPropertiesForViewConfiguration(viewConfigurationType, configurationProperties);
    }

    XrResult EnumerateViewConfigurationViews(XrSystemId systemId, XrViewConfigurationType viewConfigurationType,
                                             uint32_t viewCapacityInput, uint32_t *viewCountOutput,
                                             XrViewConfigurationView *outViews) override
    {
        auto const& system = _systems.find(systemId);
        if (system == _systems.end()) {
            return XR_ERROR_SYSTEM_INVALID;
        }

        std::vector<XrViewConfigurationView> views;
        system->second->GetViewsForViewConfiguration(viewConfigurationType, views);

        *viewCountOutput = static_cast<uint32_t>(views.size());
        if (viewCapacityInput >= views.size()) {
            memcpy(outViews, views.data(), sizeof(XrViewConfigurationView) * views.size());
            return XR_SUCCESS;
        } else if (viewCapacityInput > 0) {
            return XR_ERROR_SIZE_INSUFFICIENT;
        } else {
            return XR_SUCCESS;
        }
    }

    XrResult EnumerateEnvironmentBlendModes(XrSystemId systemId, XrViewConfigurationType viewConfigurationType,
                                            uint32_t environmentBlendModeCapacityInput,
                                            uint32_t *environmentBlendModeCountOutput,
                                            XrEnvironmentBlendMode *environmentBlendModes) override
    {
        auto const& system = _systems.find(systemId);
        if (system == _systems.end()) {
            return XR_ERROR_SYSTEM_INVALID;
        }

        std::vector<XrEnvironmentBlendMode> blendModes;
        system->second->GetBlendModesForViewConfiguration(viewConfigurationType, blendModes);

        *environmentBlendModeCountOutput = static_cast<uint32_t>(blendModes.size());
        if (environmentBlendModeCapacityInput >= blendModes.size()) {
            memcpy(environmentBlendModes, blendModes.data(), sizeof(XrEnvironmentBlendMode) * blendModes.size());
            return XR_SUCCESS;
        } else if (environmentBlendModeCapacityInput > 0) {
            return XR_ERROR_SIZE_INSUFFICIENT;
        } else {
            return XR_SUCCESS;
        }
    }

    XrResult CreateSession(const XrSessionCreateInfo *createInfo, XrSession *session) override {
        const XrGraphicsBindingMetalKHR *binding = static_cast<const XrGraphicsBindingMetalKHR *>(createInfo->next);

        if (binding == nullptr || binding->type != XR_TYPE_GRAPHICS_BINDING_METAL_KHR || binding->device == NULL) {
            return XR_ERROR_GRAPHICS_DEVICE_INVALID;
        }

        #if defined(OS_APPLE_IOS)
        *session = Session::RegisterInstance(CreateSession_iOS(GetHandle(), binding));
        _sessions.push_back(*session);
        return XR_SUCCESS;
        #elif defined(OS_APPLE_VISIONOS)
        *session = Session::RegisterInstance(CreateSession_visionOS(GetHandle(), binding));
        _sessions.push_back(*session);
        return XR_SUCCESS;
        #endif
        return XR_ERROR_RUNTIME_FAILURE;
    }

    XrResult DestroySession(XrSession session) override {
        erase(_sessions, session);
        return Session::DestroyInstance(session);
    }

    XrResult SuggestInteractionProfileBindings(const XrInteractionProfileSuggestedBinding* suggestedBindings) override {
        if (suggestedBindings->type != XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING) {
            return XR_ERROR_VALIDATION_FAILURE;
        }
        if (suggestedBindings->countSuggestedBindings > 0 && suggestedBindings->suggestedBindings == nullptr) {
            return XR_ERROR_VALIDATION_FAILURE;
        }
        for (int i = 0; i < suggestedBindings->countSuggestedBindings; ++i) {
            auto const& binding = suggestedBindings->suggestedBindings[i];
            auto action = Action::FromHandle(binding.action); // TODO: Error checking.
            auto actionSet = ActionSet::FromHandle(action->GetActionSet());
            if (actionSet->IsAttached()) {
                return XR_ERROR_ACTIONSETS_ALREADY_ATTACHED;
            }
        }

        _bindingsForInteractionProfiles[suggestedBindings->interactionProfile] = {
            suggestedBindings->suggestedBindings,
            suggestedBindings->suggestedBindings + suggestedBindings->countSuggestedBindings
        };

        for (XrSession sessionHandle : _sessions) {
            if (auto session = Session::FromHandle(sessionHandle)) {
                session->OnSuggestedActionBindingsChanged();
            }
        }

        char buffer[256];
        uint32_t len = 0;
        this->PathToString(suggestedBindings->interactionProfile, 256, &len, buffer);
        printf("Would have accepted suggested interaction profile bindings:\n");
        printf("  Interaction profile is %s\n", buffer);
        for (int i = 0; i < suggestedBindings->countSuggestedBindings; ++i) {
            XrActionSuggestedBinding const& binding = suggestedBindings->suggestedBindings[i];
            this->PathToString(binding.binding, 256, &len, buffer);
            printf("    Action %s maps to path %s\n", Action::FromHandle(binding.action)->GetName().c_str(), buffer);
        }
        return XR_SUCCESS;
    }

    void GetSuggestedInteractionProfiles(std::vector<XrPath> &interactionProfiles) override {
        interactionProfiles.clear();
        for (auto const& profileAndBinding : _bindingsForInteractionProfiles) {
            interactionProfiles.push_back(profileAndBinding.first);
        }
    }

    void GetSuggestedActionBindings(XrPath interactionProfile, std::vector<XrActionSuggestedBinding> &bindings) override {
        auto suggestedBindings = _bindingsForInteractionProfiles.find(interactionProfile);
        if (suggestedBindings != _bindingsForInteractionProfiles.end()) {
            bindings = suggestedBindings->second;
        } else {
            bindings.clear();
        }
    }

    XrResult CreateActionSet(const XrActionSetCreateInfo *createInfo, XrActionSet *actionSet) override {
        if (createInfo == nullptr) {
            return XR_ERROR_VALIDATION_FAILURE;
        }
        *actionSet = ActionSet::RegisterInstance(std::make_unique<ActionSet>(GetHandle(), *createInfo));
        _actionSets.push_back(*actionSet);
        return XR_SUCCESS;
    }

    XrResult DestroyActionSet(XrActionSet actionSet) override {
        erase(_actionSets, actionSet);
        return ActionSet::DestroyInstance(actionSet);
    }

#if defined(XR_USE_GRAPHICS_API_METAL)
    XrResult GetMetalGraphicsRequirementsKHR(XrSystemId systemId,
                                             XrGraphicsRequirementsMetalKHR *graphicsRequirements) override
    {
        auto const& system = _systems.find(systemId);
        if (system == _systems.end()) {
            return XR_ERROR_SYSTEM_INVALID;
        }

        return system->second->GetMetalGraphicsRequirementsKHR(graphicsRequirements);
    }
#endif

private:
    std::unordered_map<XrSystemId, std::unique_ptr<System>> _systems;
    std::vector<XrSession> _sessions;
    std::vector<XrActionSet> _actionSets;
    std::unordered_map<XrPath, std::vector<XrActionSuggestedBinding>> _bindingsForInteractionProfiles;
    std::unordered_map<std::string, XrPath> _pathAtoms;
    uint64_t _nextPathAtom = 1;
    uint64_t _nextSystemAtom = 1;
};

} // end anonymous namespace

std::unique_ptr<Instance> CreateInstance_Apple() {
    return std::make_unique<Instance_Apple>();
}
