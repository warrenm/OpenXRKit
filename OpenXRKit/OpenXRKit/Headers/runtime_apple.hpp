/*
** Copyright 2024-2025, Warren Moore
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

#pragma once

#include "openxr_api.h"
#include "runtime_api.hpp"

extern OPENXRKIT_PRIVATE std::unique_ptr<xr::Instance> CreateInstance_Apple(void);
extern OPENXRKIT_PRIVATE std::unique_ptr<xr::InteractionManager> CreateInteractionManager_Apple(XrInstance instance);

#if defined (OS_APPLE_IOS)
extern OPENXRKIT_PRIVATE std::unique_ptr<xr::System> CreateSystem_iOS(XrSystemId systemId);
extern OPENXRKIT_PRIVATE std::unique_ptr<xr::Session> CreateSession_iOS(XrInstance instance, const XrGraphicsBindingMetalKHR *binding);
#endif

#if defined (OS_APPLE_VISIONOS)
extern OPENXRKIT_PRIVATE std::unique_ptr<xr::System> CreateSystem_visionOS(XrSystemId systemId);
extern OPENXRKIT_PRIVATE std::unique_ptr<xr::Session> CreateSession_visionOS(XrInstance instance, const XrGraphicsBindingMetalKHR *binding);
#endif
