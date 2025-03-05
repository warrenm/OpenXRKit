/*
** Copyright 2024-2025, Warren Moore
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

#pragma once

#include <objc/objc.h> // For id

#define XR_USE_PLATFORM_APPLE
#define XR_USE_GRAPHICS_API_METAL
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_loader_negotiation.h>

#ifdef __clang__
#define OPENXRKIT_EXPORT  __attribute__((visibility("default")))
#define OPENXRKIT_PRIVATE __attribute__((visibility("hidden")))
#else
#define OPENXRKIT_EXPORT
#define OPENXRKIT_PRIVATE
#endif

#ifdef __cplusplus
extern "C" {
#endif

OPENXRKIT_EXPORT XrResult xrNegotiateLoaderRuntimeInterface(const XrNegotiateLoaderInfo *loaderInfo,
                                                            XrNegotiateRuntimeRequest *runtimeRequest);

#ifdef __cplusplus
}
#endif
