# OpenXRKit

## Overview

OpenXRKit is an experimental runtime for the OpenXR standard for mixed-reality applications on Apple platforms. It takes the form of a framework that can be embedded directly into iOS and visionOS apps, supplying a complete set of OpenXR capabilities, including support for game controllers and efficient compositing for mobile AR on iPhone and head-worn VR experiences on Apple Vision Pro. It abstracts over numerous system frameworks in order to provide an API that will be immediately familiar to any OpenXR application developer.

## Project Status

This package is experimental, incomplete, unstable, and not fit for production use. Because it is primarily designed to be used as an embedded framework rather than an installable dynamic library, it does not fulfill all of the requirements of an OpenXR runtime. Additionally, it only supports a single graphics API (Metal), which precludes it from being useful for easily porting existing applications. Nonetheless, it is proof that OpenXR can be implemented using public API on Apple platforms. Hopefully, it will be a useful point of reference among OpenXR application developers and the OpenXR standards body regarding support for Apple platforms.

## Framework Architecture

The framework is a self-contained runtime with no external dependencies other than Apple platform frameworks. Although the framework can be dynamically loaded by the official OpenXR loader if desired, this use case is not encouraged, because App Store limitations related to loading of dynamic libraries (using the `dlopen` API) preclude the loader from functioning as intended.

Instead, the framework can be embedded just like any other third-party framework or library. This is supported by the fact that the framework exports all of the symbols required of an OpenXR 1.0 core runtime. This linkage mechanism unfortunately precludes the use of layers, a powerful mechanism for augmenting and overriding runtime behavior.

Under the hood, the runtime is implemented in terms of various system APIs including Metal, ARKit, Core Animation (on iOS), Compositor Services (on visionOS), and the GameController framework. It interfaces with an application by implementing a non-ratified extension (`XR_KHR_metal_enable`) that allows binding to a Metal device and command queue (and optionally, a `CAMetalLayer`). In this way, the runtime can control frame pacing and presentation of content, while allowing the application to render whatever it desires.

The runtime abstracts over which system frameworks are used to implement various OpenXR features. For example, on iOS it uses ARKit to perform 6-DoF tracking of the device and presents composited content rendered with Metal using the Core Animation layer system. On visionOS, it uses platform-specific ARKit features and the Compositor Services framework to efficiently deliver stereo images to the system at up to 90 frames per second. In both instances, it gives applications significant control over how content is rendered.

## Basic Usage

Unlike most frameworks, you don't interact directly with the API vended by the framework. Instead, you use it indirectly through the OpenXR API provided by the OpenXR SDK. In this sense, using OpenXRKit is just like using any other OpenXR runtime, though without using the loader to discover the runtime's location on disk. You can enumerate available extensions, create an instance, and implement event polling and rendering using all of the usual OpenXR API calls.

Because OpenXR applications ordinarily run their own render loop at a cadence that differs from the event processing loop on the main application thread, it is **strongly encouraged** to run all code that interacts with OpenXR on a separate thread. This can be accomplished by subclassing `NSThread` and placing your OpenXR startup and render loop code in its `-main` method.

To render content that is to be composited with the runtime, you need to activate an extension corresponding to your graphics API. Since Metal is the only API currently supported by OpenXRKit, you will need to enable the `XR_KHR_metal_enable` extension[^1]. This will allow you to query the required Metal device family with the `xrGetMetalGraphicsRequirementsKHR` function and provide the Metal device, command queue, and layer to composite into with the `XrGraphicsBindingMetalKHR` struct.

[^1]: This is a placeholder extension has not been ratified by The Khronos Group and should be considered unofficial and experimental.
