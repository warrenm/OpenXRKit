/*
** Copyright 2024-2025, Warren Moore
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

#include "runtime_apple.hpp"

#if defined (OS_APPLE_IOS)

#import <ARKit/ARKit.h>
#import <Metal/Metal.h>
#import <QuartzCore/QuartzCore.h> // for CAMetalLayer and CADisplayLink

#include <mach/mach_time.h>
#include <sys/semaphore.h>

#include <deque>
#include <memory>
#include <vector>

#include "frame_queue.hpp"

namespace {

XrTime TimeAdvancedBySeconds(XrTime base, double duration) {
    return base + (int64_t)(duration * 1e9);
}

XrTime TimeFromTimeInterval(CFTimeInterval interval) {
    return TimeAdvancedBySeconds(0, interval);
}

CFTimeInterval TimeIntervalFromXrTime(XrTime time) {
    return time / (double)1e9;
}

XrDuration DurationFromSeconds(double duration) {
    return (int64_t)round(duration * 1e9);
}

class Clock {
public:
    Clock() {
        mach_timebase_info(&_timebase);
    }

    XrTime Now() {
        uint64_t absTime = mach_absolute_time();
        double nanos = double(absTime * uint64_t(_timebase.numer)) / double(_timebase.denom);
        return (XrTime)nanos;
    }

private:
    struct mach_timebase_info _timebase;
};

simd_float3x3 Float3x3FromCGAffineTransform(CGAffineTransform const& M) {
    // Ref. https://developer.apple.com/documentation/corefoundation/cgaffinetransform?language=objc
    // The transposition here is intentional: CGAffineTransform is set up to transform with the vector on the left, but
    // we adopt the opposite convention of multiplying with the matrix on the left (treating points as column vectors).
    simd_float3 c0 = simd_make_float3( M.a,  M.b, 0.0f);
    simd_float3 c1 = simd_make_float3( M.c,  M.d, 0.0f);
    simd_float3 c2 = simd_make_float3(M.tx, M.ty, 1.0f);
    return simd_matrix(c0, c1, c2);
}

void QueryProbableMainWindowInterfaceOrientation(void (^reponseHandler)(UIInterfaceOrientation)) {
    dispatch_async(dispatch_get_main_queue(), ^{
        UIInterfaceOrientation orientation = UIInterfaceOrientationUnknown;
        for (UIScene *scene in UIApplication.sharedApplication.connectedScenes) {
            if ([scene isKindOfClass:[UIWindowScene class]]) {
                UIWindowScene *windowScene = (UIWindowScene *)scene;
                orientation = windowScene.interfaceOrientation;
                break;
            }
        }
        reponseHandler(orientation);
    });
}

// Returns a transform matrix that can be pre-multiplied by an ARCamera transform to produce
// a camera transform that it always oriented "up" with respect to the user interface (i.e.,
// +Y points toward the top of the interface, +X to the right, and +Z toward the user).
simd_float4x4 CameraTransformForInterfaceOrientation(UIInterfaceOrientation orientation) {
    switch (orientation) {
        case UIInterfaceOrientationPortrait:
            return simd_matrix4x4(simd_quaternion(M_PI_2, simd_make_float3(0, 0, 1)));
        default:
            return matrix_identity_float4x4; // TODO: Support non-portrait orientations
    }
}

const XrPosef PoseIdentity = XrPosef {
    .position = XrVector3f { 0.0f, 0.0f, 0.0f },
    .orientation = XrQuaternionf { 0.0f, 0.0f, 0.0f, 1.0f }
};

bool DecomposeRigidTransform(const simd_float4x4 &M, XrPosef &pose) {
    simd_float4 T = M.columns[3];
    memcpy(&pose.position, &T, sizeof(XrVector3f));

    simd_float3x3 R = (simd_float3x3){
        (simd_float3){ M.columns[0].x, M.columns[0].y, M.columns[0].z },
        (simd_float3){ M.columns[1].x, M.columns[1].y, M.columns[1].z },
        (simd_float3){ M.columns[2].x, M.columns[2].y, M.columns[2].z },
    };
    simd_quatf q = simd_quaternion(R);
    memcpy(&pose.orientation, &q, sizeof(XrQuaternionf));
    return true;
}

simd_float4x4 MatrixFromPose(XrPosef const& pose) {
    simd_float4 t { 0.0f, 0.0f, 0.0f, 1.0f };
    memcpy(&t, &pose.position, sizeof(XrVector3f));
    simd_quatf q;
    memcpy(&q, &pose.orientation, sizeof(XrQuaternionf));
    simd_float4x4 M = simd_matrix4x4(q);
    M.columns[3] = t;
    return M;
}

XrPosef ComposePoseTransforms(XrPosef const& lhs, XrPosef const &rhs) {
    // TODO: Make this more efficient by exploiting known properties of poses
    simd_float4x4 M = simd_mul(MatrixFromPose(lhs), MatrixFromPose(rhs));
    XrPosef composite{};
    DecomposeRigidTransform(M, composite);
    return composite;
}

class PoseTimeSample {
public:
    PoseTimeSample() {}
    
    PoseTimeSample(XrPosef const& pose, XrTime time, XrSpaceLocationFlags locationFlags) :
        pose { pose },
        time { time },
        locationFlags { locationFlags }
    {}

    XrPosef pose = PoseIdentity;
    XrTime time = 0;
    XrSpaceLocationFlags locationFlags = 0;
};

class IARSessionEventListener {
public:
    virtual void SessionDidUpdateFrame(ARSession *session) = 0;
    virtual void CameraTrackingStateDidChange(ARCamera *camera) = 0;
};

class IDisplayLinkListener {
public:
    virtual void DisplayLinkDidFire(XrTime currentTime, XrTime presentationTime, XrDuration frameDuration) = 0;
};

}

@interface ARSessionManager : NSObject <ARSessionDelegate> {
    IARSessionEventListener *_eventListener;
}
@property (nonatomic, strong) ARSession *session;
@property (nonatomic, assign) BOOL shouldAttemptRelocalizationAfterInterruption;
@end

@implementation ARSessionManager

- (instancetype)init {
    if (self = [super init]) {
        _session = [ARSession new];
        _session.delegate = self;
    }
    return self;
}

- (void)setEventListener:(IARSessionEventListener *)listener {
    _eventListener = listener;
}

- (void)session:(ARSession *)session didUpdateFrame:(ARFrame *)frame {
    if (_eventListener) {
        _eventListener->SessionDidUpdateFrame(session);
    }
}

- (void)session:(ARSession *)session didAddAnchors:(NSArray<__kindof ARAnchor*>*)anchors {}

- (void)session:(ARSession *)session didUpdateAnchors:(NSArray<__kindof ARAnchor*>*)anchors {}

- (void)session:(ARSession *)session didRemoveAnchors:(NSArray<__kindof ARAnchor*>*)anchors {}

- (void)session:(ARSession *)session didFailWithError:(NSError *)error {}

- (void)session:(ARSession *)session cameraDidChangeTrackingState:(ARCamera *)camera {
    if (_eventListener) {
        _eventListener->CameraTrackingStateDidChange(camera);
    }
}

- (void)sessionWasInterrupted:(ARSession *)session {}

- (void)sessionInterruptionEnded:(ARSession *)session {}

- (BOOL)sessionShouldAttemptRelocalization:(ARSession *)session {
    return self.shouldAttemptRelocalizationAfterInterruption;
}

@end

@interface DisplayLinkManager : NSObject {
    IDisplayLinkListener *_listener;
}
@property (nonatomic, nullable, strong) CADisplayLink *displayLink;
@end

@implementation DisplayLinkManager

- (instancetype)init {
    if (self = [super init]) {
    }
    return self;
}

- (void)setListener:(IDisplayLinkListener *)listener {
    _listener = listener;
}

- (void)startDisplayLink {
    if (_displayLink) {
        [_displayLink invalidate];
    }
    _displayLink = [CADisplayLink displayLinkWithTarget:self selector:@selector(displayLinkDidFire:)];
    [_displayLink addToRunLoop:[NSRunLoop mainRunLoop] forMode:NSRunLoopCommonModes];
}

- (void)displayLinkDidFire:(CADisplayLink *)sender {
    if (_listener) {
        XrTime currentTime = TimeFromTimeInterval(sender.timestamp);
        XrDuration expectedFrameDuration = DurationFromSeconds(sender.duration);
        XrTime targetPresentationTime = TimeFromTimeInterval(sender.targetTimestamp);
        _listener->DisplayLinkDidFire(currentTime, targetPresentationTime, expectedFrameDuration);
    }
}

- (void)stopDisplayLink {
    if (_displayLink) {
        [_displayLink invalidate];
        _displayLink = nil;
    }
}

@end

@interface OpenXRKitFrameworkSigil : NSObject
@end

@implementation OpenXRKitFrameworkSigil
@end

namespace {

class ReferenceSpace_ARKit : public xr::Space {
public:
    ReferenceSpace_ARKit(XrSession session, const XrReferenceSpaceCreateInfo &createInfo) : xr::Space(session),
        _referenceSpaceType { createInfo.referenceSpaceType },
        _poseInReferenceSpace { createInfo.poseInReferenceSpace }
    {}

    XrResult Locate(XrSpace baseSpace, XrTime time, XrSpaceLocation *location) {
        if (location->type != XR_TYPE_SPACE_LOCATION) {
            return XR_ERROR_VALIDATION_FAILURE;
        }
        if (location->next != nullptr) {
            XrSpaceVelocity *velocity = reinterpret_cast<XrSpaceVelocity *>(location->next);
            if (velocity->type != XR_TYPE_SPACE_VELOCITY) {
                return XR_ERROR_VALIDATION_FAILURE;
            }
            // TODO: Support interpolated velocity inference?
            velocity->velocityFlags = 0;
            velocity->linearVelocity = {};
            velocity->angularVelocity = {};
        }
        // TODO: Factor in pose of base space?
        if (_poseTimeSamples.empty()) {
            location->locationFlags = 0;
            location->pose = _poseInReferenceSpace;
        } else {
            auto const& lastPose = _poseTimeSamples.back();
            location->locationFlags = lastPose.locationFlags;
            location->pose = ComposePoseTransforms(lastPose.pose, _poseInReferenceSpace);
            return XR_SUCCESS;
        }
        return XR_SUCCESS;
    }

    XrResult Update(ARFrame *frame) {
        //simd_float4x4 nativeCameraTransform = frame.camera.transform;
        simd_float4x4 cameraFrameAdjustment = simd_matrix4x4(simd_quaternion(M_PI_2, simd_make_float3(0, 0, 1)));
        simd_float4x4 cameraTransform = simd_mul(frame.camera.transform, cameraFrameAdjustment);

        XrTime time = TimeFromTimeInterval(frame.timestamp);

        DiscardStalePoseSamples(time);

        auto defaultFlags = XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT | XR_SPACE_LOCATION_POSITION_TRACKED_BIT |
                            XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_VALID_BIT;

        switch (_referenceSpaceType) {
            case XR_REFERENCE_SPACE_TYPE_VIEW: {
                XrPosef pose;
                DecomposeRigidTransform(cameraTransform, pose);
                _poseTimeSamples.emplace_back(pose, time, defaultFlags);
                return XR_SUCCESS;
            }
            case XR_REFERENCE_SPACE_TYPE_LOCAL: {
                XrPosef pose = PoseIdentity;
                //DecomposeRigidTransform(simd_inverse(cameraTransform), pose);
                _poseTimeSamples.emplace_back(pose, time, defaultFlags);
                return XR_SUCCESS;
            }
            default:
                return XR_ERROR_REFERENCE_SPACE_UNSUPPORTED;
        }
    }

private:
    void DiscardStalePoseSamples(XrTime referenceTime) {
        const NSTimeInterval stalenessDuration = 0.05;

        while (!_poseTimeSamples.empty()) {
            if (TimeAdvancedBySeconds(_poseTimeSamples.front().time, stalenessDuration) < referenceTime) {
                _poseTimeSamples.pop_front();
            } else {
                break;
            }
        }
    }

private:
    XrReferenceSpaceType _referenceSpaceType;
    XrPosef _poseInReferenceSpace;
    std::deque<PoseTimeSample> _poseTimeSamples;
};

class ActionSpace_ARKit : public xr::Space {
public:
    ActionSpace_ARKit(XrSession session, const XrActionSpaceCreateInfo &createInfo) : 
        xr::Space(session),
        _poseInActionSpace { createInfo.poseInActionSpace },
        _action { createInfo.action },
        _subactionPath { createInfo.subactionPath }
    {}

    XrResult Locate(XrSpace baseSpace, XrTime time, XrSpaceLocation *location) {
        return XR_ERROR_RUNTIME_FAILURE;
    }

    void Update(ARFrame *frame) {
    }

private:
    XrPosef _poseInActionSpace;
    XrAction _action;
    XrPath _subactionPath;
};

class SwapchainImage_Metal {
public:
    explicit SwapchainImage_Metal(id<MTLTexture> texture) {
        _texture = texture;
        _textureInFlightSemaphore = dispatch_semaphore_create(1);
    }

    id<MTLTexture> GetTexture() {
        return _texture;
    }

    // Waits for this resource to become available, subject to the provided timeout.
    // Returns `true` if the resource was acquired, `false` if timeout happened instead.
    bool Wait(XrDuration timeout) {
        auto waited = dispatch_semaphore_wait(_textureInFlightSemaphore, dispatch_time(DISPATCH_TIME_NOW, timeout));
        return waited == 0;
    }

    void Enqueue() {
        dispatch_semaphore_signal(_textureInFlightSemaphore);
    }

private:
    id<MTLTexture> _texture;
    dispatch_semaphore_t _textureInFlightSemaphore;
};

class Swapchain_Metal : public xr::Swapchain {
public:
    Swapchain_Metal(XrSession session, XrSwapchainCreateInfo const& createInfo, id<MTLDevice> device, uint32_t imageCount) :
        xr::Swapchain(session),
        _queueMutex PTHREAD_MUTEX_INITIALIZER,
        _device { device },
        _createFlags { createInfo.createFlags },
        _usageFlags { createInfo.usageFlags },
        _pixelFormat { static_cast<MTLPixelFormat>(createInfo.format) },
        _sampleCount { createInfo.sampleCount },
        _width { createInfo.width },
        _height { createInfo.height },
        _arrayLength { createInfo.arraySize },
        _mipLevelCount { createInfo.mipCount }
    {
        AllocateImages(imageCount);
    }

    ~Swapchain_Metal() = default;

    XrResult EnumerateSwapchainImages(uint32_t imageCapacityInput,
                                      uint32_t *imageCountOutput, XrSwapchainImageBaseHeader *images)
    {
        if (images != nullptr && images->type != XR_TYPE_SWAPCHAIN_IMAGE_METAL_KHR) {
            return XR_ERROR_VALIDATION_FAILURE;
        }
        XrSwapchainImageMetalKHR *platformImages = reinterpret_cast<XrSwapchainImageMetalKHR *>(images);

        uint32_t imageCount = (uint32_t)_images.size();

        *imageCountOutput = imageCount;
        if (imageCapacityInput >= imageCount) {
            for (int i = 0; i < imageCount; ++i) {
                XrSwapchainImageMetalKHR *image = &platformImages[i];
                if (image->type != XR_TYPE_SWAPCHAIN_IMAGE_METAL_KHR) {
                    return XR_ERROR_VALIDATION_FAILURE;
                }
                image->type = XrStructureType::XR_TYPE_SWAPCHAIN_IMAGE_METAL_KHR;
                image->next = nullptr;
                image->texture = _images[i].GetTexture();
            }
            return XR_SUCCESS;
        } else if (imageCapacityInput > 0) {
            return XR_ERROR_SIZE_INSUFFICIENT;
        } else {
            return XR_SUCCESS;
        }
    }

    XrResult AcquireSwapchainImage(const XrSwapchainImageAcquireInfo *acquireInfo, uint32_t *index) {
        (void)acquireInfo; // Not used.
        XrResult result = XR_SUCCESS;
        pthread_mutex_lock(&_queueMutex);
        if (_acquirableIndices.size() > 0) {
            auto firstAcquirableIndex = _acquirableIndices.front();
            _acquirableIndices.pop_front();
            *index = firstAcquirableIndex;
            _acquiredIndex = firstAcquirableIndex;
        } else {
            result = XR_ERROR_CALL_ORDER_INVALID;
        }
        pthread_mutex_unlock(&_queueMutex);
        return result;
    }

    XrResult WaitSwapchainImage(const XrSwapchainImageWaitInfo *waitInfo) {
        (void)waitInfo;
        pthread_mutex_lock(&_queueMutex);
        XrResult result = XR_SUCCESS;
        if (_acquiredIndex.has_value()) {
            auto &imageToWait = _images[_acquiredIndex.value()];
            if (imageToWait.Wait(waitInfo->timeout)) {
                _waitedIndex = _acquiredIndex.value();
                _acquiredIndex.reset();
            } else {
                result = XR_TIMEOUT_EXPIRED;
            }
        } else {
            result = XR_ERROR_CALL_ORDER_INVALID;
        }
        pthread_mutex_unlock(&_queueMutex);
        return result;
    }

    XrResult ReleaseSwapchainImage(const XrSwapchainImageReleaseInfo* releaseInfo) {
        (void)releaseInfo;
        pthread_mutex_lock(&_queueMutex);
        XrResult result = XR_SUCCESS;
        if (_waitedIndex.has_value()) {
            _releasedIndex = _waitedIndex.value();
            _acquirableIndices.push_back(_releasedIndex.value());
            _waitedIndex.reset();
        } else {
            result = XR_ERROR_CALL_ORDER_INVALID;
        }
        pthread_mutex_unlock(&_queueMutex);
        return result;
    }

    SwapchainImage_Metal *DequeueReleasedImage() {
        SwapchainImage_Metal *image = nullptr;
        pthread_mutex_lock(&_queueMutex);
        if (_releasedIndex.has_value()) {
            image = &_images[_releasedIndex.value()];
            _releasedIndex.reset();
        }
        pthread_mutex_unlock(&_queueMutex);
        return image;
    }

    void EnqueueAcquirableImage(SwapchainImage_Metal *image) {
        // We don't take the queue lock here because it must be possible for
        // a WaitSwapchainImage call to unblock independent of queue lock state
        image->Enqueue();
    }

private:
    XrResult AllocateImages(uint32_t imageCount) {
        MTLTextureDescriptor *textureDescriptor = [MTLTextureDescriptor new];
        textureDescriptor.textureType = (_arrayLength == 1) ? MTLTextureType2D : MTLTextureType2DArray;
        textureDescriptor.pixelFormat = _pixelFormat;
        textureDescriptor.width = _width;
        textureDescriptor.height = _height;
        textureDescriptor.depth = 1;
        textureDescriptor.arrayLength = _arrayLength;
        textureDescriptor.mipmapLevelCount = _mipLevelCount;
        textureDescriptor.storageMode = MTLStorageModePrivate;

        textureDescriptor.usage = MTLTextureUsageRenderTarget;
        if (((_usageFlags & XR_SWAPCHAIN_USAGE_UNORDERED_ACCESS_BIT) != 0) ||
            ((_usageFlags & XR_SWAPCHAIN_USAGE_SAMPLED_BIT) != 0)) {
            textureDescriptor.usage |= MTLTextureUsageShaderRead;
        }
        if ((_usageFlags & XR_SWAPCHAIN_USAGE_MUTABLE_FORMAT_BIT) != 0) {
            textureDescriptor.usage |= MTLTextureUsagePixelFormatView;
        }

        for (uint32_t i = 0; i < imageCount; ++i) {
            id<MTLTexture> texture = [_device newTextureWithDescriptor:textureDescriptor];
            _images.emplace_back(texture);
            _acquirableIndices.push_back(i);
        }

        return XR_SUCCESS;
    }

    std::vector<SwapchainImage_Metal> _images;
    std::deque<uint32_t> _acquirableIndices;
    std::optional<uint32_t> _acquiredIndex;
    std::optional<uint32_t> _waitedIndex;
    std::optional<uint32_t> _releasedIndex;
    pthread_mutex_t _queueMutex;
    id<MTLDevice> _device;
    Clock _clock;
    XrSwapchainCreateFlags _createFlags;
    XrSwapchainUsageFlags _usageFlags;
    MTLPixelFormat _pixelFormat;
    uint32_t _sampleCount;
    uint32_t _width;
    uint32_t _height;
    uint32_t _arrayLength;
    uint32_t _mipLevelCount;
};

class Compositor_CAMetalLayer {
public:
    explicit Compositor_CAMetalLayer(XrGraphicsBindingMetalKHR const* binding) :
        _layer { binding->metalLayer },
        _device { binding->device },
        _commandQueue { binding->commandQueue }
    {
        NSDictionary *cacheAttrs = @{ (__bridge NSString *)kCVMetalTextureUsage : @(MTLTextureUsageShaderRead)};
        CVReturn result = CVMetalTextureCacheCreate(kCFAllocatorDefault,
                                                    (__bridge CFDictionaryRef)cacheAttrs,
                                                    _device,
                                                    NULL,
                                                    &_textureCache);
        (void)result;
        CreatePipelines();
    }

    bool CreatePipelines() {
        NSError *error = nil;

        NSBundle *frameworkBundle = [NSBundle bundleForClass:[OpenXRKitFrameworkSigil class]];
        id<MTLLibrary> library = [_device newDefaultLibraryWithBundle:frameworkBundle error:&error];
        if (library == nil) {
            NSLog(@"[Warning] Failed to create default library for bundle (%@): %@", frameworkBundle, error.localizedDescription);
            return false;
        }

        id<MTLFunction> vertexFunction = [library newFunctionWithName:@"compositor_vertex"];
        id<MTLFunction> layerFragmentFunction = [library newFunctionWithName:@"compositor_fragment_rgb"];
        id<MTLFunction> passthroughFragmentFunction = [library newFunctionWithName:@"compositor_fragment_ycbcr"];

        MTLRenderPipelineDescriptor *renderPipelineDescriptor = [MTLRenderPipelineDescriptor new];
        renderPipelineDescriptor.vertexFunction = vertexFunction;

        renderPipelineDescriptor.colorAttachments[0].pixelFormat = _layer.pixelFormat;
        renderPipelineDescriptor.rasterSampleCount = 1;
        renderPipelineDescriptor.maxVertexAmplificationCount = 1;

        // Passthrough render state
        renderPipelineDescriptor.fragmentFunction = passthroughFragmentFunction;
        _passthroughRenderPipelineState = [_device newRenderPipelineStateWithDescriptor:renderPipelineDescriptor error:&error];
        if (_passthroughRenderPipelineState == nil) {
            NSLog(@"[Warning] Failed to create render pipeline state: %@", error.localizedDescription);
            return false;
        }

        // Opaque render state
        renderPipelineDescriptor.fragmentFunction = layerFragmentFunction;
        renderPipelineDescriptor.colorAttachments[0].blendingEnabled = NO;
        _opaqueRenderPipelineState = [_device newRenderPipelineStateWithDescriptor:renderPipelineDescriptor error:&error];
        if (_opaqueRenderPipelineState == nil) {
            NSLog(@"[Warning] Failed to create render pipeline state: %@", error.localizedDescription);
            return false;
        }

        renderPipelineDescriptor.colorAttachments[0].blendingEnabled = YES;
        renderPipelineDescriptor.colorAttachments[0].rgbBlendOperation = MTLBlendOperationAdd;
        renderPipelineDescriptor.colorAttachments[0].alphaBlendOperation = MTLBlendOperationAdd;

        // Alpha blending render state
        renderPipelineDescriptor.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorOne;
        renderPipelineDescriptor.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorOne;
        renderPipelineDescriptor.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        renderPipelineDescriptor.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        _alphaBlendRenderPipelineState = [_device newRenderPipelineStateWithDescriptor:renderPipelineDescriptor error:&error];
        if (_alphaBlendRenderPipelineState == nil) {
            NSLog(@"[Warning] Failed to create render pipeline state: %@", error.localizedDescription);
            return false;
        }

        // Additive blending render state
        renderPipelineDescriptor.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorOne;
        renderPipelineDescriptor.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorOne;
        renderPipelineDescriptor.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOne;
        renderPipelineDescriptor.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOne;
        _additiveRenderPipelineState = [_device newRenderPipelineStateWithDescriptor:renderPipelineDescriptor error:&error];
        if (_additiveRenderPipelineState == nil) {
            NSLog(@"[Warning] Failed to create render pipeline state: %@", error.localizedDescription);
            return false;
        }

        return true;
    }

    XrResult CompositeLayersAndPresent(XrFrameEndInfo const* frameEndInfo, ARFrame *arFrame) {
        id <MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
        id<CAMetalDrawable> drawable = [_layer nextDrawable];

        if (drawable == nil) {
            return XR_ERROR_RUNTIME_FAILURE;
        }

        CFTimeInterval presentationTime = TimeIntervalFromXrTime(frameEndInfo->displayTime);

        bool willRenderPassthrough = (arFrame != nil) && (arFrame.capturedImage != nil);

        MTLRenderPassDescriptor *renderPassDescriptor = [MTLRenderPassDescriptor renderPassDescriptor];
        renderPassDescriptor.colorAttachments[0].texture = drawable.texture;
        renderPassDescriptor.colorAttachments[0].loadAction = willRenderPassthrough ? MTLLoadActionDontCare : MTLLoadActionClear;
        renderPassDescriptor.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1);
        renderPassDescriptor.colorAttachments[0].storeAction = MTLStoreActionStore;

        XrResult result = XR_SUCCESS;

        id<MTLRenderCommandEncoder> renderEncoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];

        if (willRenderPassthrough) {
            MTLViewport viewport { 0, 0, _layer.drawableSize.width, _layer.drawableSize.height, 0, 1 };
            [renderEncoder setViewport:viewport];
            DrawPassthroughLayer(commandBuffer, renderEncoder, viewport, arFrame);
        }

        switch (frameEndInfo->environmentBlendMode) {
            case XR_ENVIRONMENT_BLEND_MODE_OPAQUE:
                [renderEncoder setRenderPipelineState:_opaqueRenderPipelineState];
                break;
            case XR_ENVIRONMENT_BLEND_MODE_ADDITIVE:
                [renderEncoder setRenderPipelineState:_additiveRenderPipelineState];
                break;
            case XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND:
                [renderEncoder setRenderPipelineState:_alphaBlendRenderPipelineState];
                break;
            default:
                break;
        }

        for (int i = 0; i < frameEndInfo->layerCount; ++i) {
            XrCompositionLayerBaseHeader const* layerBase = frameEndInfo->layers[i];
            if (layerBase == nullptr || layerBase->type != XR_TYPE_COMPOSITION_LAYER_PROJECTION) {
                result = XR_ERROR_LAYER_INVALID;
                continue;
            }

            auto projectionLayer = reinterpret_cast<XrCompositionLayerProjection const *>(layerBase);
            if (projectionLayer->space == XR_NULL_HANDLE) {
                result = XR_ERROR_LAYER_INVALID;
                continue;
            }

            // TODO: Honor layer subimage viewport

            auto const& view = projectionLayer->views[0];
            auto const& subimage = view.subImage;
            auto swapchain = static_cast<Swapchain_Metal *>(xr::Swapchain::FromHandle(subimage.swapchain));
            auto layerImage = swapchain->DequeueReleasedImage();
            [renderEncoder setFragmentTexture:layerImage->GetTexture() atIndex:0];
            simd_float3x3 displayTransform = matrix_identity_float3x3;
            [renderEncoder setVertexBytes:&displayTransform length:sizeof(displayTransform) atIndex:0];
            [renderEncoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];

            [commandBuffer addCompletedHandler:^(id<MTLCommandBuffer>) {
                swapchain->EnqueueAcquirableImage(layerImage);
            }];
        }

        [renderEncoder endEncoding];

        [commandBuffer addScheduledHandler:^(id<MTLCommandBuffer>) {
            [drawable presentAtTime:presentationTime];
        }];

        [commandBuffer commit];
        return result;
    }

    void SetInterfaceOrientation(UIInterfaceOrientation interfaceOrientation) {
        _interfaceOrientation = interfaceOrientation;
    }

    bool DrawPassthroughLayer(id<MTLCommandBuffer> commandBuffer, id<MTLRenderCommandEncoder> renderEncoder, MTLViewport viewport, ARFrame *frame) {
        CVPixelBufferRef pixelBuffer = frame.capturedImage;

        CVMetalTextureRef yMetalTexture = NULL, cbcrMetalTexture = NULL;
        size_t lumaPlaneWidth = CVPixelBufferGetWidthOfPlane(pixelBuffer, 0);
        size_t lumaPlaneHeight = CVPixelBufferGetHeightOfPlane(pixelBuffer, 0);
        CVReturn result;
        result = CVMetalTextureCacheCreateTextureFromImage(kCFAllocatorDefault,
                                                           _textureCache,
                                                           (CVImageBufferRef)pixelBuffer,
                                                           NULL,
                                                           MTLPixelFormatR8Unorm,
                                                           lumaPlaneWidth,
                                                           lumaPlaneHeight,
                                                           0,
                                                           &yMetalTexture);
        if (result != kCVReturnSuccess) {
            return false;
        }

        size_t chromaPlaneWidth = CVPixelBufferGetWidthOfPlane(pixelBuffer, 1);
        size_t chromaPlaneHeight = CVPixelBufferGetHeightOfPlane(pixelBuffer, 1);
        result = CVMetalTextureCacheCreateTextureFromImage(kCFAllocatorDefault,
                                                           _textureCache,
                                                           (CVImageBufferRef)pixelBuffer,
                                                           NULL,
                                                           MTLPixelFormatRG8Unorm,
                                                           chromaPlaneWidth,
                                                           chromaPlaneHeight,
                                                           1,
                                                           &cbcrMetalTexture);
        if (result != kCVReturnSuccess) {
            return false;
        }

        id<MTLTexture> yTexture = CVMetalTextureGetTexture(yMetalTexture);
        id<MTLTexture> cbcrTexture = CVMetalTextureGetTexture(cbcrMetalTexture);

        [renderEncoder setRenderPipelineState:_passthroughRenderPipelineState];
        [renderEncoder setFragmentTexture:yTexture atIndex:0];
        [renderEncoder setFragmentTexture:cbcrTexture atIndex:1];

        CGSize viewportSize = CGSizeMake(viewport.width, viewport.height);
        CGAffineTransform viewToImageTransform = [frame displayTransformForOrientation:_interfaceOrientation
                                                                          viewportSize:viewportSize];
        CGAffineTransform imageToViewTransform = CGAffineTransformInvert(viewToImageTransform);
        simd_float3x3 displayTransform = Float3x3FromCGAffineTransform(imageToViewTransform);
        [renderEncoder setVertexBytes:&displayTransform length:sizeof(displayTransform) atIndex:0];

        [renderEncoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];

        [commandBuffer addCompletedHandler:^(id<MTLCommandBuffer>) {
            if (yMetalTexture) {
                CFRelease(yMetalTexture);
            }
            if (cbcrMetalTexture) {
                CFRelease(cbcrMetalTexture);
            }
        }];

        return true;
    }

private:
    CAMetalLayer *_layer;
    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    id<MTLRenderPipelineState> _passthroughRenderPipelineState;
    id<MTLRenderPipelineState> _opaqueRenderPipelineState;
    id<MTLRenderPipelineState> _additiveRenderPipelineState;
    id<MTLRenderPipelineState> _alphaBlendRenderPipelineState;
    CVMetalTextureCacheRef _textureCache;
    UIInterfaceOrientation _interfaceOrientation;
};

class Session_iOS_ARKit : public xr::Session, public IARSessionEventListener, public IDisplayLinkListener {
public:
    Session_iOS_ARKit(XrInstance instance, const XrGraphicsBindingMetalKHR *binding) :
        xr::Session(instance),
        _device { binding->device },
        _commandQueue { binding->commandQueue },
        _metalLayer { binding->metalLayer }
    {
        _interactionManager = CreateInteractionManager_Apple(instance);

        _sessionManager = [ARSessionManager new];
        _sessionManager.eventListener = this;

        _displayLinkManager = [DisplayLinkManager new];
        _displayLinkManager.listener = this;

        _compositor = std::make_unique<Compositor_CAMetalLayer>(binding);

        PrepareToStart();
    }

    ~Session_iOS_ARKit() {
        [_sessionManager.session pause];
        
        for (auto it : _spaces) {
            xr::Space::DestroyInstance(it);
        }
        for (auto it : _swapchains) {
            xr::Swapchain::DestroyInstance(it);
        }
    }

    XrResult EnumerateReferenceSpaces(uint32_t spaceCapacityInput,
                                      uint32_t *spaceCountOutput, XrReferenceSpaceType *spaces) override
    {
        std::array<XrReferenceSpaceType, 2> supportedSpaceTypes {
            XR_REFERENCE_SPACE_TYPE_VIEW,
            XR_REFERENCE_SPACE_TYPE_LOCAL
        };
        *spaceCountOutput = (uint32_t)supportedSpaceTypes.size();
        if (spaceCapacityInput >= supportedSpaceTypes.size()) {
            memcpy(spaces, supportedSpaceTypes.data(), sizeof(XrReferenceSpaceType) * supportedSpaceTypes.size());
            return XR_SUCCESS;
        } else if (spaceCapacityInput > 0) {
            return XR_ERROR_SIZE_INSUFFICIENT;
        } else {
            return XR_SUCCESS;
        }
    }

    XrResult CreateReferenceSpace(const XrReferenceSpaceCreateInfo *createInfo, XrSpace *space) override {
        if (createInfo == nullptr) {
            return XR_ERROR_VALIDATION_FAILURE;
        }

        if (!(createInfo->referenceSpaceType == XR_REFERENCE_SPACE_TYPE_VIEW ||
              createInfo->referenceSpaceType == XR_REFERENCE_SPACE_TYPE_LOCAL))
        {
            return XR_ERROR_REFERENCE_SPACE_UNSUPPORTED;
        }

        *space = xr::Space::RegisterInstance(std::make_unique<ReferenceSpace_ARKit>(GetHandle(), *createInfo));
        _spaces.push_back(*space);
        return XR_SUCCESS;
    }

    XrResult GetReferenceSpaceBoundsRect(XrReferenceSpaceType referenceSpaceType, XrExtent2Df *bounds) override {
        memset(bounds, 0, sizeof(XrExtent2Df));
        return XR_SPACE_BOUNDS_UNAVAILABLE;
    }

    XrResult CreateActionSpace(const XrActionSpaceCreateInfo *createInfo, XrSpace *space) override {
        if (createInfo == nullptr) {
            return XR_ERROR_VALIDATION_FAILURE;
        }
        *space = xr::Space::RegisterInstance(std::make_unique<ActionSpace_ARKit>(GetHandle(), *createInfo));
        return XR_SUCCESS;
    }

    XrResult LocateSpace(XrSpace space, XrSpace baseSpace, XrTime time, XrSpaceLocation *location) override {
        auto impl = xr::Space::FromHandle(space);
        if (impl == nullptr) {
            return XR_ERROR_HANDLE_INVALID;
        }
        if (auto referenceSpace = dynamic_cast<ReferenceSpace_ARKit *>(impl)) {
            return referenceSpace->Locate(baseSpace, time, location);
        }
        return XR_SUCCESS;
    }

    XrResult DestroySpace(XrSpace space) override {
        return xr::Space::DestroyInstance(space);
    }

    XrResult EnumerateSwapchainFormats(uint32_t formatCapacityInput,
                                       uint32_t *formatCountOutput, int64_t *formats) override
    {
        constexpr uint32_t supportedFormatCount = 5;
        int64_t supportedFormats[supportedFormatCount] = {
            static_cast<int64_t>(MTLPixelFormatBGRA8Unorm_sRGB),
            static_cast<int64_t>(MTLPixelFormatBGRA8Unorm),
            static_cast<int64_t>(MTLPixelFormatRGBA8Unorm_sRGB),
            static_cast<int64_t>(MTLPixelFormatRGBA8Unorm),
            static_cast<int64_t>(MTLPixelFormatDepth32Float), // TODO: Only return when depth layer extension is active?
        };

        *formatCountOutput = supportedFormatCount;

        if (formatCapacityInput >= supportedFormatCount) {
            memcpy(formats, supportedFormats, sizeof(int64_t) * supportedFormatCount);
        } else if (formatCapacityInput > 0) {
            return XR_ERROR_SIZE_INSUFFICIENT;
        }

        return XR_SUCCESS;
    }

    XrResult CreateSwapchain(const XrSwapchainCreateInfo *createInfo, XrSwapchain *swapchain) override {
        uint32_t imageCount = 3;
        *swapchain = xr::Swapchain::RegisterInstance(std::make_unique<Swapchain_Metal>(GetHandle(), *createInfo,
                                                                                       _device, imageCount));
        return XR_SUCCESS;
    }

    XrResult DestroySwapchain(XrSwapchain swapchain) override {
        return xr::Swapchain::DestroyInstance(swapchain);
    }

    XrResult BeginSession(const XrSessionBeginInfo *beginInfo) override {
        if (_sessionState != XrSessionState::XR_SESSION_STATE_READY) {
            return XR_ERROR_SESSION_NOT_READY;
        }
        // This initial transition to "synchronized" signals to the app that this session is running
        TransitionToState(XrSessionState::XR_SESSION_STATE_SYNCHRONIZED);
        return XR_SUCCESS;
    }

    bool SessionStateIsRunningState(XrSessionState state) {
        return (state == XrSessionState::XR_SESSION_STATE_SYNCHRONIZED) ||
               (state == XrSessionState::XR_SESSION_STATE_VISIBLE) ||
               (state == XrSessionState::XR_SESSION_STATE_FOCUSED);
    }

    XrResult EndSession() override {
        if (!SessionStateIsRunningState(_sessionState)) {
            return XR_ERROR_SESSION_NOT_RUNNING;
        }
        TransitionToState(XrSessionState::XR_SESSION_STATE_EXITING);
        return XR_SUCCESS;
    }

    XrResult RequestExitSession() override {
        if (!SessionStateIsRunningState(_sessionState)) {
            return XR_ERROR_SESSION_NOT_RUNNING;
        }
        TransitionToState(XrSessionState::XR_SESSION_STATE_EXITING);
        return XR_SUCCESS;
    }

    XrResult WaitFrame(const XrFrameWaitInfo *frameWaitInfo, XrFrameState *frameState) override {
        if (!SessionStateIsRunningState(_sessionState)) {
            return XR_ERROR_SESSION_NOT_RUNNING;
        }
        if (frameWaitInfo != nullptr && frameWaitInfo->type != XR_TYPE_FRAME_WAIT_INFO) {
            return XR_ERROR_VALIDATION_FAILURE;
        }
        if (frameState->type != XR_TYPE_FRAME_STATE) {
            return XR_ERROR_VALIDATION_FAILURE;
        }

        auto waitedFrame = _frameQueue.WaitFrame();
        if (waitedFrame.has_value()) {
            XrBool32 shouldRender = (_sessionState == XR_SESSION_STATE_VISIBLE) || 
                                    (_sessionState == XR_SESSION_STATE_FOCUSED);
            frameState->predictedDisplayTime = waitedFrame->expectedPresentationTime;
            frameState->predictedDisplayPeriod = waitedFrame->expectedDuration;
            frameState->shouldRender = shouldRender;
            return XR_SUCCESS;
        } else {
            return XR_ERROR_RUNTIME_FAILURE;
        }
    }

    XrResult BeginFrame(const XrFrameBeginInfo *frameBeginInfo) override {
        (void)frameBeginInfo; // Only for extensibility

        if (!SessionStateIsRunningState(_sessionState)) {
            return XR_ERROR_SESSION_NOT_RUNNING;
        }

        QueryProbableMainWindowInterfaceOrientation(^(UIInterfaceOrientation interfaceOrientation) {
            this->_interfaceOrientation = interfaceOrientation; // race!
        });

        bool discarded;
        auto frameToBegin = _frameQueue.BeginFrame(discarded);
        if (frameToBegin.has_value()) {
            return discarded ? XR_FRAME_DISCARDED : XR_SUCCESS;
        }

        return XR_ERROR_CALL_ORDER_INVALID;
    }

    XrResult EndFrame(const XrFrameEndInfo *frameEndInfo) override {
        if (!SessionStateIsRunningState(_sessionState)) {
            return XR_ERROR_SESSION_NOT_RUNNING;
        }

        auto endedFrame = _frameQueue.EndFrame();
        if (endedFrame.has_value()) {
            ARFrame *arFrame = _sessionManager.session.currentFrame;
            _compositor->SetInterfaceOrientation(_interfaceOrientation); // TODO: Avoid this by passing the relevant resources and properties instead of the ARFrame itself
            _compositor->CompositeLayersAndPresent(frameEndInfo, arFrame);
            XrDuration framePresentationBudget = endedFrame->expectedPresentationTime - _clock.Now();
            if (framePresentationBudget < 0) {
                //printf("[Info] Frame %03llu missed presentation deadline and would not have been composited!\n", endedFrame->frameIndex);
            }
        }
        return XR_SUCCESS;
    }

    UIInterfaceOrientation GetInterfaceOrientation() {
        return _interfaceOrientation;
    }

    CGSize GetViewportSize() const {
        return _metalLayer.drawableSize;
    }

    XrResult LocateViews(const XrViewLocateInfo *viewLocateInfo, XrViewState *viewState,
                         uint32_t viewCapacityInput, uint32_t *viewCountOutput, XrView *views) override
    {
        if (viewLocateInfo == nullptr || viewLocateInfo->type != XR_TYPE_VIEW_LOCATE_INFO) {
            return XR_ERROR_VALIDATION_FAILURE;
        }

        xr::Space *space = xr::Space::FromHandle(viewLocateInfo->space);
        if (space == nullptr) {
            return XR_ERROR_HANDLE_INVALID;
        }

        if (viewState == nullptr || viewState->type != XR_TYPE_VIEW_STATE) {
            return XR_ERROR_VALIDATION_FAILURE;
        }

        viewState->viewStateFlags = 0;

        ARFrame *frame = _sessionManager.session.currentFrame;
        ARCamera *camera = frame.camera;
        if (frame != nil && camera != nil) {
            if (camera.trackingState == ARTrackingStateNormal) {
                viewState->viewStateFlags = XR_VIEW_STATE_POSITION_VALID_BIT | XR_VIEW_STATE_POSITION_TRACKED_BIT |
                                            XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_ORIENTATION_TRACKED_BIT;
            } else if (camera.trackingState == ARTrackingStateLimited) {
                viewState->viewStateFlags = XR_VIEW_STATE_POSITION_VALID_BIT | XR_VIEW_STATE_ORIENTATION_VALID_BIT;
            }
        }

        uint32_t viewCount = 1;
        if (viewCapacityInput >= viewCount) {
            *viewCountOutput = viewCount;

            XrView &view = views[0];

            if (camera != nil) {
                // TODO: Express this in terms of an anchor or other reference space instead of doing these redundant calculations
                simd_float4x4 cameraFrameAdjustment = CameraTransformForInterfaceOrientation(GetInterfaceOrientation());
                simd_float4x4 cameraTransform = simd_mul(frame.camera.transform, cameraFrameAdjustment);
                DecomposeRigidTransform(cameraTransform, view.pose);

                simd_float4x4 projectionMatrix = [camera projectionMatrixForOrientation:GetInterfaceOrientation()
                                                                           viewportSize:GetViewportSize()
                                                                                  zNear:1
                                                                                   zFar:100];
                float xx = projectionMatrix.columns[0][0], yy = projectionMatrix.columns[1][1];
                float aspectRatio = yy / xx;
                float fovHalfY = atan(1 / yy);
                float fovHalfX = atan(aspectRatio / yy);
                view.fov = XrFovf { -fovHalfX, fovHalfX, fovHalfY, -fovHalfY };
            } else {
                view.pose = PoseIdentity;
                // It's impossible to properly estimate the field of view without a
                // camera frame, so we just hardcode a 1:2 aspect ratio.
                view.fov = XrFovf { -M_PI_4 * 0.5f, M_PI_4 * 0.5f, M_PI_4, -M_PI_4 };
            }
        }

        return XR_SUCCESS;
    }

    XrResult AttachSessionActionSets(const XrSessionActionSetsAttachInfo *attachInfo) override {
        auto instance = xr::Instance::FromHandle(GetInstance());
        if (instance == nullptr) {
            return XR_ERROR_INSTANCE_LOST;
        }
        if (_hasAttachedActionSets) {
            return XR_ERROR_ACTIONSETS_ALREADY_ATTACHED;
        }
        _hasAttachedActionSets = true;
        return _interactionManager->AttachSessionActionSets(attachInfo);
    }

    XrResult GetCurrentInteractionProfile(XrPath topLevelUserPath, XrInteractionProfileState *interactionProfile) override {
        return _interactionManager->GetCurrentInteractionProfile(topLevelUserPath, interactionProfile);
    }

    void OnSuggestedActionBindingsChanged() override {
        _needsInteractionProfileUpdate = true;
    }

    void SelectInteractionProfileIfNeeded() {
        if (_needsInteractionProfileUpdate) {
            _needsInteractionProfileUpdate = false;
            auto instance = xr::Instance::FromHandle(GetInstance());
            if (instance == nullptr) {
                return;
            }

            std::vector<XrPath> suggestedProfiles;
            instance->GetSuggestedInteractionProfiles(suggestedProfiles);

            std::vector<XrPath> supportedProfiles;
            _interactionManager->GetSupportedInteractionProfiles(supportedProfiles);

            bool didBindProfile = false;
            for (auto const &profile : supportedProfiles) {
                auto matchedProfile = std::find(begin(supportedProfiles), end(supportedProfiles), profile);
                if (matchedProfile != end(supportedProfiles)) {
                    _interactionManager->BindInteractionProfile(*matchedProfile);
                    didBindProfile = true;
                    break;
                }
            }
            if (didBindProfile) {
                xr::Event event {
                    .type = XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED,
                    .session = GetHandle()
                };
                instance->PostEvent(event);
            }
        }
    }

    XrResult GetActionStateBoolean(const XrActionStateGetInfo *getInfo, XrActionStateBoolean *state) override {
        return _interactionManager->GetActionStateBoolean(getInfo, state);
    }

    XrResult GetActionStateFloat(const XrActionStateGetInfo *getInfo, XrActionStateFloat *state) override {
        return _interactionManager->GetActionStateFloat(getInfo, state);
    }

    XrResult GetActionStateVector2f(const XrActionStateGetInfo *getInfo, XrActionStateVector2f *state) override {
        return _interactionManager->GetActionStateVector2f(getInfo, state);
    }

    XrResult GetActionStatePose(const XrActionStateGetInfo *getInfo, XrActionStatePose *state) override {
        return _interactionManager->GetActionStatePose(getInfo, state);
    }

    XrResult SyncActions(const XrActionsSyncInfo *syncInfo) override {
        SelectInteractionProfileIfNeeded();
        return _interactionManager->SyncActions(syncInfo);
    }

    XrResult EnumerateBoundSourcesForAction(const XrBoundSourcesForActionEnumerateInfo *enumerateInfo,
                                            uint32_t sourceCapacityInput, uint32_t *sourceCountOutput, XrPath *sources) override
    {
        return _interactionManager->EnumerateBoundSourcesForAction(enumerateInfo, sourceCapacityInput, 
                                                                   sourceCountOutput, sources);
    }

    XrResult GetInputSourceLocalizedName(const XrInputSourceLocalizedNameGetInfo *getInfo, uint32_t bufferCapacityInput,
                                         uint32_t *bufferCountOutput, char *buffer) override
    {
        return _interactionManager->GetInputSourceLocalizedName(getInfo, bufferCapacityInput, bufferCountOutput, buffer);
    }

    XrResult ApplyHapticFeedback(const XrHapticActionInfo *hapticActionInfo,
                                 const XrHapticBaseHeader *hapticFeedback) override
    {
        return _interactionManager->ApplyHapticFeedback(hapticActionInfo, hapticFeedback);
    }

    XrResult StopHapticFeedback(const XrHapticActionInfo *hapticActionInfo) override {
        return _interactionManager->StopHapticFeedback(hapticActionInfo);
    }

    void SessionDidUpdateFrame(ARSession *session) override {
        ARFrame *frame = session.currentFrame;
        if (frame == nil) {
            return;
        }
        for (auto it : _spaces) {
            auto space = xr::Space::FromHandle(it);
            if (auto referenceSpace = dynamic_cast<ReferenceSpace_ARKit *>(space)) {
                referenceSpace->Update(frame);
            }
        }
    }

    void CameraTrackingStateDidChange(ARCamera *camera) override {
        if (SessionStateIsRunningState(_sessionState)) {
            switch (camera.trackingState) {
                case ARTrackingStateNotAvailable:
                    TransitionToState(XR_SESSION_STATE_SYNCHRONIZED);
                    break;
                case ARTrackingStateLimited:
                    // We are presumably visible but have limited tracking
                    TransitionToState(XR_SESSION_STATE_VISIBLE);
                    break;
                case ARTrackingStateNormal:
                    // Tracking is nominal, so we map that to "focused"
                    TransitionToState(XR_SESSION_STATE_FOCUSED);
                    break;
            }
        } else {
            // Session state is only dependent on tracking state when the session is running; otherwise we don't care.
        }
    }

    void DisplayLinkDidFire(XrTime currentTime, XrTime presentationTime, XrDuration frameDuration) override {
        (void)_frameQueue.EnqueueNewFrame(0, currentTime, frameDuration, presentationTime);
    }

private:

    void PrepareToStart() {
        if ([ARWorldTrackingConfiguration isSupported]) {
            ARConfiguration *configuration = [ARWorldTrackingConfiguration new];
            [_sessionManager.session runWithConfiguration:configuration];
        } else {
            //printf("[Warning] This platform does not support the necessary ARKit configuration\n");
        }

        [_displayLinkManager startDisplayLink];

        TransitionToState(XrSessionState::XR_SESSION_STATE_READY);
    }

    void TransitionToState(XrSessionState nextState) {
        if (_sessionState == nextState) { // No change, early-out.
            return;
        }
        if (auto instanceImpl = xr::Instance::FromHandle(GetInstance())) {
            switch (nextState) {
                // TODO: Event-specific data
                case XrSessionState::XR_SESSION_STATE_IDLE: break;
                case XrSessionState::XR_SESSION_STATE_READY: break;
                case XrSessionState::XR_SESSION_STATE_SYNCHRONIZED: break;
                case XrSessionState::XR_SESSION_STATE_VISIBLE: break;
                case XrSessionState::XR_SESSION_STATE_FOCUSED: break;
                case XrSessionState::XR_SESSION_STATE_STOPPING: break;
                case XrSessionState::XR_SESSION_STATE_LOSS_PENDING: break;
                case XrSessionState::XR_SESSION_STATE_EXITING: break;
                default: break;
            }
            xr::Event event {
                .type = XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED,
                .session = GetHandle(),
                .sessionState = nextState,
                .time = _clock.Now()
            };
            _sessionState = nextState;
            instanceImpl->PostEvent(event);
        }
    }

private:
    std::vector<XrSpace> _spaces;
    std::vector<XrSwapchain> _swapchains;
    std::unique_ptr<Compositor_CAMetalLayer> _compositor;
    std::unique_ptr<xr::InteractionManager> _interactionManager;
    FrameQueue<uint32_t> _frameQueue;
    Clock _clock;
    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    CAMetalLayer *_metalLayer;
    UIInterfaceOrientation _interfaceOrientation = UIInterfaceOrientationPortrait;
    ARSessionManager *_sessionManager = nil;
    DisplayLinkManager *_displayLinkManager = nil;
    XrSessionState _sessionState = XR_SESSION_STATE_IDLE;
    bool _needsInteractionProfileUpdate = true;
    bool _hasAttachedActionSets = false;
};

} // end anonymous namespace

std::unique_ptr<xr::Session> CreateSession_iOS(XrInstance instance, const XrGraphicsBindingMetalKHR *binding) {
    return std::make_unique<Session_iOS_ARKit>(instance, binding);
}

#endif
