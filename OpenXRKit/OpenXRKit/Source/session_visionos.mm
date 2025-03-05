/*
** Copyright 2024-2025, Warren Moore
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

#include "runtime_apple.hpp"

#if defined (OS_APPLE_VISIONOS)

#import <ARKit/ARKit.h>
#import <Metal/Metal.h>
#import <CompositorServices/CompositorServices.h>

#include <mach/mach_time.h>
#include <sys/semaphore.h>

#include <deque>
#include <memory>
#include <vector>

#include "frame_queue.hpp"

using namespace xr;

#define VALIDATE_TYPE_NONNULL(TYPE, value)          \
do {                                                \
    if (value == nullptr || value->type != TYPE) {  \
        return XR_ERROR_VALIDATION_FAILURE;         \
    }                                               \
} while(0);

#define VALIDATE_TYPE_NULLABLE(TYPE, value)         \
do {                                                \
    if (value != nullptr && value->type != TYPE) {  \
        return XR_ERROR_VALIDATION_FAILURE;         \
    }                                               \
} while(0);

#define VALIDATE_NONNULL(value)                     \
do {                                                \
    if (value == nullptr) {                         \
        return XR_ERROR_VALIDATION_FAILURE;         \
    }                                               \
} while(0);

@interface OpenXRKitFrameworkSigil : NSObject
@end

@implementation OpenXRKitFrameworkSigil
@end

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

class Clock final {
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

const XrPosef PoseIdentity = XrPosef {
    .position = XrVector3f { 0.0f, 0.0f, 0.0f },
    .orientation = XrQuaternionf { 0.0f, 0.0f, 0.0f, 1.0f }
};

int32_t GetXrHandJointIndexForHandSkeletonJointIndex(uint64_t jointIndex) {
    switch (jointIndex) {
        case ar_hand_skeleton_joint_name_wrist: return XR_HAND_JOINT_WRIST_EXT;
        case ar_hand_skeleton_joint_name_thumb_knuckle: return XR_HAND_JOINT_THUMB_METACARPAL_EXT;
        case ar_hand_skeleton_joint_name_thumb_intermediate_base: return XR_HAND_JOINT_THUMB_PROXIMAL_EXT;
        case ar_hand_skeleton_joint_name_thumb_intermediate_tip: return XR_HAND_JOINT_THUMB_DISTAL_EXT;
        case ar_hand_skeleton_joint_name_thumb_tip: return XR_HAND_JOINT_THUMB_TIP_EXT;
        case ar_hand_skeleton_joint_name_index_finger_metacarpal: return XR_HAND_JOINT_INDEX_METACARPAL_EXT;
        case ar_hand_skeleton_joint_name_index_finger_knuckle: return XR_HAND_JOINT_INDEX_PROXIMAL_EXT;
        case ar_hand_skeleton_joint_name_index_finger_intermediate_base: return XR_HAND_JOINT_INDEX_INTERMEDIATE_EXT;
        case ar_hand_skeleton_joint_name_index_finger_intermediate_tip: return XR_HAND_JOINT_INDEX_DISTAL_EXT;
        case ar_hand_skeleton_joint_name_index_finger_tip: return XR_HAND_JOINT_INDEX_TIP_EXT ;
        case ar_hand_skeleton_joint_name_middle_finger_metacarpal: return XR_HAND_JOINT_MIDDLE_METACARPAL_EXT;
        case ar_hand_skeleton_joint_name_middle_finger_knuckle: return XR_HAND_JOINT_MIDDLE_PROXIMAL_EXT;
        case ar_hand_skeleton_joint_name_middle_finger_intermediate_base: return XR_HAND_JOINT_MIDDLE_INTERMEDIATE_EXT;
        case ar_hand_skeleton_joint_name_middle_finger_intermediate_tip: return XR_HAND_JOINT_MIDDLE_DISTAL_EXT;
        case ar_hand_skeleton_joint_name_middle_finger_tip: return XR_HAND_JOINT_MIDDLE_TIP_EXT;
        case ar_hand_skeleton_joint_name_ring_finger_metacarpal: return XR_HAND_JOINT_RING_METACARPAL_EXT;
        case ar_hand_skeleton_joint_name_ring_finger_knuckle: return XR_HAND_JOINT_RING_PROXIMAL_EXT;
        case ar_hand_skeleton_joint_name_ring_finger_intermediate_base: return XR_HAND_JOINT_RING_INTERMEDIATE_EXT;
        case ar_hand_skeleton_joint_name_ring_finger_intermediate_tip: return XR_HAND_JOINT_RING_DISTAL_EXT;
        case ar_hand_skeleton_joint_name_ring_finger_tip: return XR_HAND_JOINT_RING_TIP_EXT;
        case ar_hand_skeleton_joint_name_little_finger_metacarpal: return XR_HAND_JOINT_LITTLE_METACARPAL_EXT;
        case ar_hand_skeleton_joint_name_little_finger_knuckle: return XR_HAND_JOINT_LITTLE_PROXIMAL_EXT;
        case ar_hand_skeleton_joint_name_little_finger_intermediate_base: return XR_HAND_JOINT_LITTLE_INTERMEDIATE_EXT;
        case ar_hand_skeleton_joint_name_little_finger_intermediate_tip: return XR_HAND_JOINT_LITTLE_DISTAL_EXT;
        case ar_hand_skeleton_joint_name_little_finger_tip: return XR_HAND_JOINT_LITTLE_TIP_EXT;
        case ar_hand_skeleton_joint_name_forearm_wrist:
            [[fallthrough]]; // No OpenXR equivalent
        case ar_hand_skeleton_joint_name_forearm_arm:
            [[fallthrough]]; // No OpenXR equivalent
        default:
            return -1;
    }
}

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

XrPosef LerpPoseTransforms(XrPosef const& lhs, XrPosef const &rhs, float fraction) {
    simd_float3 posA, posB;
    memcpy(&posA, &lhs.position, sizeof(XrVector3f));
    memcpy(&posB, &rhs.position, sizeof(XrVector3f));
    simd_float3 pos = simd_lerp(posA, posB, fraction);
    simd_quatf rotA, rotB;
    memcpy(&rotA, &lhs.orientation, sizeof(XrQuaternionf));
    memcpy(&rotB, &rhs.orientation, sizeof(XrQuaternionf));
    simd_quatf rot = simd_slerp(rotA, rotB, fraction);
    XrPosef out;
    memcpy(&out.position, &pos, sizeof(XrVector3f));
    memcpy(&out.orientation, &rot, sizeof(XrQuaternionf));
    return out;
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

class ReferenceSpace_ARKit_visionOS : public Space {
public:
    ReferenceSpace_ARKit_visionOS(XrSession session, const XrReferenceSpaceCreateInfo &createInfo) : Space(session),
        _referenceSpaceType { createInfo.referenceSpaceType },
        _poseInReferenceSpace { createInfo.poseInReferenceSpace }
    {}

    XrResult Locate(XrSpace baseSpace, XrTime time, XrSpaceLocation *location) {
        VALIDATE_TYPE_NONNULL(XR_TYPE_SPACE_LOCATION, location);
        if (location->next != nullptr) {
            XrSpaceVelocity *velocity = reinterpret_cast<XrSpaceVelocity *>(location->next);
            VALIDATE_TYPE_NONNULL(XR_TYPE_SPACE_VELOCITY, velocity);
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
            // TODO: Interpolate/extrapolate based on `time`
            auto const& lastPose = _poseTimeSamples.back();
            location->locationFlags = lastPose.locationFlags;
            location->pose = ComposePoseTransforms(_poseInReferenceSpace, lastPose.pose);
            return XR_SUCCESS;
        }
        return XR_SUCCESS;
    }

    XrResult Update(XrTime now, XrPosef viewFromDevicePose, XrPosef deviceFromOriginPose, XrSpaceLocationFlags flags) {
        DiscardStalePoseSamples(now);

        switch (_referenceSpaceType) {
            case XR_REFERENCE_SPACE_TYPE_VIEW: {
                XrPosef localPose = ComposePoseTransforms(viewFromDevicePose, deviceFromOriginPose);
                _poseTimeSamples.emplace_back(localPose, now, flags);
                return XR_SUCCESS;
            }
            case XR_REFERENCE_SPACE_TYPE_LOCAL: {
                XrPosef pose = PoseIdentity;
                _poseTimeSamples.emplace_back(pose, now, flags);
                return XR_SUCCESS;
            }
            case XR_REFERENCE_SPACE_TYPE_STAGE: {
                XrPosef pose = PoseIdentity;
                _poseTimeSamples.emplace_back(pose, now, flags);
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
    std::deque<PoseTimeSample> _poseTimeSamples;
    XrPosef _poseInReferenceSpace;
    XrReferenceSpaceType _referenceSpaceType;
};

class ActionSpace_ARKit_visionOS : public Space {
public:
    ActionSpace_ARKit_visionOS(XrSession session, const XrActionSpaceCreateInfo &createInfo) :
        Space(session),
        _poseInActionSpace { createInfo.poseInActionSpace },
        _action { createInfo.action },
        _subactionPath { createInfo.subactionPath }
    {
        auto sessionImpl = Session::FromHandle(session);
        auto instance = sessionImpl->GetInstance();
        xrStringToPath(instance, "/user/hand/left", &_userHandLeftPath);
        xrStringToPath(instance, "/user/hand/right", &_userHandRightPath);
    }

    XrResult Locate(XrSpace baseSpace, XrTime time, XrSpaceLocation *location) {
        //XrSpaceLocation baseSpaceLocation;
        //XrResult baseSpaceResult = xrLocateSpace(baseSpace, XR_NULL_HANDLE, time, &baseSpaceLocation);
        //if (!XR_SUCCEEDED(baseSpaceResult)) {
        //    return baseSpaceResult;
        //}
        // TODO Factor in base space pose
        if (!_poseTimeSamples.empty()) {
            // TODO: Interpolate among samples based on time
            auto const& lastPoseSample = _poseTimeSamples.back();
            location->locationFlags = lastPoseSample.locationFlags;
            location->pose = lastPoseSample.pose;
            return XR_SUCCESS;
        } else {
            location->locationFlags = 0;
            location->pose = PoseIdentity;
            return XR_SUCCESS;
        }
    }

    void Update(XrTime time, XrPosef leftHandPose, XrPosef rightHandPose, XrSpaceLocationFlags flags) {
        DiscardStalePoseSamples(time);

        PoseTimeSample sample;
        sample.time = time;
        sample.locationFlags = flags;
        if (_subactionPath == _userHandLeftPath) {
            sample.pose = leftHandPose;
        } else {
            sample.pose = rightHandPose;
        }
        _poseTimeSamples.push_back(sample);
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

    std::deque<PoseTimeSample> _poseTimeSamples;
    XrPosef _poseInActionSpace;
    XrAction _action;
    XrPath _subactionPath;
    XrPath _userHandLeftPath = XR_NULL_PATH;
    XrPath _userHandRightPath = XR_NULL_PATH;
};

struct HandTrackerTimeSample {
    XrTime time;
    std::array<PoseTimeSample, XR_HAND_JOINT_COUNT_EXT> jointPoses;
};

class HandTracker_ARKit_visionOS : public HandTrackerEXT {
public:
    HandTracker_ARKit_visionOS(XrSession session, XrHandEXT hand, XrHandJointSetEXT handJointSet) :
        HandTrackerEXT(session, hand, handJointSet)
    {
        //_handAnchor = ar_hand_anchor_create();
    }

    //ar_hand_anchor_t GetAnchor() const {
    //    return _handAnchor;
    //}

    XrResult Locate(XrSpace baseSpace, XrTime time, XrHandJointLocationsEXT *location) {
        VALIDATE_TYPE_NONNULL(XR_TYPE_HAND_JOINT_LOCATIONS_EXT, location);
        if (location->jointCount != XR_HAND_JOINT_COUNT_EXT) {
            return XR_ERROR_VALIDATION_FAILURE;
        }
        if (_jointPoseTimeSamples.empty()) {
            location->isActive = false;
            return XR_SUCCESS; // TODO: Invalidate everything
        }
        auto const& lastSample = _jointPoseTimeSamples.back();
        location->isActive = true;
        for (uint32_t jointIndex = XR_HAND_JOINT_WRIST_EXT; jointIndex < XR_HAND_JOINT_COUNT_EXT; ++jointIndex) {
            location->jointLocations[jointIndex].radius = 0.015f;
            location->jointLocations[jointIndex].locationFlags = lastSample.jointPoses[jointIndex].locationFlags;
            location->jointLocations[jointIndex].pose = lastSample.jointPoses[jointIndex].pose;
        }
        return XR_SUCCESS;
    }

    void AddJointPoseTimeSample(HandTrackerTimeSample sample) {
        _jointPoseTimeSamples.push_back(std::move(sample));

        // TODO: Make retirement time-based rather than sample count based
        while (_jointPoseTimeSamples.size() > 10) {
            _jointPoseTimeSamples.pop_front();
        }
    }

private:
    //ar_hand_anchor_t _handAnchor;
    std::deque<HandTrackerTimeSample> _jointPoseTimeSamples;
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

    void Dequeue() {
        _inFlight = true;
    }

    bool IsInFlight() const {
        return _inFlight;
    }

    void Enqueue() {
        _inFlight = false;
        dispatch_semaphore_signal(_textureInFlightSemaphore);
    }

private:
    id<MTLTexture> _texture;
    dispatch_semaphore_t _textureInFlightSemaphore;
    bool _inFlight;
};

class Swapchain_Metal : public Swapchain {
public:
    Swapchain_Metal(XrSession session, XrSwapchainCreateInfo const& createInfo, id<MTLDevice> device, uint32_t imageCount) :
        Swapchain(session),
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
        VALIDATE_NONNULL(imageCountOutput);
        VALIDATE_TYPE_NULLABLE(XR_TYPE_SWAPCHAIN_IMAGE_METAL_KHR, images);
        XrSwapchainImageMetalKHR *platformImages = reinterpret_cast<XrSwapchainImageMetalKHR *>(images);

        uint32_t imageCount = (uint32_t)_images.size();

        *imageCountOutput = imageCount;
        if (imageCapacityInput >= imageCount) {
            for (int i = 0; i < imageCount; ++i) {
                XrSwapchainImageMetalKHR *image = &platformImages[i];
                VALIDATE_TYPE_NONNULL(XR_TYPE_SWAPCHAIN_IMAGE_METAL_KHR, image);
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
            } else if (!imageToWait.IsInFlight()) {
                imageToWait.Enqueue();
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
            image->Dequeue();
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
        textureDescriptor.storageMode = MTLStorageModeShared;
        if (((_usageFlags & XR_SWAPCHAIN_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT) != 0)) {
            textureDescriptor.storageMode = MTLStorageModePrivate;
        }

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

class Compositor_CP {
public:
    explicit Compositor_CP(XrGraphicsBindingMetalKHR const* binding) :
        _device { binding->device },
        _commandQueue { binding->commandQueue }
    {
        cp_layer_renderer_t layerRenderer = binding->drawableProvider;
        cp_layer_renderer_configuration_t rendererConfiguration = cp_layer_renderer_get_configuration(layerRenderer);
        _colorPixelFormat = cp_layer_renderer_configuration_get_color_format(rendererConfiguration);
        _depthPixelFormat = cp_layer_renderer_configuration_get_depth_format(rendererConfiguration);
        _rendererLayout = cp_layer_renderer_configuration_get_layout(rendererConfiguration);
        _foveationEnabled = cp_layer_renderer_configuration_get_foveation_enabled(rendererConfiguration);

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
        //id<MTLFunction> passthroughFragmentFunction = [library newFunctionWithName:@"compositor_fragment_ycbcr"];

        MTLRenderPipelineDescriptor *renderPipelineDescriptor = [MTLRenderPipelineDescriptor new];
        renderPipelineDescriptor.vertexFunction = vertexFunction;

        renderPipelineDescriptor.colorAttachments[0].pixelFormat = _colorPixelFormat;
        renderPipelineDescriptor.rasterSampleCount = 1;
        renderPipelineDescriptor.maxVertexAmplificationCount = 1; // TODO: Set to two for layered stereo

        renderPipelineDescriptor.depthAttachmentPixelFormat = _depthPixelFormat;

        // Passthrough render state
        //renderPipelineDescriptor.fragmentFunction = passthroughFragmentFunction;
        //_passthroughRenderPipelineState = [_device newRenderPipelineStateWithDescriptor:renderPipelineDescriptor error:&error];
        //if (_passthroughRenderPipelineState == nil) {
        //    NSLog(@"[Warning] Failed to create render pipeline state: %@", error.localizedDescription);
        //    return false;
        //}

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

    XrResult CompositeLayersAndPresent(XrFrameEndInfo const *frameEndInfo, cp_frame_t frame, cp_drawable_t drawable) {
        id <MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];

        //CFTimeInterval presentationTime = TimeIntervalFromXrTime(frameEndInfo->displayTime);

        size_t viewCount = cp_drawable_get_view_count(drawable);
        //size_t textureCount = cp_drawable_get_texture_count(drawable);

        XrResult result = XR_SUCCESS;

        for (size_t viewIndex = 0; viewIndex < viewCount; ++viewIndex) {
            size_t textureIndex = viewIndex;

            //cp_view_t view = cp_drawable_get_view(drawable, viewIndex);

            MTLRenderPassDescriptor *renderPass = CreateRenderPassDescriptor(drawable, textureIndex);

            id<MTLRenderCommandEncoder> renderEncoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPass];

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

            SwapchainImage_Metal *depthImage = nullptr;

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

                auto const& view = projectionLayer->views[viewIndex];
                auto const& subimage = view.subImage;
                auto swapchain = static_cast<Swapchain_Metal *>(Swapchain::FromHandle(subimage.swapchain));
                auto layerImage = swapchain->DequeueReleasedImage();

                Swapchain_Metal *depthSwapchain = nullptr;

                auto nextBase = static_cast<const XrBaseInStructure *>(view.next);
                if (nextBase && nextBase->type == XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR) {
                    auto depthInfo = static_cast<const XrCompositionLayerDepthInfoKHR *>(view.next);
                    auto depthSubimage = depthInfo->subImage;
                    depthSwapchain = static_cast<Swapchain_Metal *>(Swapchain::FromHandle(depthSubimage.swapchain));
                    depthImage = depthSwapchain->DequeueReleasedImage();
                }

                [renderEncoder setFragmentTexture:layerImage->GetTexture() atIndex:0];
                simd_float3x3 displayTransform = matrix_identity_float3x3;
                [renderEncoder setVertexBytes:&displayTransform length:sizeof(displayTransform) atIndex:0];
                [renderEncoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];

                [commandBuffer addCompletedHandler:^(id<MTLCommandBuffer>) {
                    swapchain->EnqueueAcquirableImage(layerImage);
                    if (depthSwapchain && depthImage) {
                        depthSwapchain->EnqueueAcquirableImage(depthImage);
                    }
                }];
            }

            [renderEncoder endEncoding];

            // HACKS: Completely replace depth buffer with previously-rendered projected depth
            if (depthImage) {
                id <MTLBlitCommandEncoder> depthBlitEncoder = [commandBuffer blitCommandEncoder];
                [depthBlitEncoder copyFromTexture:depthImage->GetTexture() toTexture:renderPass.depthAttachment.texture];
                [depthBlitEncoder endEncoding];
            }
        }

        cp_drawable_encode_present(drawable, commandBuffer);

        [commandBuffer commit];
        return result;
    }

private:
    MTLRenderPassDescriptor *CreateRenderPassDescriptor(cp_drawable_t drawable, size_t textureIndex) {
        MTLRenderPassDescriptor *renderPassDescriptor = [MTLRenderPassDescriptor renderPassDescriptor];
        renderPassDescriptor.colorAttachments[0].texture = cp_drawable_get_color_texture(drawable, textureIndex);
        renderPassDescriptor.colorAttachments[0].loadAction = MTLLoadActionClear;

        renderPassDescriptor.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1);

        renderPassDescriptor.colorAttachments[0].storeAction = MTLStoreActionStore;

        renderPassDescriptor.depthAttachment.texture = cp_drawable_get_depth_texture(drawable, textureIndex);
        renderPassDescriptor.depthAttachment.loadAction = MTLLoadActionClear;
        renderPassDescriptor.depthAttachment.clearDepth = 0.0;
        renderPassDescriptor.depthAttachment.storeAction = MTLStoreActionStore;

        if (_rendererLayout == cp_layer_renderer_layout_layered) {
            renderPassDescriptor.renderTargetArrayLength = cp_drawable_get_view_count(drawable);
        } else {
            renderPassDescriptor.renderTargetArrayLength = 1;
        }

        if (_foveationEnabled) {
            renderPassDescriptor.rasterizationRateMap = cp_drawable_get_rasterization_rate_map(drawable, textureIndex);
        }

        return renderPassDescriptor;
    }

    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    id<MTLRenderPipelineState> _opaqueRenderPipelineState;
    id<MTLRenderPipelineState> _additiveRenderPipelineState;
    id<MTLRenderPipelineState> _alphaBlendRenderPipelineState;
    MTLPixelFormat _colorPixelFormat;
    MTLPixelFormat _depthPixelFormat;
    cp_layer_renderer_layout _rendererLayout;
    bool _foveationEnabled;
};

class Session_visionOS_ARKit : public Session {
public:
    Session_visionOS_ARKit(XrInstance instance, const XrGraphicsBindingMetalKHR *binding) :
        Session { instance },
        _device { binding->device },
        _commandQueue { binding->commandQueue }
    {
        assert(_device != nil);

        if (_commandQueue == nil) {
            _commandQueue = [_device newCommandQueue];
        }

        _arSession = ar_session_create();

        _interactionManager = CreateInteractionManager_Apple(instance);

        if ([binding->drawableProvider isKindOfClass:[CP_OBJECT_NAME(cp_layer_renderer) class]]) {
            _layerRenderer = (cp_layer_renderer_t)binding->drawableProvider;
            _compositor = std::make_unique<Compositor_CP>(binding);
        } /*else if ([binding->metalLayer isKindOfClass:[CAMetalLayer class]]) {
            _compositor = std::make_unique<Compositor_CAMetalLayer>(binding);
        }*/

        PrepareToStart();
    }

    ~Session_visionOS_ARKit() {
        ar_session_stop(_arSession);

        for (auto it : _spaces) {
            Space::DestroyInstance(it);
        }
        for (auto it : _handTrackers) {
            HandTrackerEXT::DestroyInstance(it);
        }
        for (auto it : _swapchains) {
            Swapchain::DestroyInstance(it);
        }
    }

    XrResult EnumerateReferenceSpaces(uint32_t spaceCapacityInput,
                                      uint32_t *spaceCountOutput, XrReferenceSpaceType *spaces) override
    {
        std::array<XrReferenceSpaceType, 3> supportedSpaceTypes {
            XR_REFERENCE_SPACE_TYPE_VIEW,
            XR_REFERENCE_SPACE_TYPE_LOCAL,
            XR_REFERENCE_SPACE_TYPE_STAGE,
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
        VALIDATE_TYPE_NONNULL(XR_TYPE_REFERENCE_SPACE_CREATE_INFO, createInfo);

        if (!(createInfo->referenceSpaceType == XR_REFERENCE_SPACE_TYPE_VIEW ||
              createInfo->referenceSpaceType == XR_REFERENCE_SPACE_TYPE_LOCAL ||
              createInfo->referenceSpaceType == XR_REFERENCE_SPACE_TYPE_STAGE))
        {
            return XR_ERROR_REFERENCE_SPACE_UNSUPPORTED;
        }

        *space = Space::RegisterInstance(std::make_unique<ReferenceSpace_ARKit_visionOS>(GetHandle(), *createInfo));
        _spaces.push_back(*space);
        return XR_SUCCESS;
    }

    XrResult GetReferenceSpaceBoundsRect(XrReferenceSpaceType referenceSpaceType, XrExtent2Df *bounds) override {
        // 7.1) Not all systems or spaces may support boundaries. If a runtime is unable to provide bounds for
        // a given space, XR_SPACE_BOUNDS_UNAVAILABLE will be returned and all fields of bounds will be set to 0.
        memset(bounds, 0, sizeof(XrExtent2Df));
        return XR_SPACE_BOUNDS_UNAVAILABLE;
    }

    XrResult CreateActionSpace(const XrActionSpaceCreateInfo *createInfo, XrSpace *space) override {
        VALIDATE_TYPE_NONNULL(XR_TYPE_ACTION_SPACE_CREATE_INFO, createInfo);
        XrAction action = createInfo->action;
        auto actionImpl = Action::FromHandle(action);
        if (actionImpl == nullptr) {
            return XR_ERROR_HANDLE_INVALID;
        }
        if (actionImpl->GetActionType() != XR_ACTION_TYPE_POSE_INPUT) {
            return XR_ERROR_ACTION_TYPE_MISMATCH;
        }
        if (createInfo->subactionPath != XR_NULL_PATH) {
            auto const &subactionPaths = actionImpl->GetSubactionPaths();
            if (std::find(begin(subactionPaths), end(subactionPaths), createInfo->subactionPath) == end(subactionPaths)) {
                return XR_ERROR_PATH_UNSUPPORTED;
            }
        }
        *space = Space::RegisterInstance(std::make_unique<ActionSpace_ARKit_visionOS>(GetHandle(), *createInfo));
        return XR_SUCCESS;
    }

    XrResult LocateSpace(XrSpace space, XrSpace baseSpace, XrTime time, XrSpaceLocation *location) override {
        auto impl = Space::FromHandle(space);
        if (impl == nullptr) {
            return XR_ERROR_HANDLE_INVALID;
        }
        if (auto referenceSpace = dynamic_cast<ReferenceSpace_ARKit_visionOS *>(impl)) {
            return referenceSpace->Locate(baseSpace, time, location);
        }
        if (auto actionSpace = dynamic_cast<ActionSpace_ARKit_visionOS *>(impl)) {
            return actionSpace->Locate(baseSpace, time, location);
        }
        return XR_ERROR_HANDLE_INVALID;
    }

    XrResult DestroySpace(XrSpace space) override {
        return Space::DestroyInstance(space);
    }

    XrResult EnumerateSwapchainFormats(uint32_t formatCapacityInput,
                                       uint32_t *formatCountOutput, int64_t *formats) override
    {
        constexpr uint32_t supportedFormatCount = 6;
        int64_t supportedFormats[supportedFormatCount] = {
            static_cast<int64_t>(MTLPixelFormatRGBA16Float),
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
        *swapchain = Swapchain::RegisterInstance(std::make_unique<Swapchain_Metal>(GetHandle(), *createInfo,
                                                                                       _device, imageCount));
        return XR_SUCCESS;
    }

    XrResult DestroySwapchain(XrSwapchain swapchain) override {
        return Swapchain::DestroyInstance(swapchain);
    }

    XrResult BeginSession(const XrSessionBeginInfo *beginInfo) override {
        if (_sessionState != XrSessionState::XR_SESSION_STATE_READY) {
            return XR_ERROR_SESSION_NOT_READY;
        }

        if (SessionStateIsRunningState(_sessionState)) {
            return XR_ERROR_SESSION_RUNNING;
        }

        if (_layerRenderer != nil) {
            cp_layer_renderer_state rendererState = cp_layer_renderer_get_state(_layerRenderer);
            if (rendererState == cp_layer_renderer_state_paused) {
                cp_layer_renderer_wait_until_running(_layerRenderer);
            }
        }

        TransitionToState(XR_SESSION_STATE_SYNCHRONIZED);
        return XR_SUCCESS;
    }

    bool SessionStateIsRunningState(XrSessionState state) {
        return (state == XR_SESSION_STATE_SYNCHRONIZED) ||
               (state == XR_SESSION_STATE_VISIBLE) ||
               (state == XR_SESSION_STATE_FOCUSED);
    }

    XrResult EndSession() override {
        if (!SessionStateIsRunningState(_sessionState)) {
            return XR_ERROR_SESSION_NOT_RUNNING;
        }
        TransitionToState(XR_SESSION_STATE_EXITING);
        return XR_SUCCESS;
    }

    XrResult RequestExitSession() override {
        if (!SessionStateIsRunningState(_sessionState)) {
            return XR_ERROR_SESSION_NOT_RUNNING;
        }
        TransitionToState(XR_SESSION_STATE_EXITING);
        return XR_SUCCESS;
    }

    XrResult WaitFrame(const XrFrameWaitInfo *frameWaitInfo, XrFrameState *frameState) override {
        VALIDATE_TYPE_NONNULL(XR_TYPE_FRAME_WAIT_INFO, frameWaitInfo);
        VALIDATE_TYPE_NONNULL(XR_TYPE_FRAME_STATE, frameState);
        if (!SessionStateIsRunningState(_sessionState)) {
            return XR_ERROR_SESSION_NOT_RUNNING;
        }
        if (!EnqueueCompositorServicesFrame()) {
            return XR_SESSION_LOSS_PENDING;
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

        cp_layer_renderer_state rendererState = cp_layer_renderer_get_state(_layerRenderer);
        if (rendererState != cp_layer_renderer_state_running) {
            return XR_ERROR_RUNTIME_FAILURE;
        }

        bool discarded;
        auto frameToBegin = _frameQueue.BeginFrame(discarded);
        if (frameToBegin.has_value()) {
            StartCompositorServicesFrame(frameBeginInfo, frameToBegin.value());
            return discarded ? XR_FRAME_DISCARDED : XR_SUCCESS;
        }

        return XR_ERROR_CALL_ORDER_INVALID;
    }

    bool EnqueueCompositorServicesFrame() {
        if (_layerRenderer == nil) {
            return false;
        }

        cp_layer_renderer_state rendererState = cp_layer_renderer_get_state(_layerRenderer);
        if (rendererState != cp_layer_renderer_state_running) {
            return false;
        }
        cp_frame_t frame = cp_layer_renderer_query_next_frame(_layerRenderer);

        cp_frame_timing_t frameTiming = cp_frame_predict_timing(frame);
        cp_time_t estimatedPresentationTime = cp_frame_timing_get_presentation_time(frameTiming);

        XrTime currentTime = _clock.Now();
        XrDuration frameDuration = DurationFromSeconds(1.0 / 90.0); // TODO: Get actual estimate from CP
        XrTime presentationTime = TimeFromTimeInterval(cp_time_to_cf_time_interval(estimatedPresentationTime));

        (void)_frameQueue.EnqueueNewFrame(frame, currentTime, frameDuration, presentationTime);

        return true;
    }

    void StartCompositorServicesFrame(const XrFrameBeginInfo *frameBeginInfo, FrameTiming<cp_frame_t> beginningFrame) {
        cp_frame_t frame = beginningFrame.userData;
        cp_frame_timing_t frameTiming = cp_frame_predict_timing(frame);

        cp_frame_start_update(frame);
        // TODO: Query anchors and action states
        //PerformUpdateSyncInternal();
        cp_frame_end_update(frame);

        cp_time_t inputTime = cp_frame_timing_get_optimal_input_time(frameTiming);
        cp_time_wait_until(inputTime);

        cp_frame_start_submission(frame);
        cp_drawable_t drawable = GetDrawableForFrame(frame);
        if (drawable == nil) {
            return;
        }

        XrTime now = _clock.Now();
        ar_device_anchor_t deviceAnchor = ar_device_anchor_create();
        auto deviceTrackingStatus = ar_world_tracking_provider_query_device_anchor_at_timestamp(_worldTrackingProvider,
                                                                                                TimeIntervalFromXrTime(now),
                                                                                                deviceAnchor);
        UpdateReferenceSpaces(now, deviceAnchor, drawable);
    }

    void EndCompositorServicesFrame(const XrFrameEndInfo *frameEndInfo, FrameTiming<cp_frame_t> endingFrame) {
        cp_frame_t frame = endingFrame.userData;
        cp_drawable_t drawable = GetDrawableForFrame(frame);

        if (drawable == nil) {
            return;
        }

        cp_frame_timing_t drawableTiming = cp_drawable_get_frame_timing(drawable);
        cp_time_t drawablePresentationTime = cp_frame_timing_get_presentation_time(drawableTiming);

        // TODO: Do this after rendering and before presentation to get better prediction
        ar_world_tracking_provider_query_device_anchor_at_timestamp(_worldTrackingProvider,
                                                                    cp_time_to_cf_time_interval(drawablePresentationTime),
                                                                    _deviceAnchor);
        cp_drawable_set_device_anchor(drawable, _deviceAnchor);

        _compositor->CompositeLayersAndPresent(frameEndInfo, frame, drawable);

        cp_frame_end_submission(frame);
        ReleaseDrawableForFrame(frame);
    }

    cp_drawable_t GetDrawableForFrame(cp_frame_t frame) {
        auto frameIndex = cp_frame_get_frame_index(frame);
        auto it = _drawableByFrameIndex.find(frameIndex);
        if (it != _drawableByFrameIndex.end()) {
            return it->second;
        } else {
            cp_drawable_t drawable = cp_frame_query_drawable(frame);
            _drawableByFrameIndex[frameIndex] = drawable;
            return drawable;
        }
        return nil;
    }

    void ReleaseDrawableForFrame(cp_frame_t frame) {
        auto frameIndex = cp_frame_get_frame_index(frame);
        _drawableByFrameIndex.erase(frameIndex);
    }

    void UpdateReferenceSpaces(XrTime now, ar_device_anchor_t _Nullable deviceAnchor, cp_drawable_t drawable) {
        const XrSpaceLocationFlags trackedFlags = XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT |
                                                  XR_SPACE_LOCATION_POSITION_TRACKED_BIT;
        const XrSpaceLocationFlags validFlags = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT |
                                                XR_SPACE_LOCATION_POSITION_VALID_BIT;

        XrSpaceLocationFlags flags = validFlags;

        if (deviceAnchor) {
            simd_float4x4 deviceTransform = ar_anchor_get_origin_from_anchor_transform(deviceAnchor);
            DecomposeRigidTransform(deviceTransform, _latestDevicePose);
            if (ar_trackable_anchor_is_tracked(deviceAnchor)) {
                flags |= trackedFlags;
            }
        }

        XrPosef monocularViewPose = PoseIdentity; // average view pose relative to device anchor transform
        size_t viewCount = cp_drawable_get_view_count(drawable);
        if (viewCount == 1) {
            cp_view_t view = cp_drawable_get_view(drawable, 0);
            simd_float4x4 viewMatrix = cp_view_get_transform(view);
            DecomposeRigidTransform(viewMatrix, monocularViewPose);
        } else if (viewCount == 2) {
            cp_view_t leftView = cp_drawable_get_view(drawable, 0);
            cp_view_t rightView = cp_drawable_get_view(drawable, 1);
            simd_float4x4 leftViewMatrix = cp_view_get_transform(leftView);
            simd_float4x4 rightViewMatrix = cp_view_get_transform(rightView);
            XrPosef leftPose, rightPose;
            DecomposeRigidTransform(leftViewMatrix, leftPose);
            DecomposeRigidTransform(rightViewMatrix, rightPose);
            monocularViewPose = LerpPoseTransforms(leftPose, rightPose, 0.5f);
        }

        for (auto space : _spaces) {
            auto spaceImpl = Space::FromHandle(space);
            if (auto referenceSpace = dynamic_cast<ReferenceSpace_ARKit_visionOS *>(spaceImpl)) {
                referenceSpace->Update(now, monocularViewPose, _latestDevicePose, flags);
            }
        }
    }

    XrResult EndFrame(const XrFrameEndInfo *frameEndInfo) override {
        if (!SessionStateIsRunningState(_sessionState)) {
            return XR_ERROR_SESSION_NOT_RUNNING;
        }

        auto endedFrame = _frameQueue.EndFrame();
        if (endedFrame.has_value()) {
            EndCompositorServicesFrame(frameEndInfo, endedFrame.value());
            XrDuration framePresentationBudget = endedFrame->expectedPresentationTime - _clock.Now();
            if (framePresentationBudget < 0) {
                //printf("[Info] Frame missed presentation deadline and would not have been composited!\n");
            }
        }

        return XR_SUCCESS;
    }

    XrResult LocateViews(const XrViewLocateInfo *viewLocateInfo, XrViewState *viewState,
                         uint32_t viewCapacityInput, uint32_t *viewCountOutput, XrView *views) override
    {
        VALIDATE_TYPE_NONNULL(XR_TYPE_VIEW_LOCATE_INFO, viewLocateInfo);
        VALIDATE_TYPE_NONNULL(XR_TYPE_VIEW_STATE, viewState);
        VALIDATE_NONNULL(viewCountOutput);

        Space *space = Space::FromHandle(viewLocateInfo->space);
        if (space == nullptr) {
            return XR_ERROR_HANDLE_INVALID;
        }

        auto activeFrame = _frameQueue.GetActiveFrame();
        if (!activeFrame.has_value()) {
            return XR_ERROR_RUNTIME_FAILURE;
        }

        cp_frame_t frame = activeFrame.value().userData;
        auto drawable = GetDrawableForFrame(frame);
        if (drawable == nil) {
            return XR_ERROR_RUNTIME_FAILURE;
        }

        cp_drawable_set_depth_range(drawable, simd_float2 { INFINITY, 0.01f });

        ar_device_anchor_t deviceAnchor = ar_device_anchor_create();
        ar_device_anchor_query_status_t trackingStatus =
            ar_world_tracking_provider_query_device_anchor_at_timestamp(_worldTrackingProvider,
                                                                        TimeIntervalFromXrTime(viewLocateInfo->displayTime),
                                                                        deviceAnchor);
        viewState->viewStateFlags = 0;
        if (trackingStatus == ar_device_anchor_query_status_success) {
            viewState->viewStateFlags = XR_VIEW_STATE_POSITION_VALID_BIT | XR_VIEW_STATE_POSITION_TRACKED_BIT |
                                        XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_ORIENTATION_TRACKED_BIT;
        }
        simd_float4x4 deviceTransform = ar_anchor_get_origin_from_anchor_transform(deviceAnchor);

        uint32_t viewCount = (uint32_t)cp_drawable_get_view_count(drawable);
        *viewCountOutput = viewCount;

        if (views != nullptr) {
            if (viewCapacityInput >= viewCount) {
                for (size_t viewIndex = 0; viewIndex < viewCount; ++viewIndex) {
                    XrView &view = views[viewIndex];

                    cp_view_t cpView = cp_drawable_get_view(drawable, viewIndex);

                    simd_float4x4 deviceFromView = cp_view_get_transform(cpView);
                    simd_float4x4 worldToViewMatrix = simd_mul(deviceTransform, deviceFromView);

                    DecomposeRigidTransform(worldToViewMatrix, view.pose);

                    simd_float4 tangents = cp_view_get_tangents(cpView);
                    view.fov = XrFovf { -atanf(tangents[0]), atanf(tangents[1]), atanf(tangents[2]), -atanf(tangents[3]) };
                }
            } else {
                return XR_ERROR_SIZE_INSUFFICIENT;
            }
        }

        return XR_SUCCESS;
    }

    XrResult AttachSessionActionSets(const XrSessionActionSetsAttachInfo *attachInfo) override {
        auto instance = Instance::FromHandle(GetInstance());
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
            auto instance = Instance::FromHandle(GetInstance());
            if (instance == nullptr) {
                return;
            }

            std::vector<XrPath> suggestedProfiles;
            instance->GetSuggestedInteractionProfiles(suggestedProfiles);

            std::vector<XrPath> supportedProfiles;
            _interactionManager->GetSupportedInteractionProfiles(supportedProfiles);

            bool didBindProfile = false;
            for (auto const &suggestedProfile : suggestedProfiles) {
                auto matchedProfile = std::find(begin(supportedProfiles), end(supportedProfiles), suggestedProfile);
                if (matchedProfile != end(supportedProfiles)) {
                    _interactionManager->BindInteractionProfile(*matchedProfile);
                    didBindProfile = true;
                    break;
                }
            }
            if (didBindProfile) {
                Event event {
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

    bool HandTrackingIsSupported() const {
        return ar_hand_tracking_provider_is_supported();
    }

    XrResult CreateHandTrackerEXT(const XrHandTrackerCreateInfoEXT *createInfo, XrHandTrackerEXT *handTracker) override {
        VALIDATE_TYPE_NONNULL(XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT, createInfo);
        VALIDATE_NONNULL(handTracker);
        if (!HandTrackingIsSupported()) {
            // We could plumb the system ID down, retrieve its properties, and ask it if it supports hand tracking,
            // but this is the same test we perform over there, so anyone calling this method should expect this
            // result, since they should know not to call us in the first place.
            return XR_ERROR_FEATURE_UNSUPPORTED;
        }
        if (createInfo->handJointSet != XR_HAND_JOINT_SET_DEFAULT_EXT) {
            return XR_ERROR_EXTENSION_NOT_PRESENT;
        }
        auto handTrackerImpl = std::make_unique<HandTracker_ARKit_visionOS>(GetHandle(),
                                                                            createInfo->hand,
                                                                            createInfo->handJointSet);
        *handTracker = HandTrackerEXT::RegisterInstance(std::move(handTrackerImpl));
        _handTrackers.push_back(*handTracker);
        return XR_SUCCESS;
    }

    XrResult DestroyHandTrackerEXT(XrHandTrackerEXT handTracker) override {
        return HandTrackerEXT::DestroyInstance(handTracker);
    }

    XrResult LocateHandJointsEXT(XrHandTrackerEXT handTracker,
                                 const XrHandJointsLocateInfoEXT *locateInfo,
                                 XrHandJointLocationsEXT *locations) override
    {
        VALIDATE_TYPE_NONNULL(XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT, locateInfo);

        //if ((_handTrackingProvider == nil) ||
        //    (ar_data_provider_get_state(_handTrackingProvider) != ar_data_provider_state_running))
        //{
        //    return XR_ERROR_RUNTIME_FAILURE;
        //}

        XrTime time = locateInfo->time;
        XrSpace locationSpace = locateInfo->baseSpace;

        if (auto impl = HandTrackerEXT::FromHandle(handTracker)) {
            if (auto arImpl = dynamic_cast<HandTracker_ARKit_visionOS *>(impl)) {
                return arImpl->Locate(locationSpace, time, locations);
            }
        }
        return XR_ERROR_HANDLE_INVALID;
    }

private:

    void PrepareToStart() {
        ar_session_set_data_provider_state_change_handler(_arSession,
                                                          dispatch_get_main_queue(),
                                                          ^(ar_data_providers_t dataProviders,
                                                            ar_data_provider_state_t newState,
                                                            ar_error_t error,
                                                            ar_data_provider_t failedDataProvider)
        {
            if (error) {
                ar_error_code_t errorCode = ar_error_get_error_code(error);
                switch (errorCode) {
                    case ar_session_error_code_data_provider_not_authorized:
                        break;
                    case ar_session_error_code_data_provider_failed_to_run:
                        break;
                }
                TransitionToState(XR_SESSION_STATE_LOSS_PENDING);
            } else {
                switch (newState) {
                    case ar_data_provider_state_initialized:
                        NSLog(@"Data provider did initialize");
                        break;
                    case ar_data_provider_state_running:
                        NSLog(@"Data provider is running");
                        TransitionToState(XR_SESSION_STATE_VISIBLE);
                        TransitionToState(XR_SESSION_STATE_FOCUSED);
                        break;
                    case ar_data_provider_state_paused:
                        NSLog(@"Data provider is paused");
                        TransitionToState(XR_SESSION_STATE_SYNCHRONIZED);
                        break;
                    case ar_data_provider_state_stopped:
                        NSLog(@"Data provider is stopped");
                        TransitionToState(XR_SESSION_STATE_STOPPING);
                        break;
                }
            }
        });

        ar_session_set_authorization_update_handler(_arSession,
                                                    dispatch_get_main_queue(),
                                                    ^(ar_authorization_result_t authResult)
        {
            NSLog(@"AR session authorization did produce result: %@", authResult);
        });

        ar_data_providers_t dataProviders = ar_data_providers_create();

        if (ar_world_tracking_provider_is_supported()) {
            ar_world_tracking_configuration_t worldTrackingConfig = ar_world_tracking_configuration_create();
            _worldTrackingProvider = ar_world_tracking_provider_create(worldTrackingConfig);
            ar_data_providers_add_data_provider(dataProviders, _worldTrackingProvider);
        } else {
            printf("[Warning] This platform does not support the necessary ARKit configuration\n");
        }

        if (_wantsHandTracking && HandTrackingIsSupported()) {
            ar_hand_tracking_configuration_t handTrackingConfig = ar_hand_tracking_configuration_create();
            _handTrackingProvider = ar_hand_tracking_provider_create(handTrackingConfig);
            ar_data_providers_add_data_provider(dataProviders, _handTrackingProvider);
            ar_hand_tracking_provider_set_update_handler(_handTrackingProvider,
                                                         dispatch_get_main_queue(),
                                                         ^(ar_hand_anchor_t leftAnchor, ar_hand_anchor_t rightAnchor)
            {
                UpdateHandTrackers(_clock.Now(), leftAnchor, rightAnchor);
            });
        }

        ar_session_run(_arSession, dataProviders);

        _deviceAnchor = ar_device_anchor_create();

        TransitionToState(XrSessionState::XR_SESSION_STATE_READY);
    }

    void UpdateHandTrackers(XrTime time, ar_hand_anchor_t leftAnchor, ar_hand_anchor_t rightAnchor) {
        const XrSpaceLocationFlags trackedFlags = XR_SPACE_LOCATION_POSITION_TRACKED_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
        const XrSpaceLocationFlags validFlags = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
        // TODO: Thread safety!
        [@[leftAnchor, rightAnchor] enumerateObjectsUsingBlock:^(ar_hand_anchor_t anchor, NSUInteger index, BOOL *stop) {
            simd_float4x4 originFromAnchorTransform = ar_anchor_get_origin_from_anchor_transform(anchor);
            ar_hand_chirality_t handedness = ar_hand_anchor_get_chirality(anchor);
            ar_hand_skeleton_t skeleton = ar_hand_anchor_get_hand_skeleton(anchor);
            __block HandTrackerTimeSample timeSample {
                .time = time,
            };
            ar_hand_skeleton_enumerate_joints(skeleton, ^bool(ar_skeleton_joint_t joint) {
                int64_t jointIndex = ar_skeleton_joint_get_index(joint);
                int32_t xrJointIndex = GetXrHandJointIndexForHandSkeletonJointIndex(jointIndex);
                bool jointIsTracked = ar_skeleton_joint_is_tracked(joint);
                simd_float4x4 anchorFromJointTransform = ar_skeleton_joint_get_anchor_from_joint_transform(joint);
                if (xrJointIndex > 0) {
                    timeSample.jointPoses[xrJointIndex].time = time;
                    timeSample.jointPoses[xrJointIndex].locationFlags = jointIsTracked ? (trackedFlags | validFlags) : 0;
                    simd_float4x4 worldPose = simd_mul(originFromAnchorTransform, anchorFromJointTransform);
                    DecomposeRigidTransform(worldPose, timeSample.jointPoses[xrJointIndex].pose);
                }
                return true;
            });
            for (auto tracker : _handTrackers) {
                auto impl = HandTrackerEXT::FromHandle(tracker);
                if (auto arImpl = dynamic_cast<HandTracker_ARKit_visionOS *>(impl)) {
                    switch (handedness) {
                        case ar_hand_chirality_right:
                            if (arImpl->GetHand() == XR_HAND_RIGHT_EXT) {
                                arImpl->AddJointPoseTimeSample(timeSample);
                            }
                            break;
                        case ar_hand_chirality_left:
                            if (arImpl->GetHand() == XR_HAND_LEFT_EXT) {
                                arImpl->AddJointPoseTimeSample(timeSample);
                            }
                            break;
                    }
                }
            }
        }];
        XrPosef leftHandPose, rightHandPose;
        simd_float4x4 leftHandTransform = ar_anchor_get_origin_from_anchor_transform(leftAnchor);
        simd_float4x4 rightHandTransform = ar_anchor_get_origin_from_anchor_transform(rightAnchor);
        DecomposeRigidTransform(leftHandTransform, leftHandPose);
        DecomposeRigidTransform(rightHandTransform, rightHandPose);
        for (auto space : _spaces) {
            auto impl = Space::FromHandle(space);
            if (auto actionSpaceImpl = dynamic_cast<ActionSpace_ARKit_visionOS *>(impl)) {
                actionSpaceImpl->Update(time, leftHandPose, rightHandPose, trackedFlags | validFlags);
            }
        }
    }

    void TransitionToState(XrSessionState nextState) {
        if (_sessionState == nextState) { // No change, early-out.
            return;
        }
        if (auto instanceImpl = Instance::FromHandle(GetInstance())) {
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
            Event event {
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
    FrameQueue<cp_frame_t> _frameQueue;
    std::unordered_map<cp_layer_frame_index_t, cp_drawable_t> _drawableByFrameIndex;
    XrPosef _latestDevicePose = PoseIdentity;
    Clock _clock;
    std::vector<XrSpace> _spaces;
    std::vector<XrSwapchain> _swapchains;
    std::vector<XrHandTrackerEXT> _handTrackers;
    std::unique_ptr<Compositor_CP> _compositor;
    std::unique_ptr<InteractionManager> _interactionManager;
    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    cp_layer_renderer_t _layerRenderer;
    ar_session_t _arSession = nil;
    ar_device_anchor_t _deviceAnchor;
    ar_world_tracking_provider_t _worldTrackingProvider = nil;
    ar_hand_tracking_provider_t _handTrackingProvider = nil;
    XrSessionState _sessionState = XR_SESSION_STATE_IDLE;
    bool _needsInteractionProfileUpdate = true;
    bool _wantsHandTracking = true;
    bool _hasAttachedActionSets = false;
};

} // end anonymous namespace

std::unique_ptr<Session> CreateSession_visionOS(XrInstance instance, const XrGraphicsBindingMetalKHR *binding) {
    assert(binding != NULL);
    assert(binding->device != nil);
    return std::make_unique<Session_visionOS_ARKit>(instance, binding);
}

#endif
