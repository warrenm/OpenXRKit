/*
** Copyright 2024-2025, Warren Moore
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

#pragma once

#include <sys/semaphore.h>
#include <optional>
#include <deque>

template <typename UserData>
struct FrameTiming {
    UserData userData;
    uint64_t frameIndex = 0;
    XrTime startTime = 0;
    XrDuration expectedDuration = 0;
    XrTime expectedPresentationTime = 0;
};

template <typename UserData>
class FrameQueue {
public:
    using Frame = FrameTiming<UserData>;

    FrameQueue() :
        _queueMutex PTHREAD_MUTEX_INITIALIZER,
        _queueNonemptyCondition PTHREAD_COND_INITIALIZER
    {}

    [[nodiscard]]
    std::optional<Frame> WaitFrame() {
        pthread_mutex_lock(&_queueMutex);
        while (_waitableFrames.empty()) {
            pthread_cond_wait(&_queueNonemptyCondition, &_queueMutex);
        }
        std::optional<Frame> frame;
        if (_waitableFrames.size() != 0) {
            frame = _waitableFrames.back();
            _waitableFrames.clear();
            _waitedFrames.push_back(frame.value());
        }
        pthread_mutex_unlock(&_queueMutex);
        return frame;
    }

    [[nodiscard]]
    std::optional<Frame> BeginFrame(bool &discardedPrevious) {
        pthread_mutex_lock(&_queueMutex);
        std::optional<Frame> frame;
        if (_waitedFrames.size() != 0) {
            frame = _waitedFrames[0];
            _waitedFrames.pop_front();
            discardedPrevious = (_activeFrames.size() > 0);
            _activeFrames.push_back(frame.value());
        }
        pthread_mutex_unlock(&_queueMutex);
        return frame;
    }

    [[nodiscard]]
    std::optional<Frame> GetActiveFrame() {
        pthread_mutex_lock(&_queueMutex);
        std::optional<Frame> frame;
        if (_activeFrames.size() != 0) {
            frame = _activeFrames[0];
        }
        pthread_mutex_unlock(&_queueMutex);
        return frame;
    }

    [[nodiscard]]
    std::optional<Frame> EndFrame() {
        pthread_mutex_lock(&_queueMutex);
        std::optional<Frame> frame;
        if (_activeFrames.size() != 0) {
            frame = _activeFrames[0];
            _activeFrames.pop_front();
        }
        pthread_mutex_unlock(&_queueMutex);
        return frame;
    }

    Frame EnqueueNewFrame(UserData userData, XrTime now, XrDuration expectedFrameDuration, XrTime presentationTime) {
        pthread_mutex_lock(&_queueMutex);
        Frame nextFrame {
            .userData = userData,
            .frameIndex = _nextFrameIndex++,
            .startTime = now,
            .expectedDuration = expectedFrameDuration,
            .expectedPresentationTime = presentationTime,
        };
        _waitableFrames.push_back(nextFrame);
        pthread_cond_signal(&_queueNonemptyCondition);
        pthread_mutex_unlock(&_queueMutex);
        return nextFrame;
    }

private:
    std::deque<Frame> _waitableFrames;
    std::deque<Frame> _waitedFrames;
    std::deque<Frame> _activeFrames;
    pthread_cond_t _queueNonemptyCondition;
    pthread_mutex_t _queueMutex;
    uint64_t _nextFrameIndex = 1;
};
