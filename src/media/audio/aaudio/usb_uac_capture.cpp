/*
 *  Copyright (C) 2004-2026 Savoir-faire Linux Inc.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 */

#include "usb_uac_capture.h"

#include "logger.h"

#include <linux/usbdevice_fs.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <algorithm>
#include <memory>
#include <thread>
#include <vector>

namespace jami {

namespace {

constexpr int URB_COUNT = 8;
constexpr int PACKETS_PER_URB = 8;
// Some Android USB host implementations do not wake poll() for completed
// usbfs URBs. Keep the timeout short and also try REAPURBNDELAY after a
// timeout, otherwise a successfully submitted stream can remain silent
// forever even though the kernel completion queue is progressing.
constexpr int POLL_TIMEOUT_MS = 10;

struct Transfer
{
    std::unique_ptr<uint8_t[]> storage;
    size_t storageSize {0};
    usbdevfs_urb* urb {nullptr};
    int packetSize {0};
    bool submitted {false};
};

} // namespace

struct UsbUacCapture::Impl
{
    int fd {-1};
    int endpoint {0};
    int packetSize {0};
    std::atomic_bool stopping {false};
    std::atomic_bool running {false};
    PcmCallback callback;
    std::vector<std::unique_ptr<Transfer>> transfers;
    std::thread thread;
    uint64_t completedTransfers {0};
    uint64_t validPackets {0};
    uint64_t errorPackets {0};
    uint64_t emptyPackets {0};
    uint64_t pcmBytes {0};
    uint64_t submitAttempts {0};
    uint64_t submitErrors {0};
    uint64_t pollTimeouts {0};
    uint64_t pollEvents {0};
    uint64_t reapAfterPollTimeouts {0};
    uint64_t reapedWithoutPollEvent {0};
    uint64_t reapEmpty {0};
    uint64_t reapErrors {0};
    uint64_t pcmCallbacks {0};
    uint64_t callbackTimeUs {0};
    uint64_t maxCallbackTimeUs {0};
    uint64_t lastStatsCompleted {0};
    uint64_t lastStatsPcmBytes {0};
    std::chrono::steady_clock::time_point lastStats {};
    std::chrono::steady_clock::time_point lastCompletion {};

    void logStats(bool force = false)
    {
        const auto now = std::chrono::steady_clock::now();
        if (lastStats == std::chrono::steady_clock::time_point {})
            lastStats = now;
        if (!force && now - lastStats < std::chrono::seconds(1))
            return;

        const auto elapsed = std::chrono::duration<double>(now - lastStats).count();
        const auto completedSinceLast = completedTransfers - lastStatsCompleted;
        const auto bytesSinceLast = pcmBytes - lastStatsPcmBytes;
        JAMI_WARNING("[USB UAC stats] fd={} endpoint=0x{:02x} submitted={} submitErr={} completed={} (+{} / {:.1f}s) validPackets={} errorPackets={} emptyPackets={} pcmBytes={} (+{}) callbacks={} pollEvents={} pollTimeouts={} reapAfterTimeouts={} reapedWithoutPoll={} reapEmpty={} reapErr={} callbackAvg={:.2f}ms callbackMax={:.2f}ms lastCompletionMsAgo={:.1f}",
                     fd,
                     endpoint,
                     submitAttempts,
                     submitErrors,
                     completedTransfers,
                     completedSinceLast,
                     elapsed,
                     validPackets,
                     errorPackets,
                     emptyPackets,
                     pcmBytes,
                     bytesSinceLast,
                     pcmCallbacks,
                     pollEvents,
                     pollTimeouts,
                     reapAfterPollTimeouts,
                     reapedWithoutPollEvent,
                     reapEmpty,
                     reapErrors,
                     pcmCallbacks ? static_cast<double>(callbackTimeUs) / pcmCallbacks / 1000.0 : 0.0,
                     static_cast<double>(maxCallbackTimeUs) / 1000.0,
                     lastCompletion == std::chrono::steady_clock::time_point {}
                         ? -1.0
                         : std::chrono::duration<double, std::milli>(now - lastCompletion).count());
        lastStatsCompleted = completedTransfers;
        lastStatsPcmBytes = pcmBytes;
        lastStats = now;
    }

    bool submit(Transfer& transfer)
    {
        ++submitAttempts;
        auto* urb = transfer.urb;
        urb->type = USBDEVFS_URB_TYPE_ISO;
        urb->endpoint = static_cast<unsigned char>(endpoint);
        urb->flags = USBDEVFS_URB_ISO_ASAP;
        urb->buffer_length = transfer.packetSize * PACKETS_PER_URB;
        urb->actual_length = 0;
        urb->start_frame = -1;
        urb->number_of_packets = PACKETS_PER_URB;
        urb->error_count = 0;

        for (int i = 0; i < PACKETS_PER_URB; ++i) {
            auto& packet = urb->iso_frame_desc[i];
            packet.length = transfer.packetSize;
            packet.actual_length = 0;
            packet.status = 0;
        }

        if (ioctl(fd, USBDEVFS_SUBMITURB, urb) < 0) {
            ++submitErrors;
            if (!stopping.load(std::memory_order_relaxed)) {
                JAMI_ERROR("USB UAC: failed to submit isochronous URB: {} ({})", strerror(errno), errno);
            }
            return false;
        }
        transfer.submitted = true;
        return true;
    }

    void process(Transfer& transfer)
    {
        std::vector<uint8_t> pcm;
        pcm.reserve(static_cast<size_t>(packetSize) * PACKETS_PER_URB);
        auto* urb = transfer.urb;
        auto* buffer = static_cast<uint8_t*>(urb->buffer);
        ++completedTransfers;
        lastCompletion = std::chrono::steady_clock::now();
        unsigned validPacketsInTransfer = 0;
        unsigned errorPacketsInTransfer = 0;
        unsigned emptyPacketsInTransfer = 0;

        for (int i = 0; i < urb->number_of_packets; ++i) {
            const auto& packet = urb->iso_frame_desc[i];
            if (packet.status != 0) {
                ++errorPackets;
                ++errorPacketsInTransfer;
                continue;
            }
            if (packet.actual_length == 0) {
                ++emptyPackets;
                ++emptyPacketsInTransfer;
                continue;
            }
            if (packet.actual_length > packet.length) {
                JAMI_WARNING("USB UAC: invalid isochronous packet length {} > {}", packet.actual_length, packet.length);
                ++errorPackets;
                ++errorPacketsInTransfer;
                continue;
            }
            const auto offset = static_cast<size_t>(i) * transfer.packetSize;
            pcm.insert(pcm.end(), buffer + offset, buffer + offset + packet.actual_length);
            ++validPackets;
            ++validPacketsInTransfer;
        }

        pcmBytes += pcm.size();
        if (completedTransfers == 1 || completedTransfers % 250 == 0) {
            JAMI_WARNING("USB UAC: completed URB #{}, validPackets={}, errorPackets={}, emptyPackets={}, bytes={}",
                         completedTransfers,
                         validPacketsInTransfer,
                         errorPacketsInTransfer,
                         emptyPacketsInTransfer,
                         pcm.size());
        }
        if (!pcm.empty() && callback) {
            const auto callbackStart = std::chrono::steady_clock::now();
            ++pcmCallbacks;
            callback(pcm.data(), pcm.size());
            const auto callbackUs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - callbackStart).count());
            callbackTimeUs += callbackUs;
            maxCallbackTimeUs = std::max(maxCallbackTimeUs, callbackUs);
        }
        logStats();
    }

    void run()
    {
        JAMI_WARNING("USB UAC: worker entered fd={} endpoint=0x{:02x} packetSize={}", fd, endpoint, packetSize);
        pollfd pollFd {fd, POLLIN | POLLERR | POLLHUP, 0};
        while (!stopping.load(std::memory_order_relaxed)) {
            const auto pollResult = poll(&pollFd, 1, POLL_TIMEOUT_MS);
            if (stopping.load(std::memory_order_relaxed))
                break;
            if (pollResult < 0) {
                if (errno == EINTR)
                    continue;
                JAMI_ERROR("USB UAC: poll failed: {} ({})", strerror(errno), errno);
                break;
            }
            if (pollResult == 0) {
                ++pollTimeouts;
                ++reapAfterPollTimeouts;
            } else {
                ++pollEvents;
                if (pollFd.revents & (POLLERR | POLLHUP))
                    JAMI_WARNING("USB UAC: poll returned revents=0x{:x} fd={}", pollFd.revents, fd);
            }

            while (!stopping.load(std::memory_order_relaxed)) {
                void* context = nullptr;
                if (ioctl(fd, USBDEVFS_REAPURBNDELAY, &context) < 0) {
                    if (errno == EAGAIN) {
                        ++reapEmpty;
                        break;
                    }
                    ++reapErrors;
                    if (!stopping.load(std::memory_order_relaxed))
                        JAMI_ERROR("USB UAC: failed to reap isochronous URB: {} ({})", strerror(errno), errno);
                    break;
                }
                if (!context) {
                    ++reapEmpty;
                    break;
                }

                // USBDEVFS_REAPURB returns the completed usbdevfs_urb pointer,
                // not the value stored in usbdevfs_urb::usercontext. Recover
                // the owning Transfer through usercontext before touching the
                // URB. Treating context as Transfer* corrupts the pointer at
                // transfer.urb and crashes the daemon on the first completion.
                auto* completedUrb = static_cast<usbdevfs_urb*>(context);
                auto* transfer = completedUrb
                                     ? static_cast<Transfer*>(completedUrb->usercontext)
                                     : nullptr;
                if (!transfer || transfer->urb != completedUrb) {
                    ++reapErrors;
                    JAMI_ERROR("USB UAC: reaped invalid URB context urb={} usercontext={} transferUrb={}",
                               static_cast<void*>(completedUrb),
                               completedUrb ? completedUrb->usercontext : nullptr,
                               transfer ? static_cast<void*>(transfer->urb) : nullptr);
                    break;
                }
                if (pollResult == 0) {
                    ++reapedWithoutPollEvent;
                    if (reapedWithoutPollEvent == 1 || reapedWithoutPollEvent % 250 == 0)
                        JAMI_WARNING("USB UAC: reaped URB after poll timeout (count={})", reapedWithoutPollEvent);
                }
                transfer->submitted = false;
                process(*transfer);
                if (stopping.load(std::memory_order_relaxed))
                    break;
                if (!submit(*transfer))
                    break;
            }
        }
        running.store(false, std::memory_order_release);
        logStats(true);
        if (!stopping.load(std::memory_order_relaxed)) {
            JAMI_ERROR("USB UAC: capture worker stopped unexpectedly (completedTransfers={}, validPackets={}, errorPackets={}, emptyPackets={}, bytes={})",
                       completedTransfers,
                       validPackets,
                       errorPackets,
                       emptyPackets,
                       pcmBytes);
        }
    }
};

UsbUacCapture::UsbUacCapture()
    : impl_(std::make_unique<Impl>())
{}

UsbUacCapture::~UsbUacCapture()
{
    stop();
}

bool
UsbUacCapture::start(int fileDescriptor,
                     int interfaceNumber,
                     int alternateSetting,
                     int endpointAddress,
                     int packetSize,
                     PcmCallback callback)
{
    stop();

    if (fileDescriptor < 0 || endpointAddress <= 0 || packetSize <= 0 || !callback) {
        JAMI_ERROR("USB UAC: invalid capture parameters fd={} interface={} alt={} endpoint={} packetSize={}",
                   fileDescriptor,
                   interfaceNumber,
                   alternateSetting,
                   endpointAddress,
                   packetSize);
        return false;
    }

    const auto ownedFd = dup(fileDescriptor);
    if (ownedFd < 0) {
        JAMI_ERROR("USB UAC: failed to duplicate USB file descriptor: {} ({})", strerror(errno), errno);
        return false;
    }

    impl_->fd = ownedFd;
    impl_->endpoint = endpointAddress;
    impl_->packetSize = packetSize;
    impl_->callback = std::move(callback);
    impl_->stopping.store(false, std::memory_order_relaxed);
    impl_->completedTransfers = 0;
    impl_->validPackets = 0;
    impl_->errorPackets = 0;
    impl_->emptyPackets = 0;
    impl_->pcmBytes = 0;
    impl_->submitAttempts = 0;
    impl_->submitErrors = 0;
    impl_->pollTimeouts = 0;
    impl_->pollEvents = 0;
    impl_->reapAfterPollTimeouts = 0;
    impl_->reapedWithoutPollEvent = 0;
    impl_->reapEmpty = 0;
    impl_->reapErrors = 0;
    impl_->pcmCallbacks = 0;
    impl_->callbackTimeUs = 0;
    impl_->maxCallbackTimeUs = 0;
    impl_->lastStatsCompleted = 0;
    impl_->lastStatsPcmBytes = 0;
    impl_->lastStats = {};
    impl_->lastCompletion = {};

    const auto urbSize = offsetof(usbdevfs_urb, iso_frame_desc)
                         + sizeof(usbdevfs_iso_packet_desc) * PACKETS_PER_URB;
    const auto bufferSize = static_cast<size_t>(packetSize) * PACKETS_PER_URB;
    const auto storageSize = urbSize + bufferSize;
    impl_->transfers.reserve(URB_COUNT);

    for (int i = 0; i < URB_COUNT; ++i) {
        auto transfer = std::make_unique<Transfer>();
        transfer->storageSize = storageSize;
        transfer->packetSize = packetSize;
        transfer->storage = std::make_unique<uint8_t[]>(storageSize);
        std::memset(transfer->storage.get(), 0, storageSize);
        transfer->urb = reinterpret_cast<usbdevfs_urb*>(transfer->storage.get());
        transfer->urb->buffer = transfer->storage.get() + urbSize;
        transfer->urb->usercontext = transfer.get();
        impl_->transfers.emplace_back(std::move(transfer));
    }

    // Submit the first batch synchronously. UsbDeviceConnection.fileDescriptor
    // is only useful while the app still owns the USB permission/claim. The
    // previous implementation returned true before the worker submitted any
    // URB, so an EPERM/EIO at this point left the Java and AAudio layers in a
    // false "capture active" state and the normal microphone was suppressed.
    for (auto& transfer : impl_->transfers) {
        if (!impl_->submit(*transfer)) {
            JAMI_ERROR("USB UAC: initial isochronous URB submission failed");
            for (auto& pending : impl_->transfers) {
                if (pending->submitted)
                    ioctl(ownedFd, USBDEVFS_DISCARDURB, pending->urb);
            }
            close(ownedFd);
            impl_->fd = -1;
            impl_->transfers.clear();
            impl_->callback = {};
            return false;
        }
    }

    JAMI_WARNING("USB UAC: starting isochronous capture fd={} interface={} alt={} endpoint=0x{:02x} packetSize={} ({} URBs x {} packets), submitted={}",
                 ownedFd,
                 interfaceNumber,
                 alternateSetting,
                 endpointAddress,
                 packetSize,
                 URB_COUNT,
                 PACKETS_PER_URB,
                 impl_->submitAttempts - impl_->submitErrors);
    impl_->running.store(true, std::memory_order_release);
    impl_->thread = std::thread([this] { impl_->run(); });
    return true;
}

void
UsbUacCapture::stop()
{
    if (!impl_)
        return;

    impl_->stopping.store(true, std::memory_order_relaxed);
    if (impl_->fd >= 0) {
        close(impl_->fd);
    }
    if (impl_->thread.joinable())
        impl_->thread.join();
    impl_->fd = -1;
    impl_->transfers.clear();
    impl_->callback = {};
    impl_->running.store(false, std::memory_order_release);
}

bool
UsbUacCapture::isRunning() const
{
    return impl_ && impl_->running.load(std::memory_order_acquire);
}

} // namespace jami
