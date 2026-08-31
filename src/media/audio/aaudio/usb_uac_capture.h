/*
 *  Copyright (C) 2004-2026 Savoir-faire Linux Inc.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <functional>
#include <memory>

namespace jami {

/**
 * Reads PCM from a USB Audio Class isochronous IN endpoint.
 *
 * Android's public UsbRequest API does not expose isochronous transfers. The
 * Java side therefore opens and claims the interface, then passes the
 * UsbDeviceConnection file descriptor to this small usbfs reader.
 */
class UsbUacCapture final
{
public:
    using PcmCallback = std::function<void(const uint8_t*, size_t)>;

    UsbUacCapture();
    ~UsbUacCapture();

    UsbUacCapture(const UsbUacCapture&) = delete;
    UsbUacCapture& operator=(const UsbUacCapture&) = delete;

    bool start(int fileDescriptor,
               int interfaceNumber,
               int alternateSetting,
               int endpointAddress,
               int packetSize,
               PcmCallback callback);
    void stop();
    bool isRunning() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace jami
