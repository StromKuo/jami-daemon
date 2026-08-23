/*
 *  Copyright (C) 2004-2026 Savoir-faire Linux Inc.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "libav_deps.h" // MUST BE INCLUDED FIRST
#include "media_decoder.h"
#include "media_device.h"
#include "media_buffer.h"
#include "media_io_handle.h"
#include "audio/ringbufferpool.h"
#include "decoder_finder.h"
#include "manager.h"

#ifdef ENABLE_HWACCEL
#include "video/accel.h"
#endif

#ifdef __ANDROID__
extern "C" {
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_mediacodec.h>
}
#endif

#include "string_utils.h"
#include "logger.h"
#include "client/jami_signal.h"

#include <unistd.h>
#include <cstddef>
#include <thread> // hardware_concurrency
#include <chrono>
#include <algorithm>
#include <asio/steady_timer.hpp>

namespace jami {

// maximum number of packets the jitter buffer can queue
const unsigned jitterBufferMaxSize_ {1500};
// maximum time a packet can be queued
const constexpr auto jitterBufferMaxDelay_ = std::chrono::milliseconds(50);

MediaDemuxer::MediaDemuxer()
    : inputCtx_(avformat_alloc_context())
    , startTime_(AV_NOPTS_VALUE)
{}

MediaDemuxer::~MediaDemuxer()
{
    if (streamInfoTimer_) {
        streamInfoTimer_->cancel();
        streamInfoTimer_.reset();
    }
    if (inputCtx_)
        avformat_close_input(&inputCtx_);
    av_dict_free(&options_);
}

const char*
MediaDemuxer::getStatusStr(Status status)
{
    switch (status) {
    case Status::Success:
        return "Success";
    case Status::EndOfFile:
        return "End of file";
    case Status::ReadBufferOverflow:
        return "Read overflow";
    case Status::ReadError:
        return "Read error";
    case Status::FallBack:
        return "Fallback";
    case Status::RestartRequired:
        return "Restart required";
    default:
        return "Undefined";
    }
}

int
MediaDemuxer::openInput(const DeviceParams& params)
{
    inputParams_ = params;
    const auto* iformat = av_find_input_format(params.format.c_str());

    if (!iformat && !params.format.empty())
        JAMI_WARNING("Unable to find format \"{}\"", params.format);

    std::string input;

    if (params.input == "pipewiregrab") {
        //
        // We rely on pipewiregrab for screen/window sharing on Wayland.
        // Because pipewiregrab is a "video source filter" (part of FFmpeg's libavfilter
        // library), its options must all be passed as part of the `input` string.
        //
        input = fmt::format("pipewiregrab=draw_mouse=1:fd={}:node={}", params.fd, params.node);
        JAMI_LOG("Attempting to open input {}", input);
        //
        // In all other cases, we use the `options_` AVDictionary to pass options to FFmpeg.
        //
        // NOTE: We rely on the "lavfi" virtual input device to read pipewiregrab's output
        // and create a corresponding stream (cf. the getDeviceParams function in
        // daemon/src/media/video/v4l2/video_device_impl.cpp). The `options_` dictionary
        // could be used to set lavfi's parameters if that was ever needed, but it isn't at
        // the moment. (Doc: https://ffmpeg.org/ffmpeg-devices.html#lavfi)
        //
    } else {
        if (params.width and params.height) {
            auto sizeStr = fmt::format("{}x{}", params.width, params.height);
            av_dict_set(&options_, "video_size", sizeStr.c_str(), 0);
        }

        if (params.framerate) {
#ifdef _WIN32
            // On Windows, framerate settings don't reduce to avrational values
            // that correspond to valid video device formats.
            // e.g. A the rational<double>(10000000, 333333) or 30.000030000
            //      will be reduced by av_reduce to 999991/33333 or 30.00003000003
            //      which cause the device opening routine to fail.
            // So we treat this imprecise reduction and adjust the value,
            // or let dshow choose the framerate, which is, unfortunately,
            // NOT the highest according to our experimentations.
            auto framerate {params.framerate.real()};
            framerate = params.framerate.numerator() / (params.framerate.denominator() + 0.5);
            if (params.framerate.denominator() != 4999998)
                av_dict_set(&options_, "framerate", jami::to_string(framerate).c_str(), 0);
#else
            av_dict_set(&options_, "framerate", jami::to_string(params.framerate.real()).c_str(), 0);
#endif
        }

        if (params.offset_x || params.offset_y) {
            av_dict_set(&options_, "offset_x", std::to_string(params.offset_x).c_str(), 0);
            av_dict_set(&options_, "offset_y", std::to_string(params.offset_y).c_str(), 0);
        }
        if (params.channel)
            av_dict_set(&options_, "channel", std::to_string(params.channel).c_str(), 0);
        av_dict_set(&options_, "loop", params.loop.c_str(), 0);
        av_dict_set(&options_, "sdp_flags", params.sdp_flags.c_str(), 0);

        // Set jitter buffer options
        av_dict_set(&options_, "reorder_queue_size", std::to_string(jitterBufferMaxSize_).c_str(), 0);
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(jitterBufferMaxDelay_).count();
        av_dict_set(&options_, "max_delay", std::to_string(us).c_str(), 0);

        if (!params.pixel_format.empty()) {
            av_dict_set(&options_, "pixel_format", params.pixel_format.c_str(), 0);
        }
        if (!params.window_id.empty()) {
            av_dict_set(&options_, "window_id", params.window_id.c_str(), 0);
        }
        av_dict_set(&options_, "draw_mouse", "1", 0);
        av_dict_set(&options_, "is_area", std::to_string(params.is_area).c_str(), 0);

        input = params.input;

        JAMI_LOG("Attempting to open input {} with format {}, pixel format {}, size {}x{}, rate {}",
                 input,
                 params.format,
                 params.pixel_format,
                 params.width,
                 params.height,
                 params.framerate.real());
    }

    // Ask FFmpeg to open the input using the options set above
    if (params.disable_dts_probe_delay && params.format == "sdp") {
        av_opt_set_int(inputCtx_, "max_ts_probe", 0, AV_OPT_SEARCH_CHILDREN);
        av_opt_set_int(inputCtx_, "fpsprobesize", 0, AV_OPT_SEARCH_CHILDREN);
    } else {
        // Don't waste time fetching framerate when finding stream info
        av_opt_set_int(inputCtx_, "fpsprobesize", 1, AV_OPT_SEARCH_CHILDREN);
    }

    int ret = avformat_open_input(&inputCtx_, input.c_str(), iformat, options_ ? &options_ : NULL);

    if (ret) {
        JAMI_ERROR("avformat_open_input failed: {}", libav_utils::getError(ret));
    } else if (inputCtx_->nb_streams > 0 && inputCtx_->streams[0]->codecpar) {
        baseWidth_ = inputCtx_->streams[0]->codecpar->width;
        baseHeight_ = inputCtx_->streams[0]->codecpar->height;
        JAMI_LOG("Opened input using format {:s} and resolution {:d}x{:d}", params.format, baseWidth_, baseHeight_);
    }

    return ret;
}

int64_t
MediaDemuxer::getDuration() const
{
    return inputCtx_->duration;
}

bool
MediaDemuxer::seekFrame(int, int64_t timestamp)
{
    std::lock_guard lk(inputCtxMutex_);
    if (av_seek_frame(inputCtx_, -1, timestamp, AVSEEK_FLAG_BACKWARD) >= 0) {
        clearFrames();
        return true;
    }
    return false;
}

void
MediaDemuxer::findStreamInfo(bool videoStream)
{
    if (not streamInfoFound_) {
        inputCtx_->max_analyze_duration = 30l * AV_TIME_BASE;
        if (videoStream && keyFrameRequestCb_) {
            if (!streamInfoTimer_)
                streamInfoTimer_ = std::make_unique<asio::steady_timer>(*Manager::instance().ioContext());
            streamInfoTimer_->expires_after(std::chrono::milliseconds(1500));
            streamInfoTimer_->async_wait([weak = weak_from_this()](const std::error_code& ec) {
                if (ec)
                    return;
                if (auto self = weak.lock()) {
                    if (!self->streamInfoFound_) {
                        JAMI_LOG("findStreamInfo: 1500ms elapsed, requesting keyframe to aid probing");
                        if (self->keyFrameRequestCb_)
                            self->keyFrameRequestCb_();
                    }
                }
            });
        }

        int err = avformat_find_stream_info(inputCtx_, nullptr);
        if (err < 0) {
            JAMI_ERROR("Unable to find stream info: {}", libav_utils::getError(err));
        }
        streamInfoFound_ = true;
        if (streamInfoTimer_) {
            streamInfoTimer_->cancel();
            streamInfoTimer_.reset();
        }
    }
}

int
MediaDemuxer::selectStream(AVMediaType type)
{
    auto sti = av_find_best_stream(inputCtx_, type, -1, -1, nullptr, 0);
    if (type == AVMEDIA_TYPE_VIDEO && sti >= 0) {
        auto* st = inputCtx_->streams[sti];
        auto disposition = st->disposition;
        if (disposition & AV_DISPOSITION_ATTACHED_PIC) {
            JAMI_LOG("Skipping attached picture stream");
            sti = -1;
        }
    }
    return sti;
}

void
MediaDemuxer::setInterruptCallback(int (*cb)(void*), void* opaque)
{
    if (cb) {
        inputCtx_->interrupt_callback.callback = cb;
        inputCtx_->interrupt_callback.opaque = opaque;
    } else {
        inputCtx_->interrupt_callback.callback = 0;
    }
}
void
MediaDemuxer::setNeedFrameCb(std::function<void()> cb)
{
    needFrameCb_ = std::move(cb);
}

void
MediaDemuxer::setFileFinishedCb(std::function<void(bool)> cb)
{
    fileFinishedCb_ = std::move(cb);
}

void
MediaDemuxer::setKeyFrameRequestCb(std::function<void()> cb)
{
    keyFrameRequestCb_ = std::move(cb);
}

void
MediaDemuxer::clearFrames()
{
    {
        std::lock_guard lk {videoBufferMutex_};
        while (!videoBuffer_.empty()) {
            videoBuffer_.pop();
        }
    }
    {
        std::lock_guard lk {audioBufferMutex_};
        while (!audioBuffer_.empty()) {
            audioBuffer_.pop();
        }
    }
}

bool
MediaDemuxer::emitFrame(bool isAudio)
{
    if (isAudio) {
        return pushFrameFrom(audioBuffer_, isAudio, audioBufferMutex_);
    } else {
        return pushFrameFrom(videoBuffer_, isAudio, videoBufferMutex_);
    }
}

bool
MediaDemuxer::pushFrameFrom(std::queue<std::unique_ptr<AVPacket, std::function<void(AVPacket*)>>>& buffer,
                            bool isAudio,
                            std::mutex& mutex)
{
    std::unique_lock lock(mutex);
    if (buffer.empty()) {
        if (currentState_ == MediaDemuxer::CurrentState::Finished) {
            fileFinishedCb_(isAudio);
        } else {
            needFrameCb_();
        }
        return false;
    }
    auto packet = std::move(buffer.front());
    if (!packet) {
        return false;
    }
    auto streamIndex = packet->stream_index;
    if (static_cast<unsigned>(streamIndex) >= streams_.size() || streamIndex < 0) {
        return false;
    }
    if (auto& cb = streams_[streamIndex]) {
        buffer.pop();
        lock.unlock();
        cb(*packet.get());
    }
    return true;
}

MediaDemuxer::Status
MediaDemuxer::demuxe()
{
    auto packet = std::unique_ptr<AVPacket, std::function<void(AVPacket*)>>(av_packet_alloc(), [](AVPacket* p) {
        if (p)
            av_packet_free(&p);
    });

    bool isVideo;
    {
        std::lock_guard lk(inputCtxMutex_);
        int ret = av_read_frame(inputCtx_, packet.get());
        if (ret == AVERROR(EAGAIN)) {
            return Status::Success;
        } else if (ret == AVERROR_EOF) {
            return Status::EndOfFile;
        } else if (ret < 0) {
            JAMI_ERROR("Unable to read frame: {}", libav_utils::getError(ret));
            return Status::ReadError;
        }

        auto streamIndex = packet->stream_index;
        if (static_cast<unsigned>(streamIndex) >= streams_.size() || streamIndex < 0) {
            return Status::Success;
        }

        isVideo = inputCtx_->streams[streamIndex]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO;
    }

    if (isVideo) {
        std::lock_guard lk {videoBufferMutex_};
        videoBuffer_.push(std::move(packet));
        if (videoBuffer_.size() >= 90) {
            return Status::ReadBufferOverflow;
        }
    } else {
        std::lock_guard lk {audioBufferMutex_};
        audioBuffer_.push(std::move(packet));
        if (audioBuffer_.size() >= 300) {
            return Status::ReadBufferOverflow;
        }
    }
    return Status::Success;
}

void
MediaDemuxer::setIOContext(MediaIOHandle* ioctx)
{
    inputCtx_->pb = ioctx->getContext();
}

MediaDemuxer::Status
MediaDemuxer::decode()
{
    if (inputParams_.format == "x11grab" || inputParams_.format == "dxgigrab") {
        auto ret = inputCtx_->iformat->read_header(inputCtx_);
        if (ret == AVERROR_EXTERNAL) {
            JAMI_ERROR("Unable to read frame: {}\n", libav_utils::getError(ret));
            return Status::ReadError;
        }
        auto* codecpar = inputCtx_->streams[0]->codecpar;
        if (baseHeight_ != codecpar->height || baseWidth_ != codecpar->width) {
            baseHeight_ = codecpar->height;
            baseWidth_ = codecpar->width;
            inputParams_.height = ((baseHeight_ >> 3) << 3);
            inputParams_.width = ((baseWidth_ >> 3) << 3);
            return Status::RestartRequired;
        }
    }

    libjami::PacketBuffer packet(av_packet_alloc());
    int ret = av_read_frame(inputCtx_, packet.get());
    if (ret == AVERROR(EAGAIN)) {
        /*no data available. Calculate time until next frame.
         We do not use the emulated frame mechanism from the decoder because it will affect all
         platforms. With the current implementation, the demuxer will be waiting just in case when
         av_read_frame returns EAGAIN. For some platforms, av_read_frame is blocking and it will
         never happen.
         */
        if (inputParams_.framerate.numerator() == 0)
            return Status::Success;
        rational<double> frameTime = 1e6 / inputParams_.framerate;
        int64_t timeToSleep = lastReadPacketTime_ - av_gettime_relative() + frameTime.real<int64_t>();
        if (timeToSleep <= 0) {
            return Status::Success;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(timeToSleep));
        return Status::Success;
    } else if (ret == AVERROR_EOF) {
        return Status::EndOfFile;
    } else if (ret == AVERROR(EACCES)) {
        return Status::RestartRequired;
    } else if (ret < 0) {
        auto media = inputCtx_->streams[0]->codecpar->codec_type;
        const auto* const type = media == AVMediaType::AVMEDIA_TYPE_AUDIO
                                     ? "AUDIO"
                                     : (media == AVMediaType::AVMEDIA_TYPE_VIDEO ? "VIDEO" : "UNSUPPORTED");
        JAMI_ERROR("Unable to read [{}] frame: {}", type, libav_utils::getError(ret));
        return Status::ReadError;
    }

    auto streamIndex = packet->stream_index;
    if (static_cast<unsigned>(streamIndex) >= streams_.size() || streamIndex < 0) {
        return Status::Success;
    }

    if (inputParams_.format == "sdp"
        && inputCtx_->streams[streamIndex]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
        ++videoPacketsRead_;
        videoBytesRead_ += packet->size;
        videoLastPts_ = packet->pts;
        videoLastDts_ = packet->dts;
        if (packet->pts == AV_NOPTS_VALUE)
            ++videoPacketsMissingPts_;

        const auto now = std::chrono::steady_clock::now();
        if (videoStatsLastLog_ == std::chrono::steady_clock::time_point {}
            || now - videoStatsLastLog_ >= std::chrono::seconds(1)) {
            const auto elapsed = videoStatsLastLog_ == std::chrono::steady_clock::time_point {}
                                     ? 1.0
                                     : std::chrono::duration<double>(now - videoStatsLastLog_).count();
            JAMI_LOG("[RTPDemux:{}] packets={} (+{:.1f}/s) bytes={} (+{:.1f} KB/s) "
                     "missing_pts={} (+{}) last_pts={} last_dts={} packet_size={}",
                     fmt::ptr(this),
                     videoPacketsRead_,
                     (videoPacketsRead_ - videoPacketsLastLog_) / elapsed,
                     videoBytesRead_,
                     (videoBytesRead_ - videoBytesLastLog_) / elapsed / 1024.0,
                     videoPacketsMissingPts_,
                     videoPacketsMissingPts_ - videoMissingPtsLastLog_,
                     videoLastPts_,
                     videoLastDts_,
                     packet->size);
            videoStatsLastLog_ = now;
            videoPacketsLastLog_ = videoPacketsRead_;
            videoMissingPtsLastLog_ = videoPacketsMissingPts_;
            videoBytesLastLog_ = videoBytesRead_;
        }
    }

    lastReadPacketTime_ = av_gettime_relative();

    auto& cb = streams_[streamIndex];
    if (cb) {
        DecodeStatus ret = cb(*packet.get());
        if (ret == DecodeStatus::FallBack)
            return Status::FallBack;
        if (ret == DecodeStatus::DecodeError)
            return Status::ReadError;
    }
    return Status::Success;
}

MediaDecoder::MediaDecoder(const std::shared_ptr<MediaDemuxer>& demuxer, int index)
    : demuxer_(demuxer)
    , avStream_(demuxer->getStream(index))
{
    demuxer->setStreamCallback(index, [this](AVPacket& packet) { return decode(packet); });
    setupStream();
}

MediaDecoder::MediaDecoder(const std::shared_ptr<MediaDemuxer>& demuxer, int index, MediaObserver observer)
    : demuxer_(demuxer)
    , avStream_(demuxer->getStream(index))
    , callback_(std::move(observer))
{
    demuxer->setStreamCallback(index, [this](AVPacket& packet) { return decode(packet); });
    setupStream();
}

bool
MediaDecoder::emitFrame(bool isAudio)
{
    return demuxer_->emitFrame(isAudio);
}

MediaDecoder::MediaDecoder()
    : demuxer_(new MediaDemuxer)
{}

MediaDecoder::MediaDecoder(MediaObserver o)
    : demuxer_(new MediaDemuxer)
    , callback_(std::move(o))
{}

MediaDecoder::~MediaDecoder()
{
#ifdef ENABLE_HWACCEL
    if (decoderCtx_ && decoderCtx_->hw_device_ctx)
        av_buffer_unref(&decoderCtx_->hw_device_ctx);
#endif
    if (decoderCtx_)
        avcodec_free_context(&decoderCtx_);
}

void
MediaDecoder::flushBuffers()
{
    avcodec_flush_buffers(decoderCtx_);
}

int
MediaDecoder::openInput(const DeviceParams& p)
{
    passthrough_ = p.passthrough;
    return demuxer_->openInput(p);
}

void
MediaDecoder::setInterruptCallback(int (*cb)(void*), void* opaque)
{
    demuxer_->setInterruptCallback(cb, opaque);
}

void
MediaDecoder::setIOContext(MediaIOHandle* ioctx)
{
    demuxer_->setIOContext(ioctx);
}

void
MediaDecoder::setKeyFrameRequestCb(std::function<void()> cb)
{
    demuxer_->setKeyFrameRequestCb(std::move(cb));
}

int
MediaDecoder::setup(AVMediaType type)
{
    if (prepare(type) < 0)
        return -1;
    return setupStream();
}

int
MediaDecoder::prepare(AVMediaType type)
{
    demuxer_->findStreamInfo(type == AVMEDIA_TYPE_VIDEO);
    auto stream = demuxer_->selectStream(type);
    if (stream < 0) {
        JAMI_ERROR("No stream found for type {}", static_cast<int>(type));
        return -1;
    }
    avStream_ = demuxer_->getStream(stream);
    if (avStream_ == nullptr) {
        JAMI_ERROR("No stream found at index {}", stream);
        return -1;
    }
    demuxer_->setStreamCallback(stream, [this](AVPacket& packet) { return decode(packet); });
    return 0;
}

int
MediaDecoder::setupStream()
{
    int ret = 0;
    decoderReady_ = false;
    avcodec_free_context(&decoderCtx_);

    if (prepareDecoderContext() < 0)
        return -1; // failed

#ifdef __ANDROID__
    const bool useMediaCodecSurface = nativeWindow_ && decoderCtx_->codec_id == AV_CODEC_ID_H264;
#else
    const bool useMediaCodecSurface = false;
#endif
    bool mediaCodecSurfaceReady = false;

#ifdef ENABLE_HWACCEL
    // if there was a fallback to software decoding, do not enable accel
    // it has been disabled already by the video_receive_thread/video_input
    enableAccel_ &= Manager::instance().videoPreferences.getDecodingAccelerated();

    if (enableAccel_ and not fallback_ and not useMediaCodecSurface) {
        auto APIs = video::HardwareAccel::getCompatibleAccel(decoderCtx_->codec_id,
                                                             decoderCtx_->width,
                                                             decoderCtx_->height,
                                                             CODEC_DECODER);
        for (const auto& it : APIs) {
            accel_ = std::make_unique<video::HardwareAccel>(it); // save accel
            auto ret = accel_->initAPI(false, nullptr);
            if (ret < 0) {
                accel_.reset();
                continue;
            }
            if (prepareDecoderContext() < 0)
                return -1; // failed
            accel_->setDetails(decoderCtx_);
            decoderCtx_->opaque = accel_.get();
            decoderCtx_->pix_fmt = accel_->getFormat();
            if (avcodec_open2(decoderCtx_, inputDecoder_, &options_) < 0) {
                // Failed to open codec
                JAMI_WARNING("Fail to open hardware decoder for {} with {}",
                             avcodec_get_name(decoderCtx_->codec_id),
                             it.getName());
                avcodec_free_context(&decoderCtx_);
                decoderCtx_ = nullptr;
                accel_.reset();
                continue;
            } else {
                // Codec opened successfully.
                JAMI_WARNING("Using hardware decoding for {} with {}",
                             avcodec_get_name(decoderCtx_->codec_id),
                             it.getName());
                break;
            }
        }
    }
#endif

#ifdef __ANDROID__
    if (nativeWindow_ && decoderCtx_->codec_id == AV_CODEC_ID_H264) {
        auto* device = av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_MEDIACODEC);
        if (device) {
            auto* deviceCtx = reinterpret_cast<AVHWDeviceContext*>(device->data);
            auto* mediaCodecCtx = reinterpret_cast<AVMediaCodecDeviceContext*>(deviceCtx->hwctx);
            mediaCodecCtx->surface = surface_;
            mediaCodecCtx->native_window = nativeWindow_;
            mediaCodecCtx->create_window = 0;
            if (av_hwdevice_ctx_init(device) >= 0) {
                decoderCtx_->hw_device_ctx = device;
                mediaCodecSurfaceReady = true;
                JAMI_LOG("Using Android MediaCodec output surface for {} (surface={}, nativeWindow={})",
                         avcodec_get_name(decoderCtx_->codec_id), surface_, nativeWindow_);
            } else {
                JAMI_WARNING("Unable to initialize Android MediaCodec output surface; using buffer output");
                av_buffer_unref(&device);
            }
        } else {
            JAMI_WARNING("Unable to allocate Android MediaCodec device context; using buffer output");
        }
    }
#endif

    JAMI_LOG("Using {} ({}) decoder for {}",
             inputDecoder_->long_name,
             inputDecoder_->name,
             av_get_media_type_string(avStream_->codecpar->codec_type));
    decoderCtx_->thread_count = std::max(1, std::min(8, static_cast<int>(std::thread::hardware_concurrency()) / 2));
    if (emulateRate_)
        JAMI_LOG("Using framerate emulation");
    startTime_ = av_gettime(); // Used to set pts after decoding, and for rate emulation

#ifdef ENABLE_HWACCEL
    if (!accel_) {
        if (!mediaCodecSurfaceReady)
            JAMI_WARNING("Not using hardware decoding for {}", avcodec_get_name(decoderCtx_->codec_id));
        ret = avcodec_open2(decoderCtx_, inputDecoder_, nullptr);
    }
#else
    ret = avcodec_open2(decoderCtx_, inputDecoder_, nullptr);
#endif
    if (ret < 0) {
        JAMI_ERROR("Unable to open codec: {}", libav_utils::getError(ret));
        return -1;
    }

    decoderReady_ = true;
    return 0;
}

int
MediaDecoder::prepareDecoderContext()
{
    inputDecoder_ = findDecoder(avStream_->codecpar->codec_id);
#ifdef __ANDROID__
    // Only select the named MediaCodec decoder when the caller supplied an
    // output window. Without a Surface it uses ByteBuffer output, which is
    // not the zero-copy path needed by the Android TV renderer.
    if (nativeWindow_ && avStream_->codecpar->codec_id == AV_CODEC_ID_H264) {
        if (const auto* mediaCodecDecoder = avcodec_find_decoder_by_name("h264_mediacodec"))
            inputDecoder_ = mediaCodecDecoder;
    }
#endif
    if (!inputDecoder_) {
        JAMI_ERROR("Unsupported codec");
        return -1;
    }

    decoderCtx_ = avcodec_alloc_context3(inputDecoder_);
    if (!decoderCtx_) {
        JAMI_ERROR("Failed to create decoder context");
        return -1;
    }
    avcodec_parameters_to_context(decoderCtx_, avStream_->codecpar);
    decoderCtx_->pkt_timebase = avStream_->time_base;
    width_ = decoderCtx_->width;
    height_ = decoderCtx_->height;
    decoderCtx_->framerate = avStream_->avg_frame_rate;
    if (avStream_->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
        if (decoderCtx_->framerate.num == 0 || decoderCtx_->framerate.den == 0)
            decoderCtx_->framerate = inputParams_.framerate;
        if (decoderCtx_->framerate.num == 0 || decoderCtx_->framerate.den == 0)
            decoderCtx_->framerate = {30, 1};
    } else if (avStream_->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
        if (decoderCtx_->codec_id == AV_CODEC_ID_OPUS) {
            av_opt_set_int(decoderCtx_, "decode_fec", fecEnabled_ ? 1 : 0, AV_OPT_SEARCH_CHILDREN);
        }
        auto format = libav_utils::choose_sample_fmt_default(
            inputDecoder_, Manager::instance().getRingBufferPool().getInternalAudioFormat().sampleFormat);
        decoderCtx_->sample_fmt = format;
        decoderCtx_->request_sample_fmt = format;
    }
    return 0;
}

void
MediaDecoder::updateStartTime(int64_t startTime)
{
    startTime_ = startTime;
}

DecodeStatus
MediaDecoder::decode(AVPacket& packet)
{
    const bool isVideo = inputDecoder_ && inputDecoder_->type == AVMEDIA_TYPE_VIDEO;
    if (isVideo) {
        ++decodePackets_;
        if (packet.pts == AV_NOPTS_VALUE)
            ++decodeMissingPts_;
    }

    if (inputDecoder_->type == AVMEDIA_TYPE_VIDEO && passthrough_) {
#ifdef ENABLE_VIDEO
        auto f = std::static_pointer_cast<MediaFrame>(std::make_shared<VideoFrame>());
        if (auto p = av_packet_clone(&packet))
            f->setPacket(libjami::PacketBuffer(p));
        if (callback_)
            callback_(std::move(f));
        if (contextCallback_ && firstDecode_.load()) {
            firstDecode_.exchange(false);
            contextCallback_();
        }
        return DecodeStatus::FrameFinished;
#endif
    }

    // MediaCodec treats a missing PTS as zero. Give packets arriving through
    // the custom ICE transport a monotonic timestamp instead.
    if (inputDecoder_->type == AVMEDIA_TYPE_VIDEO && packet.pts == AV_NOPTS_VALUE) {
        packet.pts = av_rescale_q(av_gettime() - startTime_,
                                  {1, AV_TIME_BASE},
                                  avStream_->time_base);
        if (packet.dts == AV_NOPTS_VALUE)
            packet.dts = packet.pts;
    }

    bool packetSent = false;
    bool frameFinished = false;
    bool outputForPacket = false;
    int ret = 0;

    auto processFrame = [&](std::shared_ptr<MediaFrame>& f) -> DecodeStatus {
        auto* frame = f->pointer();
        if (inputDecoder_->type == AVMEDIA_TYPE_VIDEO) {
            decoderCtx_->time_base.num = decoderCtx_->framerate.den;
            decoderCtx_->time_base.den = decoderCtx_->framerate.num;
        } else {
            decoderCtx_->time_base.num = 1;
            decoderCtx_->time_base.den = decoderCtx_->sample_rate;
        }
        frame->time_base = decoderCtx_->time_base;

        if (resolutionChangedCallback_ && (decoderCtx_->width != width_ || decoderCtx_->height != height_)) {
            JAMI_LOG("Resolution changed from {}x{} to {}x{}",
                     width_,
                     height_,
                     decoderCtx_->width,
                     decoderCtx_->height);
            width_ = decoderCtx_->width;
            height_ = decoderCtx_->height;
            resolutionChangedCallback_(width_, height_);
        }

        if (inputDecoder_->type == AVMEDIA_TYPE_VIDEO) {
            frame->format = (AVPixelFormat) correctPixFmt(frame->format);
        } else if (frame->ch_layout.order == AV_CHANNEL_ORDER_UNSPEC) {
            av_channel_layout_default(&frame->ch_layout, frame->ch_layout.nb_channels);
        }

        auto packetTimestamp = frame->pts;
        frame->pts = av_rescale_q_rnd(av_gettime() - startTime_,
                                      {1, AV_TIME_BASE},
                                      decoderCtx_->time_base,
                                      static_cast<AVRounding>(AV_ROUND_NEAR_INF | AV_ROUND_PASS_MINMAX));
        lastTimestamp_ = frame->pts;
        if (emulateRate_ && packetTimestamp != AV_NOPTS_VALUE) {
            auto startTime = avStream_->start_time == AV_NOPTS_VALUE ? 0 : avStream_->start_time;
            rational<double> frame_time = rational<double>(getTimeBase())
                                          * rational<double>(static_cast<double>(packetTimestamp - startTime));
            auto target_relative = static_cast<std::int64_t>(frame_time.real() * 1e6);
            auto target_absolute = startTime_ + target_relative;
            if (target_relative < seekTime_)
                return DecodeStatus::Success;
            if (target_relative >= seekTime_)
                resetSeekTime();
            auto now = av_gettime();
            if (target_absolute > now)
                std::this_thread::sleep_for(std::chrono::microseconds(target_absolute - now));
        }

        if (callback_)
            callback_(std::move(f));
        if (contextCallback_ && firstDecode_.load()) {
            firstDecode_.exchange(false);
            contextCallback_();
        }
        return DecodeStatus::FrameFinished;
    };

    for (;;) {
        if (!packetSent) {
            ret = avcodec_send_packet(decoderCtx_, &packet);
            if (ret == AVERROR(EAGAIN)) {
                if (isVideo)
                    ++decodeSendEagain_;
            } else if (ret < 0) {
                if (isVideo)
                    ++decodeSendErrors_;
#ifdef ENABLE_HWACCEL
                if (accel_) {
                    JAMI_WARNING("Decoding error falling back to software");
                    fallback_ = true;
                    accel_.reset();
                    avcodec_flush_buffers(decoderCtx_);
                    setupStream();
                    return DecodeStatus::FallBack;
                }
#endif
                avcodec_flush_buffers(decoderCtx_);
                return ret == AVERROR_EOF ? DecodeStatus::Success : DecodeStatus::DecodeError;
            } else {
                packetSent = true;
            }
        }

#ifdef ENABLE_VIDEO
        auto f = (inputDecoder_->type == AVMEDIA_TYPE_VIDEO)
                     ? std::static_pointer_cast<MediaFrame>(std::make_shared<VideoFrame>())
                     : std::static_pointer_cast<MediaFrame>(std::make_shared<AudioFrame>());
#else
        auto f = std::static_pointer_cast<MediaFrame>(std::make_shared<AudioFrame>());
#endif
        ret = avcodec_receive_frame(decoderCtx_, f->pointer());
        if (ret == 0) {
            frameFinished = true;
            if (isVideo) {
                ++decodeFrames_;
                if (outputForPacket || !packetSent)
                    ++decodeDrainFrames_;
                if (decodeFrames_ == 1) {
                    JAMI_LOG("[VideoDecode:{}] first output frame format={} size={}x{} surface={}",
                             fmt::ptr(this),
                             f->pointer()->format,
                             f->pointer()->width,
                             f->pointer()->height,
                             f->pointer()->format == AV_PIX_FMT_MEDIACODEC);
                }
            }
            outputForPacket = true;
            processFrame(f);
            continue;
        }

        if (ret == AVERROR(EAGAIN)) {
            if (isVideo)
                ++decodeReceiveEagain_;
            if (!packetSent) {
                JAMI_ERROR("[VideoDecode:{}] avcodec_send_packet and avcodec_receive_frame both returned EAGAIN",
                           fmt::ptr(this));
                return DecodeStatus::DecodeError;
            }
            break;
        }
        if (ret == AVERROR_EOF)
            break;

        if (isVideo)
            ++decodeReceiveErrors_;
        JAMI_ERROR("[VideoDecode:{}] avcodec_receive_frame failed: {}",
                   fmt::ptr(this),
                   libav_utils::getError(ret));
        return DecodeStatus::DecodeError;
    }

    if (isVideo)
        logVideoStats();
    return frameFinished ? DecodeStatus::FrameFinished : DecodeStatus::Success;
}

void
MediaDecoder::logVideoStats(bool force)
{
    if (!inputDecoder_ || inputDecoder_->type != AVMEDIA_TYPE_VIDEO)
        return;

    const auto now = std::chrono::steady_clock::now();
    if (!force && decodeStatsLastLog_ != std::chrono::steady_clock::time_point {}
        && now - decodeStatsLastLog_ < std::chrono::seconds(1))
        return;

    const auto elapsed = decodeStatsLastLog_ == std::chrono::steady_clock::time_point {}
                             ? 1.0
                             : std::chrono::duration<double>(now - decodeStatsLastLog_).count();
    JAMI_LOG("[VideoDecode:{}] codec={} surface={} input={} (+{:.1f}/s) output={} (+{:.1f}/s) "
             "missing_pts={} send_errors={} send_eagain={} receive_errors={} receive_eagain={} "
             "drain_frames={} size={}x{} pix_fmt={}",
             fmt::ptr(this),
             inputDecoder_->name ? inputDecoder_->name : "unknown",
             isSurfaceOutput(),
             decodePackets_,
             (decodePackets_ - decodePacketsLastLog_) / elapsed,
             decodeFrames_,
             (decodeFrames_ - decodeFramesLastLog_) / elapsed,
             decodeMissingPts_,
             decodeSendErrors_,
             decodeSendEagain_,
             decodeReceiveErrors_,
             decodeReceiveEagain_,
             decodeDrainFrames_,
             decoderCtx_ ? decoderCtx_->width : 0,
             decoderCtx_ ? decoderCtx_->height : 0,
             decoderCtx_ ? static_cast<int>(decoderCtx_->pix_fmt) : static_cast<int>(AV_PIX_FMT_NONE));
    decodeStatsLastLog_ = now;
    decodePacketsLastLog_ = decodePackets_;
    decodeFramesLastLog_ = decodeFrames_;
}

void
MediaDecoder::setSeekTime(int64_t time)
{
    seekTime_ = time;
}

MediaDemuxer::Status
MediaDecoder::decode()
{
    auto ret = demuxer_->decode();
    if (ret == MediaDemuxer::Status::RestartRequired) {
        avcodec_flush_buffers(decoderCtx_);
        setupStream();
        ret = MediaDemuxer::Status::EndOfFile;
    }
    return ret;
}

#ifdef ENABLE_VIDEO
#ifdef ENABLE_HWACCEL
void
MediaDecoder::enableAccel(bool enableAccel)
{
    enableAccel_ = enableAccel;
    emitSignal<libjami::ConfigurationSignal::HardwareDecodingChanged>(enableAccel_);
    if (!enableAccel) {
        accel_.reset();
        if (decoderCtx_)
            decoderCtx_->opaque = nullptr;
    }
}
#endif

DecodeStatus
MediaDecoder::flush()
{
    AVPacket inpacket;
    av_init_packet(&inpacket);

    int frameFinished = 0;
    int ret = 0;
    ret = avcodec_send_packet(decoderCtx_, &inpacket);
    if (ret < 0 && ret != AVERROR(EAGAIN))
        return ret == AVERROR_EOF ? DecodeStatus::Success : DecodeStatus::DecodeError;

    auto result = std::make_shared<MediaFrame>();
    ret = avcodec_receive_frame(decoderCtx_, result->pointer());
    if (ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF)
        return DecodeStatus::DecodeError;
    if (ret >= 0)
        frameFinished = 1;

    if (frameFinished) {
        av_packet_unref(&inpacket);
        if (callback_)
            callback_(std::move(result));
        return DecodeStatus::FrameFinished;
    }

    return DecodeStatus::Success;
}
#endif // ENABLE_VIDEO

int
MediaDecoder::getWidth() const
{
    if (decoderCtx_)
        return decoderCtx_->width;
    return avStream_ && avStream_->codecpar ? avStream_->codecpar->width : 0;
}

int
MediaDecoder::getHeight() const
{
    if (decoderCtx_)
        return decoderCtx_->height;
    return avStream_ && avStream_->codecpar ? avStream_->codecpar->height : 0;
}

std::string
MediaDecoder::getDecoderName() const
{
    return decoderCtx_ ? decoderCtx_->codec->name : "";
}

rational<double>
MediaDecoder::getFps() const
{
    return {(double) avStream_->avg_frame_rate.num, (double) avStream_->avg_frame_rate.den};
}

rational<unsigned>
MediaDecoder::getTimeBase() const
{
    return {(unsigned) avStream_->time_base.num, (unsigned) avStream_->time_base.den};
}

AVPixelFormat
MediaDecoder::getPixelFormat() const
{
    return isReady() ? decoderCtx_->pix_fmt : AV_PIX_FMT_NONE;
}

bool
MediaDecoder::isSurfaceOutput() const noexcept
{
#ifdef __ANDROID__
    return isReady() && decoderCtx_ && decoderCtx_->pix_fmt == AV_PIX_FMT_MEDIACODEC;
#else
    return false;
#endif
}

int
MediaDecoder::correctPixFmt(int input_pix_fmt)
{
    // https://ffmpeg.org/pipermail/ffmpeg-user/2014-February/020152.html
    int pix_fmt;
    switch (input_pix_fmt) {
    case AV_PIX_FMT_YUVJ420P:
        pix_fmt = AV_PIX_FMT_YUV420P;
        break;
    case AV_PIX_FMT_YUVJ422P:
        pix_fmt = AV_PIX_FMT_YUV422P;
        break;
    case AV_PIX_FMT_YUVJ444P:
        pix_fmt = AV_PIX_FMT_YUV444P;
        break;
    case AV_PIX_FMT_YUVJ440P:
        pix_fmt = AV_PIX_FMT_YUV440P;
        break;
    default:
        pix_fmt = input_pix_fmt;
        break;
    }
    return pix_fmt;
}

MediaStream
MediaDecoder::getStream(const std::string& name) const
{
    if (!decoderCtx_) {
        JAMI_WARNING("No decoder context");
        return {};
    }
    auto ms = MediaStream(name, decoderCtx_, lastTimestamp_);
#ifdef ENABLE_HWACCEL
    // accel_ is null if not using accelerated codecs
    if (accel_)
        ms.format = accel_->getSoftwareFormat();
#endif
    return ms;
}

} // namespace jami
