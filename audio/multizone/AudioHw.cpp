/*
 * Copyright (C) 2013 Texas Instruments
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "AudioHw"
// #define LOG_NDEBUG 0
// #define VERY_VERBOSE_LOGGING
#ifdef VERY_VERBOSE_LOGGING
#define ALOGVV ALOGV
#else
#define ALOGVV(...) do { } while(0)
#endif

#include <cutils/log.h>

#include <media/AudioParameter.h>

#include <AudioHw.h>

namespace android {

AudioStreamOut::AudioStreamOut(AudioHwDevice *hwDev,
                               PcmWriter *writer,
                               const PcmParams &params,
                               const SlotMap &map,
                               audio_devices_t devices)
    : mHwDev(hwDev), mWriter(writer),
      mParams(params), mDevices(devices), mStandby(true)
{
    if (mWriter)
        mStream = new AdaptedOutStream(params, map);
}

int AudioStreamOut::initCheck() const
{
    int ret = 0;

    if (!mHwDev) {
        ALOGE("AudioStreamOut: initCheck() invalid AudioHwDevice");
        ret = -ENODEV;
    }
    else if (!mWriter || !mWriter->initCheck()) {
        ALOGE("AudioStreamOut: initCheck() invalid PCM writer");
        ret = -ENODEV;
    }
    else if (mStream == NULL || !mStream->initCheck()) {
        ALOGE("AudioStreamOut: initCheck() invalid Out Stream");
        ret = -ENODEV;
    }

    ALOGV("AudioStreamOut: init check %d", ret);

    return ret;
}

uint32_t AudioStreamOut::getSampleRate() const
{
    uint32_t rate = mParams.sampleRate;

    ALOGVV("AudioStreamOut: getSampleRate() %u Hz", rate);

    return rate;
}

int AudioStreamOut::setSampleRate(uint32_t rate)
{
    ALOGV("AudioStreamOut: setSampleRate() %u Hz", rate);

    return 0;
}

size_t AudioStreamOut::getBufferSize() const
{
    size_t size;

    /* Take resampling ratio into account and align to the nearest
     * 16 frames as required by the AudioFlinger */
    size = (mParams.frameCount * mParams.sampleRate) / mWriter->getParams().sampleRate;
    size = ((size + 15) & ~15) * mParams.frameSize();

    ALOGVV("AudioStreamOut: getBufferSize() %u bytes", size);

    return size;
}

audio_channel_mask_t AudioStreamOut::getChannels() const
{
    uint32_t channels = mParams.channels;

    ALOGVV("AudioStreamOut: getChannels() %u channels", channels);

    return audio_channel_out_mask_from_count(channels);
}

audio_format_t AudioStreamOut::getFormat() const
{
    uint32_t sampleBits = mParams.sampleBits;

    ALOGVV("AudioStreamOut: getFormat() %u bits/sample", sampleBits);

    switch (sampleBits) {
    case 8:
        return AUDIO_FORMAT_PCM_8_BIT;
    case 24:
        return AUDIO_FORMAT_PCM_8_24_BIT;
    case 32:
        return AUDIO_FORMAT_PCM_32_BIT;
    case 16:
    default:
        return AUDIO_FORMAT_PCM_16_BIT;
    }
}

int AudioStreamOut::setFormat(audio_format_t format)
{
    ALOGV("AudioStreamOut: setFormat() %u bits/sample",
          audio_bytes_per_sample(format) * 8);

    return 0;
}

int AudioStreamOut::standby()
{
    ALOGV("AudioStreamOut: standby()");

    AutoMutex lock(mLock);

    if (!mStandby) {
        mStream->stop();
        mWriter->unregisterStream(mStream);
        mHwDev->mMixer.setPath(mDevices, false);
        mStandby = true;
    }

    return 0;
}

int AudioStreamOut::dump(int fd) const
{
    ALOGV("AudioStreamOut: dump()");
    return 0;
}

audio_devices_t AudioStreamOut::getDevice() const
{
    ALOGV("AudioStreamOut: getDevice()");
    return mDevices;
}

int AudioStreamOut::setParameters(const char *kv_pairs)
{
    ALOGV("AudioStreamOut: setParameters() '%s'", kv_pairs ? kv_pairs : "");

    int ret;

    AudioParameter parms = AudioParameter(String8(kv_pairs));
    String8 key = String8(AudioParameter::keyRouting);
    int device;

    if ((ret = parms.getInt(key, device)) == NO_ERROR) {
        if ((mDevices & AUDIO_DEVICE_OUT_ALL) != (unsigned int)device) {
            standby();
        }
        if (device & ~(mHwDev->getSupportedDevices())) {
            ALOGW("AudioStreamOut: setParameters() device(s) not supported, "
                  "will use default devices");
        }
        else
            mDevices = device;
    }

    return ret;
}

char *AudioStreamOut::getParameters(const char *keys) const
{
    ALOGV("AudioStreamOut::getParameters()");
    return NULL;
}

int AudioStreamOut::addAudioEffect(effect_handle_t effect) const
{
    ALOGV("AudioStreamOut: addAudioEffects()");
    return 0;
}

int AudioStreamOut::removeAudioEffect(effect_handle_t effect) const
{
    ALOGV("AudioStreamOut: removeAudioEffects()");
    return 0;
}

uint32_t AudioStreamOut::getLatency() const
{
    uint32_t latency = (1000 * getBufferSize()) / mWriter->getParams().sampleRate;

    ALOGVV("AudioStreamOut: getLatency() %u ms", latency);

    return latency;
}

int AudioStreamOut::setVolume(float left, float right)
{
    ALOGV("AudioStreamOut: setVolume() left=%.4f right=%.4f", left, right);
    return -ENOSYS;
}

ssize_t AudioStreamOut::write(const void* buffer, size_t bytes)
{
    uint32_t frames = mParams.bytesToFrames(bytes);
    int ret = 0;
    uint32_t usecs = (frames * 1000000) / mParams.sampleRate;

    ALOGVV("AudioStreamOut: write %u frames (%u bytes) buffer %p",
           frames, bytes, buffer);

    AutoMutex lock(mLock);

    if (mStandby) {
        mHwDev->mMixer.setPath(mDevices, true);
        ret = mWriter->registerStream(mStream);
        if (ret) {
            ALOGE("AudioStreamOut: failed to register stream %d", ret);
            return ret;
        }
        ret = mStream->start();
        if (ret) {
            ALOGE("AudioStreamOut: failed to start stream %d", ret);
            mWriter->unregisterStream(mStream);
            usleep(usecs); /* limits the rate of error messages */
            return ret;
        }

        mStandby = false;
    }

    ret = mStream->write(buffer, frames);
    if (ret < 0) {
        ALOGE("AudioStreamOut: failed to write data %d", ret);
        usleep(usecs);
    } else {
        ALOGW_IF(ret != (int)frames,
                 "AudioStreamOut: wrote only %d out of %d requested frames",
                 ret, frames);
        bytes = mParams.framesToBytes(ret);
    }

    return bytes;
}

int AudioStreamOut::getRenderPosition(uint32_t *dsp_frames) const
{
    ALOGV("AudioStreamOut: getRenderPosition()");

    return -EINVAL;
}

int AudioStreamOut::getNextWriteTimestamp(int64_t *timestamp) const
{
    ALOGVV("AudioStreamOut: getNextWriteTimestamp()");

    return -EINVAL;
}

/* ---------------------------------------------------------------------------------------- */

AudioStreamIn::AudioStreamIn(AudioHwDevice *hwDev,
                             PcmReader *reader,
                             const PcmParams &params,
                             const SlotMap &map,
                             audio_devices_t devices)
    : mHwDev(hwDev), mReader(reader), mParams(params), mDevices(devices),
      mSource(AUDIO_SOURCE_DEFAULT), mStandby(true)
{
    if (mReader)
        mStream = new AdaptedInStream(params, map);
}

int AudioStreamIn::initCheck() const
{
    int ret = 0;

    if (!mHwDev) {
        ALOGE("AudioStreamIn: initCheck() invalid AudioHwDevice");
        ret = -ENODEV;
    }
    else if (!mReader || !mReader->initCheck()) {
        ALOGE("AudioStreamIn: initCheck() invalid PCM reader");
        ret = -ENODEV;
    }
    else if (mStream == NULL || !mStream->initCheck()) {
        ALOGE("AudioStreamIn: initCheck() invalid In Stream");
        ret = -ENODEV;
    }

    ALOGV("AudioStreamIn: init check %d", ret);

    return ret;
}

uint32_t AudioStreamIn::getSampleRate() const
{
    ALOGV("AudioStreamIn: getSampleRate()");

    uint32_t rate = mParams.sampleRate;

    return rate;
}

int AudioStreamIn::setSampleRate(uint32_t rate)
{
    ALOGV("AudioStreamIn: setSampleRate() %u Hz", rate);

    return 0;
}

size_t AudioStreamIn::getBufferSize() const
{
    size_t size;

    /* Take resampling ratio into account and align to the nearest
     * 16 frames as required by the AudioFlinger */
    size = (mParams.frameCount * mParams.sampleRate) / mReader->getParams().sampleRate;
    size = ((size + 15) & ~15) * mParams.frameSize();

    ALOGVV("AudioStreamIn: getBufferSize() %u bytes", size);

    return size;
}

audio_channel_mask_t AudioStreamIn::getChannels() const
{
    ALOGV("AudioStreamIn: getChannels()");

    return audio_channel_in_mask_from_count(mParams.channels);
}

audio_format_t AudioStreamIn::getFormat() const
{
    ALOGV("AudioStreamIn: getFormat()");

    return AUDIO_FORMAT_PCM_16_BIT;
}

int AudioStreamIn::setFormat(audio_format_t format)
{
    ALOGV("AudioStreamIn: setFormat()");

    return 0;
}

int AudioStreamIn::standby()
{
    ALOGV("AudioStreamIn: standby()");

    AutoMutex lock(mLock);

    if (!mStandby) {
        mStream->stop();
        mReader->unregisterStream(mStream);
        mHwDev->mMixer.setPath(mDevices, false);
        mStandby = true;
    }

    return 0;
}

int AudioStreamIn::dump(int fd) const
{
    ALOGV("AudioStreamIn: dump()");

    return 0;
}

audio_devices_t AudioStreamIn::getDevice() const
{
    ALOGV("AudioStreamIn: getDevice()");

    return mDevices;
}

int AudioStreamIn::setParameters(const char *kv_pairs)
{
    ALOGV("AudioStreamIn: setParameters() '%s'", kv_pairs ? kv_pairs : "");

    int ret;

    AudioParameter parms = AudioParameter(String8(kv_pairs));
    String8 source_key = String8(AudioParameter::keyInputSource);
    String8 device_key = String8(AudioParameter::keyRouting);
    int source, device;

    if ((ret = parms.getInt(source_key, source)) == NO_ERROR) {
        /* no audio source uses 0 */
        if ((mSource != (unsigned int)source) &&
            (source != 0) &&
            (source < AUDIO_SOURCE_CNT)) {
            ALOGV("AudioStreamIn: setParameters() source changed [%d]->[%d]",
                  mSource, source);
            mSource = (audio_source_t)source;
            /* Nothing to do for AUDIO_PARAMETER_STREAM_INPUT_SOURCE, so only
             * record the source and continue */
        }
    }

    if ((ret = parms.getInt(device_key, device)) == NO_ERROR) {
        if ((mDevices & AUDIO_DEVICE_IN_ALL) != (unsigned int)device) {
            standby();
        }
        if (device & ~(mHwDev->getSupportedDevices())) {
            ALOGW("AudioStreamIn: setParameters() device(s) not supported, "
                  "will use default devices");
        }
        else {
            mDevices = device;
            ALOGV("AudioStreamIn: setParameters() device set to [0x%x]",
                  mDevices);
        }
    }

    return 0;
}

char *AudioStreamIn::getParameters(const char *keys) const
{
    ALOGV("AudioStreamIn: getParameters()");

    return NULL;
}

int AudioStreamIn::addAudioEffect(effect_handle_t effect) const
{
    ALOGV("AudioStreamIn: addAudioEffect()");

    return 0;
}

int AudioStreamIn::removeAudioEffect(effect_handle_t effect) const
{
    ALOGV("AudioStreamIn: removeAudioEffect()");

    return 0;
}

int AudioStreamIn::setGain(float gain)
{
    ALOGV("AudioStreamIn: setGain()");

    return 0;
}

ssize_t AudioStreamIn::read(void* buffer, size_t bytes)
{
    uint32_t frames = mParams.bytesToFrames(bytes);
    int ret = 0;
    uint32_t usecs = (frames * 1000000) / mParams.sampleRate;

    ALOGVV("AudioStreamIn: read %u frames (%u bytes) buffer %p",
           frames, bytes, buffer);

    AutoMutex lock(mLock);

    if (mStandby) {
        mHwDev->mMixer.setPath(mDevices, true);
        ret = mReader->registerStream(mStream);
        if (ret) {
            ALOGE("AudioStreamIn: failed to register Dest %d", ret);
            return ret;
        }
        ret = mStream->start();
        if (ret) {
            ALOGE("AudioStreamIn: failed to start stream %d", ret);
            mReader->unregisterStream(mStream);
            usleep(usecs); /* limits the rate of error messages */
            return ret;
        }

        mStandby = false;
    }

    ret = mStream->read(buffer, frames);
    if (ret < 0) {
        ALOGE("AudioStreamIn: failed to read data %d", ret);
        uint32_t usecs = (frames * 1000000) / mParams.sampleRate;
        usleep(usecs);
        bytes = ret;
    } else {
        ALOGW_IF(ret != (int)frames,
                 "AudioStreamIn: read only %d out of %d requested frames",
                 ret, frames);
        bytes = mParams.framesToBytes(ret);
        if (mHwDev->mMicMute)
            memset(buffer, 0, bytes);
    }

    return bytes;
}

uint32_t AudioStreamIn::getInputFramesLost()
{
    ALOGVV("AudioStreamIn: getInputFrameLost()");

    return 0;
}

/* ---------------------------------------------------------------------------------------- */

AudioHwDevice::AudioHwDevice(uint32_t card)
    : mCardId(card), mMixer(mCardId), mMicMute(false)
{
    ALOGI("AudioHwDevice: create hw device for card hw:%u", card);

    /* Mixer for dra7evm and input/output ports for JAMR3 PCM device */
    for (uint32_t i = 0; i < kNumPorts; i++) {
        ALSAInPort *inPort = new ALSAInPort(mCardId, i);
        mInPorts.push_back(inPort);

        ALSAOutPort *outPort = new ALSAOutPort(mCardId, i);
        mOutPorts.push_back(outPort);
    }

    /* PCM parameters for the port associated with on-board audio:
     * 2 channels, 16-bits/sample, 44.1kHz, buffer of 882 frames (capture) */
    PcmParams params0(kCPUNumChannels, kSampleSize, kSampleRate, kCaptureFrameCount);
    PcmReader *reader = new PcmReader(mInPorts[kCPUPortId], params0);
    mReaders.push_back(reader);
    /* 2 channels, 16-bits/sample, 44.1kHz, buffer of 1024 frames (playback) */
    params0.frameCount = kPlaybackFrameCount;
    PcmWriter *writer = new PcmWriter(mOutPorts[kCPUPortId], params0);
    mWriters.push_back(writer);

    /* PCM parameters for the port associated with JAMR3 audio:
     * 8 channels, 16-bits/sample, 44.1kHz, buffer of 882 frames (capture) */
    PcmParams params1(kJAMR3NumChannels, kSampleSize, kSampleRate, kCaptureFrameCount);
    reader = new PcmReader(mInPorts[kJAMR3PortId], params1);
    mReaders.push_back(reader);
    /* 8 channels, 16-bits/sample, 44.1kHz, buffer of 1024 frames (playback) */
    params1.frameCount = kPlaybackFrameCount;
    writer = new PcmWriter(mOutPorts[kJAMR3PortId], params1);
    mWriters.push_back(writer);

    mMixer.initRoutes();
}

AudioHwDevice::~AudioHwDevice()
{
    ALOGI("AudioHwDevice: destroy hw device for card hw:%u", mCardId);

    for (WriterVect::const_iterator i = mWriters.begin(); i != mWriters.end(); ++i) {
        delete (*i);
    }
    for (ReaderVect::const_iterator i = mReaders.begin(); i != mReaders.end(); ++i) {
        delete (*i);
    }
    for (OutPortVect::iterator i = mOutPorts.begin(); i != mOutPorts.end(); ++i) {
        delete (*i);
    }
    for (InPortVect::iterator i = mInPorts.begin(); i != mInPorts.end(); ++i) {
        delete (*i);
    }
}

uint32_t AudioHwDevice::getSupportedDevices() const
{
    uint32_t devices;

    devices = AUDIO_DEVICE_IN_BUILTIN_MIC |
              AUDIO_DEVICE_IN_BACK_MIC |
              AUDIO_DEVICE_IN_VOICE_CALL |
              AUDIO_DEVICE_OUT_SPEAKER |
              AUDIO_DEVICE_OUT_WIRED_HEADPHONE |
              AUDIO_DEVICE_OUT_WIRED_HEADSET |
              AUDIO_DEVICE_OUT_WIRED_HEADPHONE2;
    ALOGV("AudioHwDevice: supported devices 0x%08x", devices);

    return devices;
}

int AudioHwDevice::initCheck() const
{
    if (!mMixer.initCheck()) {
        ALOGE("AudioHwDevice: ALSA mixer init failed");
        return -ENODEV;
    }

    for (ReaderVect::const_iterator i = mReaders.begin(); i != mReaders.end(); ++i) {
        if (!((*i)->initCheck())) {
            ALOGE("AudioHwDevice: PCM reader initCheck failed");
            return -ENODEV;
        }
    }
    for (WriterVect::const_iterator i = mWriters.begin(); i != mWriters.end(); ++i) {
        if (!((*i)->initCheck())) {
            ALOGE("AudioHwDevice: PCM writer init failed");
            return -ENODEV;
        }
    }

    return 0;
}

int AudioHwDevice::setVoiceVolume(float volume)
{
    ALOGV("AudioHwDevice: setVoiceVolume() vol=%.4f", volume);
    return -ENOSYS;
}

int AudioHwDevice::setMasterVolume(float volume)
{
    ALOGV("AudioHwDevice: setMasterVolume() vol=%.4f", volume);
    return -ENOSYS;
}

const char *AudioHwDevice::getModeName(audio_mode_t mode) const
{
    switch (mode) {
    case AUDIO_MODE_CURRENT:
        return "CURRENT";
    case AUDIO_MODE_NORMAL:
        return "NORMAL";
    case AUDIO_MODE_RINGTONE:
        return "RINGTONE";
    case AUDIO_MODE_IN_CALL:
        return "IN_CALL";
    case AUDIO_MODE_IN_COMMUNICATION:
        return "COMMUNICATION";
    default:
        return "INVALID";
    }
}

int AudioHwDevice::setMode(audio_mode_t mode)
{
    ALOGV("AudioHwDevice: setMode() %s", getModeName(mode));

    return 0;
}

int AudioHwDevice::setMicMute(bool state)
{
    ALOGV("AudioHwDevice: setMicMute() %s", state ? "mute" : "unmute");

    mMicMute = state;

    return 0;
}

int AudioHwDevice::getMicMute(bool *state) const
{
    ALOGV("AudioHwDevice: getMicMute()");

    *state = mMicMute;

    return 0;
}

int AudioHwDevice::setParameters(const char *kv_pairs)
{
    ALOGV("AudioHwDevice: setParameters() '%s'", kv_pairs ? kv_pairs : "");

    return 0;
}

char *AudioHwDevice::getParameters(const char *keys) const
{
    ALOGV("AudioHwDevice: getParameters()");

    return NULL;
}

size_t AudioHwDevice::getInputBufferSize(const struct audio_config *config) const
{
    ALOGV("AudioHwDevice: getInputBufferSize()");

    size_t size;

    /* Take resampling ratio into account and align to the nearest
     * 16 frames as required by the AudioFlinger */
    /* Use port 0 for the calculation, since values for both ports are the same */
    uint32_t frames = mReaders[kCPUPortId]->getParams().frameCount;
    uint32_t rate = mReaders[kCPUPortId]->getParams().sampleRate;

    size = (frames * config->sample_rate) / rate;
    size = ((size + 15) & ~15) * mReaders[kCPUPortId]->getParams().frameSize();

    ALOGV("AudioHwDevice: getInputBufferSize() %d bytes", size);

    return size;
}

int AudioHwDevice::dump(int fd) const
{
    ALOGV("AudioHwDevice: dump()");

    return 0;
}

int AudioHwDevice::setMasterMute(bool mute)
{
    ALOGV("AudioHwDevice: setMasterMute() %s", mute ? "mute" : "unmute");
    return -ENOSYS;
}

AudioStreamIn* AudioHwDevice::openInputStream(audio_io_handle_t handle,
                                              audio_devices_t devices,
                                              struct audio_config *config)
{
    uint32_t port = 0;
    uint32_t channels = popcount(config->channel_mask);

    ALOGV("AudioHwDevice: openInputStream()");

    uint32_t srcMask, dstMask;
    switch (devices) {
    case AUDIO_DEVICE_IN_BUILTIN_MIC:
    case AUDIO_DEVICE_IN_VOICE_CALL:
        if (channels == 1) {
            /* Mic is in slots 0&1 (mask = 0x03) on port 0, but AF wants
             * only mono so take only one channel here */
            srcMask = 0x01;
            dstMask = 0x01;
        }
        else {
            srcMask = 0x03;
            dstMask = 0x03;
        }
        port = 0;
        break;
    case AUDIO_DEVICE_IN_BACK_MIC:
        if (channels == 1) {
            srcMask = 0x08;
            dstMask = 0x01;
        }
        else {
            ALOGE("AudioHwDevice: device 0x%08x only supports 1 channel",
                  devices);
            return NULL;
        }
        port = 1;
        break;
    default:
        ALOGE("AudioHwDevice: device 0x%08x is not supported", devices);
        return NULL;
    }

    SlotMap slotMap(srcMask, dstMask);

    /* Set the parameters for the internal input stream. Don't change the
     * parameters for capture. The resampler is used if needed. */
    PcmParams params(*config, mReaders[port]->getParams().frameCount);

    sp<AudioStreamIn> in = new AudioStreamIn(this, mReaders[port], params,
                                             slotMap, devices);
    if ((in == NULL) || in->initCheck()) {
        ALOGE("AudioHwDevice: failed to open input stream on port hw:%u,%u",
              mCardId, port);
        return NULL;
    }

    mInStreams.insert(in);

    return in.get();
}

void AudioHwDevice::closeInputStream(AudioStreamIn *in)
{
    ALOGV("AudioHwDevice: closeInputStream()");

    if (mInStreams.find(in) == mInStreams.end()) {
        ALOGW("AudioHwDevice: input stream %p is not open", in);
        return;
    }

    mInStreams.erase(in);

    in = NULL;
}

AudioStreamOut* AudioHwDevice::openOutputStream(audio_io_handle_t handle,
                                                audio_devices_t devices,
                                                audio_output_flags_t flags,
                                                struct audio_config *config)
{
    uint32_t port = 0;
    PcmParams params;

    ALOGV("AudioHwDevice: openOutputStream()");

    uint32_t destMask;
    switch (devices) {
    case AUDIO_DEVICE_OUT_SPEAKER:
        port = 0;
        destMask = 0x03;
        break;
    case AUDIO_DEVICE_OUT_WIRED_HEADPHONE:
    case AUDIO_DEVICE_OUT_WIRED_HEADSET:
        port = 1;
        destMask = 0x0c;
        break;
    case AUDIO_DEVICE_OUT_WIRED_HEADPHONE2:
        port = 1;
        destMask = 0x30;
        break;
    default:
        ALOGE("AudioHwDevice: device 0x%08x is not supported", devices);
        return NULL;
    }

    SlotMap slotMap(0x03, destMask);
    if (!slotMap.isValid()) {
        ALOGE("AudioHwDevice: failed to create slot map");
        return NULL;
    }

    /* Set the parameters for the internal output stream */
    params.frameCount = mWriters[port]->getParams().frameCount;
    params.sampleRate = config->sample_rate; /* Use stream's resampler if needed */
    params.sampleBits = 16;                  /* 16-bits/sample */
    params.channels = 2;                     /* Listening zones are stereo */

    /* Update audio config with granted parameters */
    if (popcount(config->channel_mask) != (int)params.channels) {
        ALOGV("AudioHwDevice: updating audio config channel mask [0x%x]->[0x%x]",
              config->channel_mask,
              audio_channel_out_mask_from_count(params.channels));
    }
    config->channel_mask = audio_channel_out_mask_from_count(params.channels);
    if (config->format != AUDIO_FORMAT_PCM_16_BIT) {
        ALOGV("AudioHwDevice: updating audio config format [0x%x]->[0x%x]",
              config->format, AUDIO_FORMAT_PCM_16_BIT);
    }
    config->format = AUDIO_FORMAT_PCM_16_BIT;

    sp<AudioStreamOut> out = new AudioStreamOut(this, mWriters[port], params,
                                                slotMap, devices);
    if ((out == NULL) || out->initCheck()) {
        ALOGE("AudioHwDevice: failed to open output stream on port hw:%u,%u",
              mCardId, port);
        return NULL;
    }

    mOutStreams.insert(out);

    return out.get();
}

void AudioHwDevice::closeOutputStream(AudioStreamOut *out)
{
    ALOGV("AudioHwDevice: closeOutputStream()");

    if (mOutStreams.find(out) == mOutStreams.end()) {
        ALOGW("AudioHwDevice: output stream %p is not open", out);
        return;
    }

    mOutStreams.erase(out);

    out = NULL;
}

}; /* namespace android */
