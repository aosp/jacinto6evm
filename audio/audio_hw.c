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

#define LOG_TAG "audio_hw_primary"
//#define LOG_NDEBUG 0
//#define VERY_VERBOSE_LOGGING
#ifdef VERY_VERBOSE_LOGGING
#define ALOGVV ALOGV
#else
#define ALOGVV(a...) do { } while(0)
#endif

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/time.h>

#include <cutils/log.h>
#include <cutils/str_parms.h>
#include <cutils/properties.h>

#include <audio_utils/resampler.h>
#include <system/audio.h>
#include <hardware/hardware.h>
#include <hardware/audio.h>
#include <hardware/audio_effect.h>

#include <tinyalsa/asoundlib.h>

/* yet another definition of ARRAY_SIZE macro) */
#define ARRAY_SIZE(x)           (sizeof(x)/sizeof(x[0]))

/* buffer_remix: functor for doing in-place buffer manipulations.
 *
 * NB. When remix_func is called, the memory at `buf` must be at least
 * as large as frames * sample_size * MAX(in_chans, out_chans).
 */
struct buffer_remix {
    void (*remix_func)(struct buffer_remix *data, void *buf, size_t frames);
    size_t sample_size; /* size of one audio sample, in bytes */
    size_t in_chans;    /* number of input channels */
    size_t out_chans;   /* number of output channels */
};

struct j6_audio_device {
    struct audio_hw_device device;
    struct j6_stream_in *in;
    struct j6_stream_out *out;
    struct mixer *mixer;
    audio_devices_t in_device;
    audio_devices_t out_device;
    pthread_mutex_t lock;
    unsigned int card;
    unsigned int in_port;
    unsigned int out_port;
    bool mic_mute;
};

struct j6_stream_in {
    struct audio_stream_in stream;
    struct j6_audio_device *dev;
    struct pcm_config config;
    struct pcm *pcm;
    struct buffer_remix *remix; /* adapt hw chan count to client */
    struct resampler_itfe *resampler;
    struct resampler_buffer_provider buf_provider;
    int16_t *buffer;
    size_t frames_in;
    size_t hw_frame_size;
    unsigned int requested_rate;
    unsigned int requested_channels;
    int read_status;
    pthread_mutex_t lock;
    bool standby;
};

struct j6_stream_out {
    struct audio_stream_out stream;
    struct j6_audio_device *dev;
    struct pcm_config config;
    struct pcm *pcm;
    pthread_mutex_t lock;
    bool standby;
};


static const char *supported_cards[] = {
    "dra7evm",
    "VayuEVM",
};

#define SUPPORTED_IN_DEVICES           (AUDIO_DEVICE_IN_BUILTIN_MIC | \
                                        AUDIO_DEVICE_IN_WIRED_HEADSET | \
                                        AUDIO_DEVICE_IN_DEFAULT)
#define SUPPORTED_OUT_DEVICES          (AUDIO_DEVICE_OUT_SPEAKER | \
                                        AUDIO_DEVICE_OUT_WIRED_HEADSET | \
                                        AUDIO_DEVICE_OUT_WIRED_HEADPHONE | \
                                        AUDIO_DEVICE_OUT_DEFAULT)

#define CAPTURE_SAMPLE_RATE             44100
#define CAPTURE_PERIOD_SIZE             320
#define CAPTURE_PERIOD_COUNT            2
#define CAPTURE_BUFFER_SIZE             (CAPTURE_PERIOD_SIZE * CAPTURE_PERIOD_COUNT)

#define PLAYBACK_SAMPLE_RATE            44100
#define PLAYBACK_PERIOD_SIZE            960
#define PLAYBACK_PERIOD_COUNT           4
#define PLAYBACK_BUFFER_SIZE            (PLAYBACK_PERIOD_SIZE * PLAYBACK_PERIOD_COUNT)

struct pcm_config pcm_config_capture = {
    .channels        = 2,
    .rate            = CAPTURE_SAMPLE_RATE,
    .format          = PCM_FORMAT_S16_LE,
    .period_size     = CAPTURE_PERIOD_SIZE,
    .period_count    = CAPTURE_PERIOD_COUNT,
    .start_threshold = 1,
    .stop_threshold  = CAPTURE_BUFFER_SIZE,
};

struct pcm_config pcm_config_playback = {
    .channels        = 2,
    .rate            = PLAYBACK_SAMPLE_RATE,
    .format          = PCM_FORMAT_S16_LE,
    .period_size     = PLAYBACK_PERIOD_SIZE,
    .period_count    = PLAYBACK_PERIOD_COUNT,
    .start_threshold = PLAYBACK_BUFFER_SIZE / 2,
    .stop_threshold  = PLAYBACK_BUFFER_SIZE,
    .avail_min       = PLAYBACK_PERIOD_SIZE,
};

/* Mixer control names */

/* Capture */
#define MIXER_PGA_CAPTURE_SWITCH        "PGA Capture Switch"
#define MIXER_PGA_CAPTURE_VOLUME        "PGA Capture Volume"

/* Capture gain (0dB, 59.5dB) step=0.5 */
#define CAPTURE_DB_TO_VAL(x)            ((int)(2 * (float)(x)))

/* Microphone specific */
#define MIXER_LEFT_PGA_MIC3L_SWITCH     "Left PGA Mixer Mic3L Switch"
#define MIXER_RIGHT_PGA_MIC3R_SWITCH    "Right PGA Mixer Mic3R Switch"

/* Line-In specific */
#define MIXER_LEFT_PGA_LINE1L_SWITCH    "Left PGA Mixer Line1L Switch"
#define MIXER_RIGHT_PGA_LINE1R_SWITCH   "Right PGA Mixer Line1R Switch"
#define MIXER_LEFT_LINE1L_MUX           "Left Line1L Mux"
#define MIXER_RIGHT_LINE1L_MUX          "Right Line1L Mux"
#define MIXER_LEFT_LINE1R_MUX           "Left Line1R Mux"
#define MIXER_RIGHT_LINE1R_MUX          "Right Line1R Mux"
#define MIXER_LINE_IN_SINGLE_ENDED      "single-ended"
#define MIXER_LINE_IN_DIFFERENTIAL      "differential"

/* Playback */
#define MIXER_LEFT_DAC_MUX              "Left DAC Mux"
#define MIXER_RIGHT_DAC_MUX             "Right DAC Mux"
#define MIXER_MUX_DAC_L1                "DAC_L1"
#define MIXER_MUX_DAC_R1                "DAC_R1"
#define MIXER_PCM_PLAYBACK_VOLUME       "PCM Playback Volume"

 /* Playback gain (-63.5dB, 0dB) step=0.5dB */
#define PLAYBACK_DB_TO_VAL(x)           ((int)(2 * ((float)(x) + 63.5)))

/* Headphone specific */
#define MIXER_HP_PLAYBACK_SWITCH        "HP Playback Switch"
#define MIXER_LEFT_HP_DACL1_SWITCH      "Left HP Mixer DACL1 Switch"
#define MIXER_RIGHT_HP_DACR1_SWITCH     "Right HP Mixer DACR1 Switch"
#define HP_PLAYBACK_VOLUME              "HP DAC Playback Volume"
#define HP_DRIVER_POWER_ON_TIME         "Output Driver Power-On time"
#define HP_DRIVER_50MS_DELAY            "50ms"

/* Line-Out specific */
#define MIXER_LINE_PLAYBACK_SWITCH      "Line Playback Switch"
#define MIXER_LEFT_LINE_DACL1_SWITCH    "Left Line Mixer DACL1 Switch"
#define MIXER_RIGHT_LINE_DACR1_SWITCH   "Right Line Mixer DACR1 Switch"
#define LINE_PLAYBACK_VOLUME            "Line DAC Playback Volume"

 /* Output stage gain (-59.0dB, 0dB) step=0.5dB */
#define OUTPUT_DB_TO_VAL(x)             ((int)(2 * ((float)(x) + 59.0)))

struct route_setting {
    char *ctl_name;
    int intval;
    char *strval;
};

#define RS_STR(ctrl, str) { .ctl_name = ctrl, .strval = str }
#define RS_INT(ctrl, val) { .ctl_name = ctrl, .intval = val }
#define RS_END            { .ctl_name = NULL, .strval = NULL }

/* These are values that never change */
static struct route_setting rs_defaults[] = {
    /* Capture: Mic */
    RS_INT(MIXER_LEFT_PGA_MIC3L_SWITCH, 1),
    RS_INT(MIXER_RIGHT_PGA_MIC3R_SWITCH, 1),
    RS_INT(MIXER_PGA_CAPTURE_SWITCH, 1),
    RS_INT(MIXER_PGA_CAPTURE_VOLUME, CAPTURE_DB_TO_VAL(12)), /* 12dB */
    RS_STR(MIXER_LEFT_LINE1L_MUX, MIXER_LINE_IN_SINGLE_ENDED),
    RS_STR(MIXER_RIGHT_LINE1L_MUX, MIXER_LINE_IN_SINGLE_ENDED),
    RS_STR(MIXER_LEFT_LINE1R_MUX, MIXER_LINE_IN_SINGLE_ENDED),
    RS_STR(MIXER_RIGHT_LINE1R_MUX, MIXER_LINE_IN_SINGLE_ENDED),

    /* Playback: Line-Out and Headphone */
    RS_STR(MIXER_LEFT_DAC_MUX, MIXER_MUX_DAC_L1),
    RS_STR(MIXER_RIGHT_DAC_MUX, MIXER_MUX_DAC_R1),
    RS_INT(MIXER_PCM_PLAYBACK_VOLUME, PLAYBACK_DB_TO_VAL(0)), /* 0dB */

    RS_INT(MIXER_LEFT_HP_DACL1_SWITCH, 1),
    RS_INT(MIXER_RIGHT_HP_DACR1_SWITCH, 1),
    RS_INT(MIXER_HP_PLAYBACK_SWITCH, 1),
    RS_INT(HP_PLAYBACK_VOLUME, OUTPUT_DB_TO_VAL(0)), /* 0 dB */
    RS_STR(HP_DRIVER_POWER_ON_TIME, HP_DRIVER_50MS_DELAY),

    RS_INT(MIXER_LEFT_LINE_DACL1_SWITCH, 1),
    RS_INT(MIXER_RIGHT_LINE_DACR1_SWITCH, 1),
    RS_INT(MIXER_LINE_PLAYBACK_SWITCH, 1),
    RS_INT(LINE_PLAYBACK_VOLUME, OUTPUT_DB_TO_VAL(0)), /* 0 dB */
};

/* Capture switch used for mic mute */
static struct route_setting rs_capture[] = {
    RS_INT(MIXER_PGA_CAPTURE_SWITCH, 1),
    RS_END
};

/*
 * The enable flag when 0 makes the assumption that enums are disabled by
 * "Off" and integers/booleans by 0
 */
static int set_route_by_array(struct mixer *mixer, struct route_setting *route,
                              int enable)
{
    struct mixer_ctl *ctl;
    unsigned int i, j;

    ALOGV("set_route_by_array() route=%p %s", route, enable ? "enable" : "disable");

    /* Go through the route array and set each value */
    i = 0;
    while (route[i].ctl_name) {
        ctl = mixer_get_ctl_by_name(mixer, route[i].ctl_name);
        if (!ctl) {
            ALOGE("set_route_by_array() failed to get control '%s'", route[i].ctl_name);
            return -EINVAL;
        }

        if (route[i].strval) {
            ALOGV("set_route_by_array() control='%s' val='%s'",
                  route[i].ctl_name, enable ? route[i].strval: "Off");

            if (enable)
                mixer_ctl_set_enum_by_string(ctl, route[i].strval);
            else
                mixer_ctl_set_enum_by_string(ctl, "Off");
        } else {
            ALOGV("set_route_by_array() control='%s' val=%d",
                  route[i].ctl_name, route[i].intval);

            /* This ensures multiple (i.e. stereo) values are set jointly */
            for (j = 0; j < mixer_ctl_get_num_values(ctl); j++) {
                if (enable)
                    mixer_ctl_set_value(ctl, j, route[i].intval);
                else
                    mixer_ctl_set_value(ctl, j, 0);
            }
        }
        i++;
    }

    return 0;
}

static int find_supported_card(void)
{
    char name[256] = "";
    int card = 0;
    int found = 0;
    unsigned int i;

    do {
        /* returns an error after last valid card */
        int ret = mixer_get_card_name(card, name, sizeof(name));
        if (ret)
            break;

        for (i = 0; i < ARRAY_SIZE(supported_cards); ++i) {
            if (supported_cards[i] && !strcmp(name, supported_cards[i])) {
                ALOGV("Supported card '%s' found at %d", name, card);
                found = 1;
                break;
            }
        }
    } while (!found && (card++ < MAX_CARD_COUNT));

    /* Use default card number if not found */
    if (!found)
        card = 1;

    return card;
}

/* must be called with device lock held */
static void select_input_device(struct j6_audio_device *adev)
{
    if (adev->in_device & ~SUPPORTED_IN_DEVICES)
        ALOGW("select_input_device() device not supported, will use default device");
}

/* must be called with device lock held */
static void select_output_device(struct j6_audio_device *adev)
{
    if (adev->out_device & ~SUPPORTED_OUT_DEVICES)
        ALOGW("select_output_device() device(s) not supported, will use default devices");
}

static size_t get_input_buffer_size(uint32_t sample_rate, int format, int channel_count)
{
    size_t size;

    /*
     * take resampling into account and return the closest majoring
     * multiple of 16 frames, as audioflinger expects audio buffers to
     * be a multiple of 16 frames
     */
    size = (pcm_config_capture.period_size * sample_rate) / pcm_config_capture.rate;
    size = ((size + 15) / 16) * 16;

    return size * channel_count * sizeof(int16_t);
}

/*
 * Implementation of buffer_remix::remix_func that removes
 * channels in place without doing any other processing.  The
 * extra channels are truncated.
 */
static void remove_channels_from_buf(struct buffer_remix *data, void *buf, size_t frames)
{
    size_t samp_size, in_frame, out_frame;
    size_t N, c;
    char *s, *d;

    ALOGVV("remove_channels_from_buf() remix=%p buf=%p frames=%u",
           data, buf, frames);

    if (frames == 0)
        return;

    samp_size = data->sample_size;
    in_frame = data->in_chans * samp_size;
    out_frame = data->out_chans * samp_size;

    if (out_frame >= in_frame) {
        ALOGE("BUG: remove_channels_from_buf() can not add channels to a buffer.\n");
        return;
    }

    N = frames - 1;
    d = (char*)buf + out_frame;
    s = (char*)buf + in_frame;

    /* take the first several channels and truncate the rest */
    while (N--) {
        for (c = 0; c < out_frame; ++c)
            d[c] = s[c];
        d += out_frame;
        s += in_frame;
    }
}

static int setup_stereo_to_mono_input_remix(struct j6_stream_in *in)
{
    ALOGV("setup_stereo_to_mono_input_remix() stream=%p", in);

    struct buffer_remix *br = (struct buffer_remix *)malloc(sizeof(struct buffer_remix));
    if (!br)
        return -ENOMEM;

    br->remix_func = remove_channels_from_buf;
    br->sample_size = sizeof(int16_t);
    br->in_chans = 2;
    br->out_chans = 1;
    in->remix = br;

    return 0;
}

/* audio HAL functions */

static uint32_t out_get_sample_rate(const struct audio_stream *stream)
{
    uint32_t rate = PLAYBACK_SAMPLE_RATE;

    ALOGVV("out_get_sample_rate() stream=%p rate=%u", stream, rate);

    return rate;
}

static int out_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    ALOGVV("out_set_sample_rate() stream=%p rate=%u", stream, rate);

    return 0;
}

static size_t out_get_buffer_size(const struct audio_stream *stream)
{
    uint32_t frames = ((PLAYBACK_PERIOD_SIZE + 15) / 16) * 16;
    size_t bytes = frames * audio_stream_frame_size(stream);

    ALOGVV("out_get_buffer_size() stream=%p frames=%u bytes=%u", stream, frames, bytes);

    return bytes;
}

static audio_channel_mask_t out_get_channels(const struct audio_stream *stream)
{
    audio_channel_mask_t channels = AUDIO_CHANNEL_OUT_STEREO;

    ALOGVV("out_get_channels() stream=%p channels=%u", stream, popcount(channels));

    return channels;
}

static audio_format_t out_get_format(const struct audio_stream *stream)
{
    audio_format_t format = AUDIO_FORMAT_PCM_16_BIT;

    ALOGVV("out_set_format() stream=%p format=0x%08x (%u bits/sample)",
           stream, format, audio_bytes_per_sample(format) << 3);

    return format;
}

static int out_set_format(struct audio_stream *stream, audio_format_t format)
{
    ALOGVV("out_set_format() stream=%p format=0x%08x (%u bits/sample)",
          stream, format, audio_bytes_per_sample(format) << 3);

    if (format != AUDIO_FORMAT_PCM_16_BIT) {
        return -ENOSYS;
    } else {
        return 0;
    }
}

/* must be called with locks held */
static void do_out_standby(struct j6_stream_out *out)
{
    struct j6_audio_device *adev = out->dev;

    if (!out->standby) {
        ALOGI("do_out_standby() close card %u port %u", adev->card, adev->out_port);
        pcm_close(out->pcm);
        out->pcm = NULL;
        out->standby = true;
    }
}

static int out_standby(struct audio_stream *stream)
{
    struct j6_stream_out *out = (struct j6_stream_out *)(stream);
    struct j6_audio_device *adev = out->dev;

    ALOGV("out_standby() stream=%p", out);
    pthread_mutex_lock(&adev->lock);
    pthread_mutex_lock(&out->lock);
    do_out_standby(out);
    pthread_mutex_unlock(&out->lock);
    pthread_mutex_unlock(&adev->lock);

    return 0;
}

static int out_dump(const struct audio_stream *stream, int fd)
{
    return 0;
}

static int out_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    struct j6_stream_out *out = (struct j6_stream_out *)(stream);
    struct j6_audio_device *adev = out->dev;
    struct str_parms *parms;
    char value[32];
    int ret;
    uint32_t val = 0;

    ALOGV("out_set_parameters() stream=%p parameter='%s'", out, kvpairs);

    parms = str_parms_create_str(kvpairs);

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_ROUTING, value, sizeof(value));
    if (ret >= 0) {
        val = atoi(value);
        pthread_mutex_lock(&adev->lock);
        pthread_mutex_lock(&out->lock);
        if (val != 0) {
            if ((adev->out_device & AUDIO_DEVICE_OUT_ALL) != val)
                do_out_standby(out);

            /* set the active output device */
            adev->out_device = val;
            select_output_device(adev);
        }
        pthread_mutex_unlock(&out->lock);
        pthread_mutex_unlock(&adev->lock);
    }

    return 0;
}

static char* out_get_parameters(const struct audio_stream *stream, const char *keys)
{
    return strdup("");
}

static uint32_t out_get_latency(const struct audio_stream_out *stream)
{
    const struct j6_stream_out *out = (const struct j6_stream_out *)(stream);
    uint32_t frames = PLAYBACK_BUFFER_SIZE;
    uint32_t latency = (frames * 1000) / PLAYBACK_SAMPLE_RATE;

    ALOGVV("out_get_latency() stream=%p latency=%u msecs", out, latency);

    return latency;
}

static int out_set_volume(struct audio_stream_out *stream, float left,
                          float right)
{
    return -ENOSYS;
}

static ssize_t out_write(struct audio_stream_out *stream, const void* buffer,
                         size_t bytes)
{
    struct j6_stream_out *out = (struct j6_stream_out *)(stream);
    struct j6_audio_device *adev = out->dev;
    const size_t frame_size = audio_stream_frame_size(&stream->common);
    const size_t frames = bytes / frame_size;
    uint32_t rate = out->config.rate;
    uint32_t write_usecs = frames * 1000000 / rate;

    ALOGVV("out_write() stream=%p buffer=%p size=%u/%u time=%u usecs",
           out, buffer, frames, rate, write_usecs);

    pthread_mutex_lock(&adev->lock);
    pthread_mutex_lock(&out->lock);

    if (out->standby) {
        select_output_device(adev);

        ALOGI("out_write() open card %u port %u", adev->card, adev->out_port);
        out->pcm = pcm_open(adev->card, adev->out_port, PCM_OUT | PCM_MMAP, &out->config);
        if (!pcm_is_ready(out->pcm)) {
            ALOGE("out_write() failed to open pcm out: %s", pcm_get_error(out->pcm));
            pcm_close(out->pcm);
            out->pcm = NULL;
            usleep(write_usecs); /* limits the rate of error messages */
            pthread_mutex_unlock(&out->lock);
            pthread_mutex_unlock(&adev->lock);
            return -ENODEV;
        }

        out->standby = false;
    }

    pthread_mutex_unlock(&adev->lock);

    int ret = pcm_mmap_write(out->pcm, buffer, bytes);
    if (ret) {
        ALOGE("out_write() failed to write audio data %d", ret);
        usleep(write_usecs); /* limits the rate of error messages */
    }

    pthread_mutex_unlock(&out->lock);

    return bytes;
}

static int out_get_render_position(const struct audio_stream_out *stream,
                                   uint32_t *dsp_frames)
{
    return -EINVAL;
}

static int out_add_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int out_remove_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int out_get_next_write_timestamp(const struct audio_stream_out *stream,
                                        int64_t *timestamp)
{
    return -EINVAL;
}

/** audio_stream_in implementation **/
static uint32_t in_get_sample_rate(const struct audio_stream *stream)
{
    const struct j6_stream_in *in = (const struct j6_stream_in *)(stream);

    ALOGVV("in_get_sample_rate() stream=%p rate=%u", stream, in->requested_rate);

    return in->requested_rate;
}

static int in_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    ALOGV("in_set_sample_rate() stream=%p rate=%u", stream, rate);

    return 0;
}

static size_t in_get_buffer_size(const struct audio_stream *stream)
{
    const struct j6_stream_in *in = (const struct j6_stream_in *)(stream);

    size_t bytes = get_input_buffer_size(in->requested_rate,
                                         AUDIO_FORMAT_PCM_16_BIT,
                                         in->requested_channels);

    ALOGVV("in_get_buffer_size() stream=%p bytes=%u", in, bytes);

    return bytes;
}

static audio_channel_mask_t in_get_channels(const struct audio_stream *stream)
{
    const struct j6_stream_in *in = (const struct j6_stream_in *)(stream);
    audio_channel_mask_t channels = audio_channel_out_mask_from_count(in->requested_channels);

    ALOGVV("in_get_channels() stream=%p channels=%u", in, in->requested_channels);

    return channels;
}

static audio_format_t in_get_format(const struct audio_stream *stream)
{
    audio_format_t format = AUDIO_FORMAT_PCM_16_BIT;

    ALOGVV("in_set_format() stream=%p format=0x%08x (%u bits/sample)",
           stream, format, audio_bytes_per_sample(format) << 3);

    return format;
}

static int in_set_format(struct audio_stream *stream, audio_format_t format)
{
    ALOGV("in_set_format() stream=%p format=0x%08x (%u bits/sample)",
          stream, format, audio_bytes_per_sample(format) << 3);

    if (format != AUDIO_FORMAT_PCM_16_BIT) {
        return -ENOSYS;
    } else {
        return 0;
    }
}

/* must be called with locks held */
static void do_in_standby(struct j6_stream_in *in)
{
    struct j6_audio_device *adev = in->dev;

    if (!in->standby) {
        ALOGI("do_in_standby() close card %u port %u", adev->card, adev->out_port);
        pcm_close(in->pcm);
        in->pcm = NULL;
        in->standby = true;
    }
}

static int in_standby(struct audio_stream *stream)
{
    struct j6_stream_in *in = (struct j6_stream_in *)(stream);
    struct j6_audio_device *adev = in->dev;

    ALOGV("in_standby() stream=%p", in);
    pthread_mutex_lock(&adev->lock);
    pthread_mutex_lock(&in->lock);
    do_in_standby(in);
    pthread_mutex_unlock(&in->lock);
    pthread_mutex_unlock(&adev->lock);

    return 0;
}

static int in_dump(const struct audio_stream *stream, int fd)
{
    return 0;
}

static int in_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    struct j6_stream_in *in = (struct j6_stream_in *)(stream);
    struct j6_audio_device *adev = in->dev;
    struct str_parms *parms;
    char value[32];
    int ret;
    uint32_t val = 0;

    ALOGV("in_set_parameters() stream=%p parameter='%s'", stream, kvpairs);

    parms = str_parms_create_str(kvpairs);

    /* Nothing to do for AUDIO_PARAMETER_STREAM_INPUT_SOURCE, so it's ignored */

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_ROUTING, value, sizeof(value));
    if (ret >= 0) {
        val = atoi(value);
        pthread_mutex_lock(&adev->lock);
        pthread_mutex_lock(&in->lock);
        if (val != 0) {
            if ((adev->in_device & AUDIO_DEVICE_IN_ALL) != val)
                do_in_standby(in);

            /* set the active input device */
            adev->in_device = val;
            select_input_device(adev);
        }
        pthread_mutex_unlock(&in->lock);
        pthread_mutex_unlock(&adev->lock);
    }

    return 0;
}

static char * in_get_parameters(const struct audio_stream *stream,
                                const char *keys)
{
    return strdup("");
}

static int in_set_gain(struct audio_stream_in *stream, float gain)
{
    return 0;
}

static int get_next_buffer(struct resampler_buffer_provider *buffer_provider,
                           struct resampler_buffer* buffer)
{
    struct j6_stream_in *in;
    struct buffer_remix *remix;

    if (buffer_provider == NULL || buffer == NULL)
        return -EINVAL;

    in = (struct j6_stream_in *)((char *)buffer_provider -
                                 offsetof(struct j6_stream_in, buf_provider));

    if (in->pcm == NULL) {
        buffer->raw = NULL;
        buffer->frame_count = 0;
        in->read_status = -ENODEV;
        return -ENODEV;
    }

    if (in->frames_in == 0) {
        in->read_status = pcm_read(in->pcm,
                                   (void*)in->buffer,
                                   buffer->frame_count * in->hw_frame_size);
        if (in->read_status != 0) {
            ALOGE("get_next_buffer() pcm_read error %d", in->read_status);
            buffer->raw = NULL;
            buffer->frame_count = 0;
            return in->read_status;
        }
        in->frames_in = buffer->frame_count;

        remix = in->remix;
        if (remix)
            remix->remix_func(remix, in->buffer, in->frames_in);
    }

    buffer->frame_count = (buffer->frame_count > in->frames_in) ?
                                in->frames_in : buffer->frame_count;
    buffer->i16 = in->buffer;

    return in->read_status;
}

static void release_buffer(struct resampler_buffer_provider *buffer_provider,
                                  struct resampler_buffer* buffer)
{
    struct j6_stream_in *in;

    if (buffer_provider == NULL || buffer == NULL)
        return;

    in = (struct j6_stream_in *)((char *)buffer_provider -
                                 offsetof(struct j6_stream_in, buf_provider));

    in->frames_in -= buffer->frame_count;
}

/*
 * read_frames() reads frames from kernel driver, down samples to capture rate
 * if necessary and output the number of frames requested to the buffer specified
 */
static ssize_t read_frames(struct j6_stream_in *in, void *buffer, ssize_t frames)
{
    ssize_t frames_wr = 0;
    size_t frame_size;

    ALOGVV("read_frames() stream=%p frames=%u", in, frames);

    if (in->remix)
        frame_size = audio_stream_frame_size(&in->stream.common);
    else
        frame_size = in->hw_frame_size;

    while (frames_wr < frames) {
        size_t frames_rd = frames - frames_wr;

        in->resampler->resample_from_provider(in->resampler,
                    (int16_t *)((char *)buffer + frames_wr * frame_size),
                    &frames_rd);
        /* in->read_status is updated by getNextBuffer() also called by
         * in->resampler->resample_from_provider() */
        if (in->read_status != 0)
            return in->read_status;

        frames_wr += frames_rd;
    }

    return frames_wr;
}

static ssize_t in_read(struct audio_stream_in *stream, void* buffer,
                       size_t bytes)
{
    struct j6_stream_in *in = (struct j6_stream_in *)(stream);
    struct j6_audio_device *adev = in->dev;
    const size_t frame_size = audio_stream_frame_size(&stream->common);
    const size_t frames = bytes / frame_size;
    uint32_t rate = in_get_sample_rate(&stream->common);
    uint32_t read_usecs = frames * 1000000 / rate;
    int ret;

    ALOGVV("in_read() stream=%p buffer=%p size=%u/%u time=%u usecs",
           stream, buffer, frames, rate, read_usecs);

    pthread_mutex_lock(&adev->lock);
    pthread_mutex_lock(&in->lock);

    if (in->standby) {
        select_input_device(adev);

        ALOGI("in_read() open card %u port %u", adev->card, adev->in_port);
        in->pcm = pcm_open(adev->card, adev->in_port, PCM_IN, &in->config);
        if (!pcm_is_ready(in->pcm)) {
            ALOGE("in_read() failed to open pcm in: %s", pcm_get_error(in->pcm));
            pcm_close(in->pcm);
            in->pcm = NULL;
            usleep(read_usecs); /* limits the rate of error messages */
            pthread_mutex_unlock(&in->lock);
            pthread_mutex_unlock(&adev->lock);
            return -ENODEV;
        }

        /* if no supported sample rate is available, use the resampler */
        if (in->resampler) {
            in->resampler->reset(in->resampler);
            in->frames_in = 0;
        }

        in->standby = false;
    }

    pthread_mutex_unlock(&adev->lock);

    if (in->resampler || in->remix)
        ret = read_frames(in, buffer, frames);
    else
        ret = pcm_read(in->pcm, buffer, bytes);

    if (ret < 0) {
        ALOGE("in_read() failed to read audio data %d", ret);
        usleep(read_usecs); /* limits the rate of error messages */
        memset(buffer, 0, bytes);
    }

    pthread_mutex_unlock(&in->lock);

    return bytes;
}

static uint32_t in_get_input_frames_lost(struct audio_stream_in *stream)
{
    ALOGVV("in_get_input_frames_lost() stream=%p frames=%u", stream, 0);
    return 0;
}

static int in_add_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int in_remove_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int adev_open_output_stream(struct audio_hw_device *dev,
                                   audio_io_handle_t handle,
                                   audio_devices_t devices,
                                   audio_output_flags_t flags,
                                   struct audio_config *config,
                                   struct audio_stream_out **stream_out)
{
    struct j6_audio_device *adev = (struct j6_audio_device *)dev;
    struct j6_stream_out *out;

    out = (struct j6_stream_out *)malloc(sizeof(struct j6_stream_out));
    if (!out)
        return -ENOMEM;

    ALOGV("adev_open_output_stream() stream=%p rate=%u channels=%u "
          "format=0x%08x flags=0x%08x",
          out, config->sample_rate, popcount(config->channel_mask),
          config->format, flags);

    pthread_mutex_init(&out->lock, NULL);

    out->stream.common.get_sample_rate = out_get_sample_rate;
    out->stream.common.set_sample_rate = out_set_sample_rate;
    out->stream.common.get_buffer_size = out_get_buffer_size;
    out->stream.common.get_channels = out_get_channels;
    out->stream.common.get_format = out_get_format;
    out->stream.common.set_format = out_set_format;
    out->stream.common.standby = out_standby;
    out->stream.common.dump = out_dump;
    out->stream.common.set_parameters = out_set_parameters;
    out->stream.common.get_parameters = out_get_parameters;
    out->stream.common.add_audio_effect = out_add_audio_effect;
    out->stream.common.remove_audio_effect = out_remove_audio_effect;
    out->stream.get_latency = out_get_latency;
    out->stream.set_volume = out_set_volume;
    out->stream.write = out_write;
    out->stream.get_render_position = out_get_render_position;
    out->stream.get_next_write_timestamp = out_get_next_write_timestamp;

    out->dev = adev;
    out->standby = true;
    out->config = pcm_config_playback;
    adev->out = out;

    config->format = out_get_format(&out->stream.common);
    config->channel_mask = out_get_channels(&out->stream.common);
    config->sample_rate = out_get_sample_rate(&out->stream.common);

    *stream_out = &out->stream;

    return 0;
}

static void adev_close_output_stream(struct audio_hw_device *dev,
                                     struct audio_stream_out *stream)
{
    struct j6_audio_device *adev = (struct j6_audio_device *)dev;
    struct j6_stream_out *out = (struct j6_stream_out *)(stream);

    ALOGV("adev_close_output_stream() stream=%p", out);

    out_standby(&stream->common);
    out->dev = NULL;
    adev->out = NULL;

    free(stream);
}

static int adev_set_parameters(struct audio_hw_device *dev, const char *kvpairs)
{
    return -ENOSYS;
}

static char * adev_get_parameters(const struct audio_hw_device *dev,
                                  const char *keys)
{
    return strdup("");;
}

static int adev_init_check(const struct audio_hw_device *dev)
{
    return 0;
}

static int adev_set_voice_volume(struct audio_hw_device *dev, float volume)
{
    return -ENOSYS;
}

static int adev_set_master_volume(struct audio_hw_device *dev, float volume)
{
    return -ENOSYS;
}

static int adev_get_master_volume(struct audio_hw_device *dev, float *volume)
{
    return -ENOSYS;
}

static int adev_set_master_mute(struct audio_hw_device *dev, bool muted)
{
    return -ENOSYS;
}

static int adev_get_master_mute(struct audio_hw_device *dev, bool *muted)
{
    return -ENOSYS;
}

static int adev_set_mode(struct audio_hw_device *dev, audio_mode_t mode)
{
    ALOGV("adev_set_mode() mode=0x%08x", mode);

    return 0;
}

static int adev_set_mic_mute(struct audio_hw_device *dev, bool state)
{
    struct j6_audio_device *adev = (struct j6_audio_device *)dev;

    ALOGV("adev_set_mic_mute() state=%s", state ? "mute" : "unmute");

    pthread_mutex_lock(&adev->lock);
    set_route_by_array(adev->mixer, rs_capture, state);
    adev->mic_mute = state;
    pthread_mutex_unlock(&adev->lock);

    return 0;
}

static int adev_get_mic_mute(const struct audio_hw_device *dev, bool *state)
{
    const struct j6_audio_device *adev = (const struct j6_audio_device *)dev;

    *state = adev->mic_mute;
    ALOGV("adev_get_mic_mute() state=%s", *state ? "mute" : "unmute");

    return 0;
}

static size_t adev_get_input_buffer_size(const struct audio_hw_device *dev,
                                         const struct audio_config *config)
{
    size_t bytes = get_input_buffer_size(config->sample_rate,
                                        config->format,
                                        popcount(config->channel_mask));

    ALOGVV("adev_in_get_buffer_size() bytes=%u", bytes);

    return bytes;
}

static int adev_open_input_stream(struct audio_hw_device *dev,
                                  audio_io_handle_t handle,
                                  audio_devices_t devices,
                                  struct audio_config *config,
                                  struct audio_stream_in **stream_in)
{
    struct j6_audio_device *adev = (struct j6_audio_device *)dev;
    struct j6_stream_in *in;
    int ret;

    in = (struct j6_stream_in *)malloc(sizeof(struct j6_stream_in));
    if (!in)
        return -ENOMEM;

    ALOGV("adev_open_input_stream() stream=%p rate=%u channels=%u format=0x%08x",
          in, config->sample_rate, popcount(config->channel_mask), config->format);

    pthread_mutex_init(&in->lock, NULL);

    in->stream.common.get_sample_rate = in_get_sample_rate;
    in->stream.common.set_sample_rate = in_set_sample_rate;
    in->stream.common.get_buffer_size = in_get_buffer_size;
    in->stream.common.get_channels = in_get_channels;
    in->stream.common.get_format = in_get_format;
    in->stream.common.set_format = in_set_format;
    in->stream.common.standby = in_standby;
    in->stream.common.dump = in_dump;
    in->stream.common.set_parameters = in_set_parameters;
    in->stream.common.get_parameters = in_get_parameters;
    in->stream.common.add_audio_effect = in_add_audio_effect;
    in->stream.common.remove_audio_effect = in_remove_audio_effect;
    in->stream.set_gain = in_set_gain;
    in->stream.read = in_read;
    in->stream.get_input_frames_lost = in_get_input_frames_lost;

    in->dev = adev;
    in->standby = true;
    in->config = pcm_config_capture;
    in->requested_rate = config->sample_rate;
    in->requested_channels = popcount(config->channel_mask);
    in->hw_frame_size = in->config.channels * sizeof(int16_t);
    in->remix = NULL;
    adev->in = in;

    /* in-place stereo-to-mono remix since capture stream is stereo */
    if (in->requested_channels == 1) {
        ALOGV("adev_open_input_stream() stereo-to-mono remix needed");

        /*
         * buffer size is already enough to allow stereo-to-mono remix
         * and resample if needed
         */
        in->buffer = malloc(2 * in->config.period_size * in->hw_frame_size);
        if (!in->buffer) {
            ret = -ENOMEM;
            goto err1;
        }

        ret = setup_stereo_to_mono_input_remix(in);
        if (ret) {
            ALOGE("adev_open_input_stream() failed to setup remix %d", ret);
            goto err2;
        }
    }

    if (in->requested_rate != in->config.rate) {
        ALOGV("adev_open_input_stream() resample needed, req=%uHz got=%uHz",
              in->requested_rate, in->config.rate);

        in->buf_provider.get_next_buffer = get_next_buffer;
        in->buf_provider.release_buffer = release_buffer;
        ret = create_resampler(in->config.rate,
                               in->requested_rate,
                               in->requested_channels,
                               RESAMPLER_QUALITY_DEFAULT,
                               &in->buf_provider,
                               &in->resampler);
        if (ret) {
            ALOGE("adev_open_input_stream() failed to create resampler %d", ret);
            goto err3;
        }
    }

    *stream_in = &in->stream;

    return 0;

 err3:
    free(in->remix);
 err2:
    free(in->buffer);
 err1:
    free(in);
    return ret;
}

static void adev_close_input_stream(struct audio_hw_device *dev,
                                   struct audio_stream_in *stream)
{
    struct j6_audio_device *adev = (struct j6_audio_device *)dev;
    struct j6_stream_in *in = (struct j6_stream_in *)(stream);

    ALOGV("adev_close_input_stream() stream=%p", stream);

    in_standby(&stream->common);

    if (in->resampler)
        release_resampler(in->resampler);
    in->resampler = NULL;

    if (in->remix)
        free(in->remix);
    in->remix = NULL;

    in->dev = NULL;
    adev->in = NULL;

    free(in->buffer);
    free(in);
}

static int adev_dump(const audio_hw_device_t *device, int fd)
{
    return 0;
}

/*
 * should not be needed for API version 2.0 but AudioFlinger uses it to find
 * suitable hw device, so we keep it
 */
static uint32_t adev_get_supported_devices(const struct audio_hw_device *dev)
{
    uint32_t devices = SUPPORTED_IN_DEVICES | SUPPORTED_OUT_DEVICES;

    ALOGV("adev_get_supported_devices() devices=0x%08x", devices);

    return devices;
}

static int adev_close(hw_device_t *device)
{
    struct j6_audio_device *adev = (struct j6_audio_device *)device;

    ALOGI("adev_close()");

    mixer_close(adev->mixer);
    adev->mixer = NULL;
    free(device);

    return 0;
}

static int adev_open(const hw_module_t* module, const char* name,
                     hw_device_t** device)
{
    struct j6_audio_device *adev;

    ALOGI("adev_open() %s", name);

    if (strcmp(name, AUDIO_HARDWARE_INTERFACE) != 0)
        return -EINVAL;

    adev = (struct j6_audio_device*)malloc(sizeof(struct j6_audio_device));
    if (!adev)
        return -ENOMEM;

    pthread_mutex_init(&adev->lock, NULL);

    adev->device.common.tag = HARDWARE_DEVICE_TAG;
    adev->device.common.version = AUDIO_DEVICE_API_VERSION_2_0;
    adev->device.common.module = (struct hw_module_t *) module;
    adev->device.common.close = adev_close;

    adev->device.get_supported_devices = adev_get_supported_devices;
    adev->device.init_check = adev_init_check;
    adev->device.set_voice_volume = adev_set_voice_volume;
    adev->device.set_master_volume = adev_set_master_volume;
    adev->device.get_master_volume = adev_get_master_volume;
    adev->device.set_master_mute = adev_set_master_mute;
    adev->device.get_master_mute = adev_get_master_mute;
    adev->device.set_mode = adev_set_mode;
    adev->device.set_mic_mute = adev_set_mic_mute;
    adev->device.get_mic_mute = adev_get_mic_mute;
    adev->device.set_parameters = adev_set_parameters;
    adev->device.get_parameters = adev_get_parameters;
    adev->device.get_input_buffer_size = adev_get_input_buffer_size;
    adev->device.open_output_stream = adev_open_output_stream;
    adev->device.close_output_stream = adev_close_output_stream;
    adev->device.open_input_stream = adev_open_input_stream;
    adev->device.close_input_stream = adev_close_input_stream;
    adev->device.dump = adev_dump;

    adev->in_device = AUDIO_DEVICE_IN_BUILTIN_MIC;
    adev->out_device = AUDIO_DEVICE_OUT_SPEAKER;
    adev->card = find_supported_card();
    adev->in_port = 0;
    adev->out_port = 0;
    adev->mic_mute = false;

    adev->mixer = mixer_open(adev->card);
    if (!adev->mixer) {
        free(adev);
        ALOGE("Unable to open the mixer, aborting.");
        return -EINVAL;
    }

    *device = &adev->device.common;

    set_route_by_array(adev->mixer, rs_defaults, 1);

    return 0;
}

static struct hw_module_methods_t hal_module_methods = {
    .open = adev_open,
};

struct audio_module HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = AUDIO_MODULE_API_VERSION_0_1,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = AUDIO_HARDWARE_MODULE_ID,
        .name = "Jacinto6 Audio HAL",
        .author = "Texas Instruments Inc.",
        .methods = &hal_module_methods,
    },
};
