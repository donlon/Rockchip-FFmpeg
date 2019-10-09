/*
 * RockChip MPP Video Decoder
 * Copyright (c) 2017 Lionel CHAZALLON
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <drm_fourcc.h>
#include <pthread.h>
#include <rga/RgaApi.h>
#include <rockchip/mpp_buffer.h>
#include <rockchip/rk_mpi.h>
#include <time.h>
#include <unistd.h>

#include "avcodec.h"
#include "decode.h"
#include "hwaccel.h"
#include "internal.h"
#include "libavutil/buffer.h"
#include "libavutil/common.h"
#include "libavutil/frame.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_drm.h"
#include "libavutil/imgutils.h"
#include "libavutil/log.h"

#define RECEIVE_FRAME_TIMEOUT   100
#define FRAMEGROUP_MAX_FRAMES   16
#define INPUT_MAX_PACKETS       4

static int rga_supported = -1;

typedef struct {
    MppCtx ctx;
    MppApi *mpi;
    MppBufferGroup frame_group;

    char first_packet;
    char eos_reached;

    AVBufferRef *frames_ref;
    AVBufferRef *device_ref;
} RKMPPDecoder;

typedef struct {
    AVClass *av_class;
    AVBufferRef *decoder_ref;
} RKMPPDecodeContext;

typedef struct {
    MppFrame frame;
    AVBufferRef *decoder_ref;
} RKMPPFrameContext;

static int get_rga_format(int av_fmt) {
    switch (av_fmt) {
        case AV_PIX_FMT_NV12:
            return RK_FORMAT_YCrCb_420_SP;
        case AV_PIX_FMT_YUV420P:
            return RK_FORMAT_YCbCr_420_P;
        default:
            return -1;
    }
}

// only pixel conversion, keep width/height
static int rkmpp_write_nv12(MppBuffer mpp_buffer, int mpp_vir_width,
                            int mpp_vir_height, AVFrame* dst_frame) {
    rga_info_t src_info = {0};
    rga_info_t dst_info = {0};
    int width = dst_frame->width - (dst_frame->crop_right + dst_frame->crop_left);
    int height = dst_frame->height - (dst_frame->crop_bottom + dst_frame->crop_top);
    int possible_height;
    int rga_format = get_rga_format(dst_frame->format);

    if (rga_format < 0)
        return AVERROR(EINVAL);

    if (rga_supported <= 0)
        goto bail;

    possible_height =
        (dst_frame->data[1] - dst_frame->data[0]) / dst_frame->linesize[0];

    if (dst_frame->format == AV_PIX_FMT_YUV420P &&
        (dst_frame->linesize[0] != 2 * dst_frame->linesize[1] ||
         dst_frame->linesize[1] != dst_frame->linesize[2] ||
         dst_frame->data[1] - dst_frame->data[0] !=
         4 * (dst_frame->data[2] - dst_frame->data[1]))) {
        av_log(NULL, AV_LOG_DEBUG, "dst frame memory is not continuous for planes, fall down to soft copy\n");
        goto bail; // mostly is not continuous memory
    }

    src_info.fd = mpp_buffer_get_fd(mpp_buffer);
    src_info.mmuFlag = 1;
    // mpp decoder always return nv12(yuv420sp)
    rga_set_rect(&src_info.rect, 0, 0, width, height,
                 mpp_vir_width, mpp_vir_height, RK_FORMAT_YCrCb_420_SP);

    dst_info.fd = -1;
    // dst_frame data[*] must be continuous
    dst_info.virAddr = dst_frame->data[0];
    dst_info.mmuFlag = 1;
    rga_set_rect(&dst_info.rect, dst_frame->crop_left, dst_frame->crop_top,
                 width, height, dst_frame->linesize[0], possible_height,
                 rga_format);
    if (c_RkRgaBlit(&src_info, &dst_info, NULL) < 0) {
        av_log(NULL, AV_LOG_ERROR, "Failed to do rga blit\n");
        goto bail;
    }
    return 0;

bail:
    do {
        int i;
        uint8_t* src_ptr = (uint8_t*) mpp_buffer_get_ptr(mpp_buffer);
        for (i = 0; i < height; i++)
            memcpy(dst_frame->data[0] + dst_frame->linesize[0] * i,
                   src_ptr + mpp_vir_width * i, width);
        src_ptr += mpp_vir_width * mpp_vir_height;
        switch (dst_frame->format) {
        case AV_PIX_FMT_NV12:
            for (i = 0; i < height / 2; i++)
                memcpy(dst_frame->data[1] + dst_frame->linesize[1] * i,
                   src_ptr + mpp_vir_width * i, width);
            return 0;
        case AV_PIX_FMT_YUV420P: {
            int j;
            uint8_t* dst_u = dst_frame->data[1];
            uint8_t* dst_v = dst_frame->data[2];
            for (i = 0; i < height / 2; i++) {
                for (j = 0; j < width; j++) {
                    dst_u[j] = src_ptr[2 * j + 0];
                    dst_v[j] = src_ptr[2 * j + 1];
                }
                dst_u += dst_frame->linesize[1];
                dst_v += dst_frame->linesize[2];
                src_ptr += mpp_vir_width;
            }
            return 0;
        }
        default:
            break;
        }
    } while(0);

    return AVERROR(EINVAL);
}

static MppCodingType rkmpp_get_codingtype(AVCodecContext *avctx)
{
    switch (avctx->codec_id) {
    case AV_CODEC_ID_H264:          return MPP_VIDEO_CodingAVC;
    case AV_CODEC_ID_HEVC:          return MPP_VIDEO_CodingHEVC;
    case AV_CODEC_ID_VP8:           return MPP_VIDEO_CodingVP8;
    case AV_CODEC_ID_VP9:           return MPP_VIDEO_CodingVP9;
    default:                        return MPP_VIDEO_CodingUnused;
    }
}

static uint32_t rkmpp_get_frameformat(MppFrameFormat mppformat)
{
    switch (mppformat) {
    case MPP_FMT_YUV420SP:          return DRM_FORMAT_NV12;
#ifdef DRM_FORMAT_NV12_10
    case MPP_FMT_YUV420SP_10BIT:    return DRM_FORMAT_NV12_10;
#endif
    default:                        return 0;
    }
}

static int rkmpp_write_data(AVCodecContext *avctx, uint8_t *buffer, int size, int64_t pts)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = (RKMPPDecoder *)rk_context->decoder_ref->data;
    int ret;
    MppPacket packet;

    // create the MPP packet
    ret = mpp_packet_init(&packet, buffer, size);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to init MPP packet (code = %d)\n", ret);
        return AVERROR_UNKNOWN;
    }

    if (pts == AV_NOPTS_VALUE || !pts)
        pts = avctx->reordered_opaque;

    mpp_packet_set_pts(packet, pts);

    if (!buffer)
        mpp_packet_set_eos(packet);

    ret = decoder->mpi->decode_put_packet(decoder->ctx, packet);
    if (ret != MPP_OK) {
        if (ret == MPP_ERR_BUFFER_FULL) {
            av_log(avctx, AV_LOG_DEBUG, "Buffer full writing %d bytes to decoder\n", size);
            ret = AVERROR(EAGAIN);
        } else
            ret = AVERROR_UNKNOWN;
    }
    else
        av_log(avctx, AV_LOG_DEBUG, "Wrote %d bytes to decoder\n", size);

    mpp_packet_deinit(&packet);

    return ret;
}

static int rkmpp_close_decoder(AVCodecContext *avctx)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    av_buffer_unref(&rk_context->decoder_ref);
    return 0;
}

static void rkmpp_release_decoder(void *opaque, uint8_t *data)
{
    RKMPPDecoder *decoder = (RKMPPDecoder *)data;

    if (decoder->mpi) {
        decoder->mpi->reset(decoder->ctx);
        mpp_destroy(decoder->ctx);
        decoder->ctx = NULL;
    }

    if (decoder->frame_group) {
        mpp_buffer_group_put(decoder->frame_group);
        decoder->frame_group = NULL;
    }

    av_buffer_unref(&decoder->frames_ref);
    av_buffer_unref(&decoder->device_ref);

    av_free(decoder);
}

static int rkmpp_init_decoder(AVCodecContext *avctx)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = NULL;
    MppCodingType codectype = MPP_VIDEO_CodingUnused;
    int ret;
    RK_S64 paramS64;
    RK_S32 paramS32;

    if (avctx->pix_fmt == AV_PIX_FMT_NONE &&
        avctx->sw_pix_fmt == AV_PIX_FMT_NONE) {
        // chromium only support AV_PIX_FMT_YUV420P
        avctx->pix_fmt = avctx->sw_pix_fmt = AV_PIX_FMT_YUV420P;
    } else {
        if (avctx->pix_fmt == AV_PIX_FMT_NONE)
            avctx->pix_fmt = AV_PIX_FMT_DRM_PRIME;
        avctx->sw_pix_fmt = (avctx->pix_fmt == AV_PIX_FMT_DRM_PRIME) ?
                            AV_PIX_FMT_NV12 : avctx->pix_fmt;
    }

    // create a decoder and a ref to it
    decoder = av_mallocz(sizeof(RKMPPDecoder));
    if (!decoder) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    rk_context->decoder_ref = av_buffer_create((uint8_t *)decoder, sizeof(*decoder), rkmpp_release_decoder,
                                               NULL, AV_BUFFER_FLAG_READONLY);
    if (!rk_context->decoder_ref) {
        av_free(decoder);
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    av_log(avctx, AV_LOG_DEBUG, "Initializing RKMPP decoder.\n");

    codectype = rkmpp_get_codingtype(avctx);
    if (codectype == MPP_VIDEO_CodingUnused) {
        av_log(avctx, AV_LOG_ERROR, "Unknown codec type (%d).\n", avctx->codec_id);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    ret = mpp_check_support_format(MPP_CTX_DEC, codectype);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Codec type (%d) unsupported by MPP\n", avctx->codec_id);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    // Create the MPP context
    ret = mpp_create(&decoder->ctx, &decoder->mpi);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to create MPP context (code = %d).\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    // initialize mpp
    ret = mpp_init(decoder->ctx, MPP_CTX_DEC, codectype);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to initialize MPP context (code = %d).\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    // make decode calls blocking with a timeout
    paramS32 = MPP_POLL_BLOCK;
    ret = decoder->mpi->control(decoder->ctx, MPP_SET_OUTPUT_BLOCK, &paramS32);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to set blocking mode on MPI (code = %d).\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    paramS64 = RECEIVE_FRAME_TIMEOUT;
    ret = decoder->mpi->control(decoder->ctx, MPP_SET_OUTPUT_BLOCK_TIMEOUT, &paramS64);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to set block timeout on MPI (code = %d).\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    ret = mpp_buffer_group_get_internal(&decoder->frame_group, MPP_BUFFER_TYPE_ION);
    if (ret) {
       av_log(avctx, AV_LOG_ERROR, "Failed to retrieve buffer group (code = %d)\n", ret);
       ret = AVERROR_UNKNOWN;
       goto fail;
    }

    ret = decoder->mpi->control(decoder->ctx, MPP_DEC_SET_EXT_BUF_GROUP, decoder->frame_group);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "Failed to assign buffer group (code = %d)\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    ret = mpp_buffer_group_limit_config(decoder->frame_group, 0, FRAMEGROUP_MAX_FRAMES);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "Failed to set buffer group limit (code = %d)\n", ret);
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    decoder->first_packet = 1;

    av_log(avctx, AV_LOG_DEBUG, "RKMPP decoder initialized successfully.\n");

    decoder->device_ref = av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_DRM);
    if (!decoder->device_ref) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    ret = av_hwdevice_ctx_init(decoder->device_ref);
    if (ret < 0)
        goto fail;

    if (rga_supported < 0)
        rga_supported = c_RkRgaInit() ? 0 : 1;
    if (!rga_supported) {
        if (access("/dev/rga", R_OK) != 0)
            av_log(avctx, AV_LOG_ERROR, "Fail to access rga: %s\n",
                   strerror(errno));
        av_log(avctx, AV_LOG_ERROR, "No rga support\n");
    }

    return 0;

fail:
    av_log(avctx, AV_LOG_ERROR, "Failed to initialize RKMPP decoder.\n");
    rkmpp_close_decoder(avctx);
    return ret;
}

static int rkmpp_send_packet(AVCodecContext *avctx, const AVPacket *avpkt)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = (RKMPPDecoder *)rk_context->decoder_ref->data;
    int ret;

    // handle EOF
    if (!avpkt->size) {
        av_log(avctx, AV_LOG_DEBUG, "End of stream.\n");
        decoder->eos_reached = 1;
        ret = rkmpp_write_data(avctx, NULL, 0, 0);
        if (ret)
            av_log(avctx, AV_LOG_ERROR, "Failed to send EOS to decoder (code = %d)\n", ret);
        return ret;
    }

    // on first packet, send extradata
    if (decoder->first_packet) {
        if (avctx->extradata_size) {
            ret = rkmpp_write_data(avctx, avctx->extradata,
                                            avctx->extradata_size,
                                            avpkt->pts);
            if (ret) {
                av_log(avctx, AV_LOG_ERROR, "Failed to write extradata to decoder (code = %d)\n", ret);
                return ret;
            }
        }
        decoder->first_packet = 0;
    }

    // now send packet
    ret = rkmpp_write_data(avctx, avpkt->data, avpkt->size, avpkt->pts);
    if (ret && ret!=AVERROR(EAGAIN))
        av_log(avctx, AV_LOG_ERROR, "Failed to write data to decoder (code = %d)\n", ret);

    return ret;
}

static void rkmpp_release_frame(void *opaque, uint8_t *data)
{
    AVDRMFrameDescriptor *desc = (AVDRMFrameDescriptor *)data;
    AVBufferRef *framecontextref = (AVBufferRef *)opaque;
    RKMPPFrameContext *framecontext = (RKMPPFrameContext *)framecontextref->data;

    mpp_frame_deinit(&framecontext->frame);
    av_buffer_unref(&framecontext->decoder_ref);
    av_buffer_unref(&framecontextref);

    av_free(desc);
}

static void rkmpp_setinfo_avframe(AVFrame *frame, MppFrame mppframe) {
    int mode;

    frame->pts              = mpp_frame_get_pts(mppframe);
#if FF_API_PKT_PTS
    FF_DISABLE_DEPRECATION_WARNINGS
    frame->pkt_pts          = frame->pts;
    FF_ENABLE_DEPRECATION_WARNINGS
#endif
    frame->reordered_opaque = frame->pts;
    frame->color_range      = mpp_frame_get_color_range(mppframe);
    frame->color_primaries  = mpp_frame_get_color_primaries(mppframe);
    frame->color_trc        = mpp_frame_get_color_trc(mppframe);
    frame->colorspace       = mpp_frame_get_colorspace(mppframe);

    mode = mpp_frame_get_mode(mppframe);
    frame->interlaced_frame = ((mode & MPP_FRAME_FLAG_FIELD_ORDER_MASK) == MPP_FRAME_FLAG_DEINTERLACED);
    frame->top_field_first  = ((mode & MPP_FRAME_FLAG_FIELD_ORDER_MASK) == MPP_FRAME_FLAG_TOP_FIRST);
}

static int rkmpp_get_continue_video_buffer(AVFrame *frame) {
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(frame->format);
    int ret, i, padded_height;

    if (!desc)
        return AVERROR(EINVAL);

    if ((ret = av_image_check_size(frame->width, frame->height, 0, NULL)) < 0)
        return ret;

    if (!frame->linesize[0]) {
        for(i=1; i<=16; i+=i) {
            ret = av_image_fill_linesizes(frame->linesize, frame->format,
                                          FFALIGN(frame->width, i));
            if (ret < 0)
                return ret;
            // rga need 16 align
            if (!(frame->linesize[0] & (16-1)))
                break;
        }
    }

    padded_height = frame->height;
    if ((ret = av_image_fill_pointers(frame->data, frame->format, padded_height,
                                      NULL, frame->linesize)) < 0)
        return ret;

    frame->buf[0] = av_buffer_alloc(ret + 4 * 16);
    if (!frame->buf[0]) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    if ((ret = av_image_fill_pointers(frame->data, frame->format, padded_height,
                                      frame->buf[0]->data, frame->linesize)) < 0)
        goto fail;

    frame->extended_data = frame->data;

    return 0;
fail:
    av_frame_unref(frame);
    return ret;
}

static int rkmpp_retrieve_frame(AVCodecContext *avctx, AVFrame *frame)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = (RKMPPDecoder *)rk_context->decoder_ref->data;
    RKMPPFrameContext *framecontext = NULL;
    AVBufferRef *framecontextref = NULL;
    int ret;
    MppFrame mppframe = NULL;
    MppBuffer buffer = NULL;
    AVDRMFrameDescriptor *desc = NULL;
    AVDRMLayerDescriptor *layer = NULL;
    MppFrameFormat mppformat;
    uint32_t drmformat;

    ret = decoder->mpi->decode_get_frame(decoder->ctx, &mppframe);
    if (ret != MPP_OK && ret != MPP_ERR_TIMEOUT) {
        av_log(avctx, AV_LOG_ERROR, "Failed to get a frame from MPP (code = %d)\n", ret);
        goto fail;
    }

    if (mppframe) {
        // Check whether we have a special frame or not
        if (mpp_frame_get_info_change(mppframe)) {
            AVHWFramesContext *hwframes;

            av_log(avctx, AV_LOG_INFO, "Decoder noticed an info change (%dx%d), format=%d\n",
                                        (int)mpp_frame_get_width(mppframe), (int)mpp_frame_get_height(mppframe),
                                        (int)mpp_frame_get_fmt(mppframe));

            avctx->width = mpp_frame_get_width(mppframe);
            avctx->height = mpp_frame_get_height(mppframe);
            // chromium will align u/v width height to 32
            avctx->coded_width = FFALIGN(avctx->width, 64);
            avctx->coded_height = FFALIGN(avctx->height, 64);

            decoder->mpi->control(decoder->ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);

            av_buffer_unref(&decoder->frames_ref);

            decoder->frames_ref = av_hwframe_ctx_alloc(decoder->device_ref);
            if (!decoder->frames_ref) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            mppformat = mpp_frame_get_fmt(mppframe);
            drmformat = rkmpp_get_frameformat(mppformat);

            hwframes = (AVHWFramesContext*)decoder->frames_ref->data;
            hwframes->format    = AV_PIX_FMT_DRM_PRIME;
            hwframes->sw_format = drmformat == DRM_FORMAT_NV12 ? AV_PIX_FMT_NV12 : AV_PIX_FMT_NONE;
            hwframes->width     = avctx->width;
            hwframes->height    = avctx->height;
            ret = av_hwframe_ctx_init(decoder->frames_ref);
            if (ret < 0)
                goto fail;

            // here decoder is fully initialized, we need to feed it again with data
            ret = AVERROR(EAGAIN);
            goto fail;
        } else if (mpp_frame_get_eos(mppframe)) {
            av_log(avctx, AV_LOG_DEBUG, "Received a EOS frame.\n");
            decoder->eos_reached = 1;
            ret = AVERROR_EOF;
            goto fail;
        } else if (mpp_frame_get_discard(mppframe)) {
            av_log(avctx, AV_LOG_DEBUG, "Received a discard frame.\n");
            ret = AVERROR(EAGAIN);
            goto fail;
        } else if (mpp_frame_get_errinfo(mppframe)) {
            av_log(avctx, AV_LOG_ERROR, "Received a errinfo frame.\n");
            ret = AVERROR_UNKNOWN;
            goto fail;
        }

        // here we should have a valid frame
        av_log(avctx, AV_LOG_DEBUG, "Received a frame.\n");

        // setup general frame fields
        frame->format           = avctx->pix_fmt;
        frame->width            = avctx->coded_width;
        frame->height           = avctx->coded_height;
        frame->crop_left        = 0;
        frame->crop_right       = avctx->coded_width - mpp_frame_get_width(mppframe);
        frame->crop_top         = 0;
        frame->crop_bottom      = avctx->coded_height - mpp_frame_get_height(mppframe);

        mppformat = mpp_frame_get_fmt(mppframe);
        drmformat = rkmpp_get_frameformat(mppformat);

        // now setup the frame buffer info
        buffer = mpp_frame_get_buffer(mppframe);
        if (buffer) {
            if (avctx->pix_fmt != AV_PIX_FMT_DRM_PRIME) {
                if (avctx->get_buffer2 == avcodec_default_get_buffer2) {
                    // firefox path
                    ret = rkmpp_get_continue_video_buffer(frame);
                } else {
                    ret = ff_get_buffer(avctx, frame, 0);
                }
                if (ret) {
                    av_log(avctx, AV_LOG_ERROR, "Fail to alloc frame.\n");
                    goto fail;
                }
                rkmpp_setinfo_avframe(frame, mppframe);
                // Do pixel conversion, TODO: implement rga AVFilter
                ret = rkmpp_write_nv12(buffer,
                    mpp_frame_get_hor_stride(mppframe),
                    mpp_frame_get_ver_stride(mppframe), frame);
                goto fail; // alway release mpp resource
            }
            rkmpp_setinfo_avframe(frame, mppframe);
            desc = av_mallocz(sizeof(AVDRMFrameDescriptor));
            if (!desc) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            desc->nb_objects = 1;
            desc->objects[0].fd = mpp_buffer_get_fd(buffer);
            desc->objects[0].ptr = mpp_buffer_get_ptr(buffer);
            desc->objects[0].size = mpp_buffer_get_size(buffer);

            desc->nb_layers = 1;
            layer = &desc->layers[0];
            layer->format = drmformat;
            layer->nb_planes = 2;

            layer->planes[0].object_index = 0;
            layer->planes[0].offset = 0;
            layer->planes[0].pitch = mpp_frame_get_hor_stride(mppframe);

            layer->planes[1].object_index = 0;
            layer->planes[1].offset = layer->planes[0].pitch * mpp_frame_get_ver_stride(mppframe);
            layer->planes[1].pitch = layer->planes[0].pitch;

            // we also allocate a struct in buf[0] that will allow to hold additionnal information
            // for releasing properly MPP frames and decoder
            framecontextref = av_buffer_allocz(sizeof(*framecontext));
            if (!framecontextref) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            // MPP decoder needs to be closed only when all frames have been released.
            framecontext = (RKMPPFrameContext *)framecontextref->data;
            framecontext->decoder_ref = av_buffer_ref(rk_context->decoder_ref);
            framecontext->frame = mppframe;

            frame->data[0]  = (uint8_t *)desc;
            frame->buf[0]   = av_buffer_create((uint8_t *)desc, sizeof(*desc), rkmpp_release_frame,
                                               framecontextref, AV_BUFFER_FLAG_READONLY);

            if (!frame->buf[0]) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            frame->hw_frames_ctx = av_buffer_ref(decoder->frames_ref);
            if (!frame->hw_frames_ctx) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            return 0;
        } else {
            av_log(avctx, AV_LOG_ERROR, "Failed to retrieve the frame buffer, frame is dropped (code = %d)\n", ret);
            mpp_frame_deinit(&mppframe);
        }
    } else if (decoder->eos_reached) {
        return AVERROR_EOF;
    } else if (ret == MPP_ERR_TIMEOUT) {
        av_log(avctx, AV_LOG_DEBUG, "Timeout when trying to get a frame from MPP\n");
    } else {
        av_log(avctx, AV_LOG_DEBUG, "MPP decode_get_frame ret = %d\n", ret);
    }

    return AVERROR(EAGAIN);

fail:
    if (mppframe)
        mpp_frame_deinit(&mppframe);

    if (framecontext)
        av_buffer_unref(&framecontext->decoder_ref);

    if (framecontextref)
        av_buffer_unref(&framecontextref);

    if (desc)
        av_free(desc);

    return ret;
}

static int rkmpp_receive_frame(AVCodecContext *avctx, AVFrame *frame)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = (RKMPPDecoder *)rk_context->decoder_ref->data;
    int ret = MPP_NOK;
    AVPacket pkt = {0};
    RK_S32 usedslots, freeslots;

    if (!decoder->eos_reached) {
        // we get the available slots in decoder
        ret = decoder->mpi->control(decoder->ctx, MPP_DEC_GET_STREAM_COUNT, &usedslots);
        if (ret != MPP_OK) {
            av_log(avctx, AV_LOG_ERROR, "Failed to get decoder used slots (code = %d).\n", ret);
            return ret;
        }

        freeslots = INPUT_MAX_PACKETS - usedslots;
        if (freeslots > 0) {
            ret = ff_decode_get_packet(avctx, &pkt);
            if (ret < 0 && ret != AVERROR_EOF) {
                return ret;
            }

            ret = rkmpp_send_packet(avctx, &pkt);
            av_packet_unref(&pkt);

            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "Failed to send packet to decoder (code = %d)\n", ret);
                return ret;
            }
        }
    }

    return rkmpp_retrieve_frame(avctx, frame);
}

static void rkmpp_flush(AVCodecContext *avctx)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = (RKMPPDecoder *)rk_context->decoder_ref->data;
    int ret = MPP_NOK;

    av_log(avctx, AV_LOG_DEBUG, "Flush.\n");

    ret = decoder->mpi->reset(decoder->ctx);
    if (ret == MPP_OK) {
        decoder->first_packet = 1;
        decoder->eos_reached = 0;
    } else
        av_log(avctx, AV_LOG_ERROR, "Failed to reset MPI (code = %d)\n", ret);
}

static const AVCodecHWConfigInternal *rkmpp_hw_configs[] = {
    HW_CONFIG_INTERNAL(DRM_PRIME),
    HW_CONFIG_INTERNAL(NV12),
    HW_CONFIG_INTERNAL(YUV420P),
    NULL
};

#define RKMPP_DEC_CLASS(NAME) \
    static const AVClass rkmpp_##NAME##_dec_class = { \
        .class_name = "rkmpp_" #NAME "_dec", \
        .version    = LIBAVUTIL_VERSION_INT, \
    };

#define RKMPP_DEC(NAME, ID, BSFS) \
    RKMPP_DEC_CLASS(NAME) \
    AVCodec ff_##NAME##_rkmpp_decoder = { \
        .name           = #NAME "_rkmpp", \
        .long_name      = NULL_IF_CONFIG_SMALL(#NAME " (rkmpp)"), \
        .type           = AVMEDIA_TYPE_VIDEO, \
        .id             = ID, \
        .priv_data_size = sizeof(RKMPPDecodeContext), \
        .init           = rkmpp_init_decoder, \
        .close          = rkmpp_close_decoder, \
        .receive_frame  = rkmpp_receive_frame, \
        .flush          = rkmpp_flush, \
        .priv_class     = &rkmpp_##NAME##_dec_class, \
        .capabilities   = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AVOID_PROBING | AV_CODEC_CAP_HARDWARE, \
        .pix_fmts       = (const enum AVPixelFormat[]) { AV_PIX_FMT_DRM_PRIME, \
                                                         AV_PIX_FMT_NV12, \
                                                         AV_PIX_FMT_YUV420P, \
                                                         AV_PIX_FMT_NONE}, \
        .hw_configs     = rkmpp_hw_configs, \
        .bsfs           = BSFS, \
        .wrapper_name   = "rkmpp", \
    };

RKMPP_DEC(h264,  AV_CODEC_ID_H264,          "h264_mp4toannexb")
RKMPP_DEC(hevc,  AV_CODEC_ID_HEVC,          "hevc_mp4toannexb")
RKMPP_DEC(vp8,   AV_CODEC_ID_VP8,           NULL)
RKMPP_DEC(vp9,   AV_CODEC_ID_VP9,           NULL)
