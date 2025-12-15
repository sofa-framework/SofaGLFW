/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <SofaGLFW/utils/VideoEncoderFFMPEG.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

namespace sofaglfw
{

bool VideoEncoderFFMPEG::init(const char* filename, int width, int height, int fps)
{
    // Print only error messages from ffmpeg
    av_log_set_level(AV_LOG_ERROR);
    
    // Ensure dimensions are even (required for YUV420P)
    m_encoderWidth = (width / 2) * 2;
    m_encoderHeight = (height / 2) * 2;
    
    avformat_alloc_output_context2(&m_fmtCtx, nullptr, nullptr, filename);
    if (!m_fmtCtx) return false;
    
    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec) return false;
    
    m_stream = avformat_new_stream(m_fmtCtx, nullptr);
    if (!m_stream) return false;
    
    m_codecCtx = avcodec_alloc_context3(codec);
    m_codecCtx->width = m_encoderWidth;
    m_codecCtx->height = m_encoderHeight;
    m_codecCtx->time_base = {1, fps};
    m_codecCtx->framerate = {fps, 1};
    m_codecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
    m_codecCtx->bit_rate = 2000000;
    m_codecCtx->gop_size = 10;
    m_codecCtx->max_b_frames = 1;
    
    if (m_fmtCtx->oformat->flags & AVFMT_GLOBALHEADER)
        m_codecCtx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    
    AVDictionary* opts = nullptr;
    av_dict_set(&opts, "preset", "medium", 0);
    av_dict_set(&opts, "crf", "23", 0);
    
    if (avcodec_open2(m_codecCtx, codec, &opts) < 0) {
        av_dict_free(&opts);
        return false;
    }
    av_dict_free(&opts);
    
    avcodec_parameters_from_context(m_stream->codecpar, m_codecCtx);
    
    m_frame = av_frame_alloc();
    m_frame->format = m_codecCtx->pix_fmt;
    m_frame->width = m_encoderWidth;
    m_frame->height = m_encoderHeight;
    av_frame_get_buffer(m_frame, 0);
    
    m_pkt = av_packet_alloc();
    
    if (avio_open(&m_fmtCtx->pb, filename, AVIO_FLAG_WRITE) < 0) return false;
    if (avformat_write_header(m_fmtCtx, nullptr) < 0) return false;
    
    m_bIsInitialized = true;
    
    return true;
}

void VideoEncoderFFMPEG::encodeFrame(uint8_t* rgbData, int fbWidth, int fbHeight)
{
    if(!m_bIsInitialized)
    {
        return;
    }
    
    // Recreate sws context if framebuffer size changed
    if (!m_swsCtx || fbWidth != m_encoderWidth || fbHeight != m_encoderHeight) {
        if (m_swsCtx) sws_freeContext(m_swsCtx);
        
        m_swsCtx = sws_getContext(fbWidth, fbHeight, AV_PIX_FMT_RGB24,
                                m_encoderWidth, m_encoderHeight, AV_PIX_FMT_YUV420P,
                                SWS_BILINEAR, nullptr, nullptr, nullptr);
    }
    
    uint8_t* inData[1] = {rgbData};
    int inLinesize[1] = {3 * fbWidth};
    
    sws_scale(m_swsCtx, inData, inLinesize, 0, fbHeight,
              m_frame->data, m_frame->linesize);
    
    m_frame->pts = m_frameCount++;
    
    avcodec_send_frame(m_codecCtx, m_frame);
    while (avcodec_receive_packet(m_codecCtx, m_pkt) == 0) {
        av_packet_rescale_ts(m_pkt, m_codecCtx->time_base, m_stream->time_base);
        m_pkt->stream_index = m_stream->index;
        av_interleaved_write_frame(m_fmtCtx, m_pkt);
        av_packet_unref(m_pkt);
    }
}

void VideoEncoderFFMPEG::finish()
{
    if(!m_bIsInitialized)
    {
        return;
    }
    
    avcodec_send_frame(m_codecCtx, nullptr);
    while (avcodec_receive_packet(m_codecCtx, m_pkt) == 0) {
        av_packet_rescale_ts(m_pkt, m_codecCtx->time_base, m_stream->time_base);
        m_pkt->stream_index = m_stream->index;
        av_interleaved_write_frame(m_fmtCtx, m_pkt);
        av_packet_unref(m_pkt);
    }
    
    av_write_trailer(m_fmtCtx);
    
    if (m_swsCtx) sws_freeContext(m_swsCtx);
    if (m_frame) av_frame_free(&m_frame);
    if (m_pkt) av_packet_free(&m_pkt);
    if (m_codecCtx) avcodec_free_context(&m_codecCtx);
    if (m_fmtCtx) {
        avio_closep(&m_fmtCtx->pb);
        avformat_free_context(m_fmtCtx);
    }
}

} // namespace sofaglfw
