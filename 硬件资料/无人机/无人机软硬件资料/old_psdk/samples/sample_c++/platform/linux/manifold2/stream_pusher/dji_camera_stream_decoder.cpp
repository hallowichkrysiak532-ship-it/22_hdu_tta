/**
 ********************************************************************
 * @file    dji_camera_stream_decoder.cpp
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "dji_camera_stream_decoder.hpp"
#include "unistd.h"
#include "pthread.h"
#include "dji_logger.h"

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
DJICameraStreamDecoder::DJICameraStreamDecoder()
    : initSuccess(false),
      cbThreadIsRunning(false),
      cbThreadStatus(-1),
      cb(nullptr),
      cbUserParam(nullptr),
#ifdef FFMPEG_INSTALLED
      pCodecCtx(nullptr),
      pCodec(nullptr),
      pCodecParserCtx(nullptr),
      pSwsCtx(nullptr),
      pFrameYUV(nullptr),
      pFrameRGB(nullptr),
      rgbBuf(nullptr),
#endif
      bufSize(0)
{
    pthread_mutex_init(&decodemutex, nullptr);
}

DJICameraStreamDecoder::~DJICameraStreamDecoder()
{
    pthread_mutex_destroy(&decodemutex);
    if(cb)
    {
        registerCallback(nullptr, nullptr);
    }

    cleanup();
}

bool DJICameraStreamDecoder::init()
{
    pthread_mutex_lock(&decodemutex);

    if (true == initSuccess) {
        USER_LOG_INFO("Decoder already initialized.\n");
        return true;
    }

#ifdef FFMPEG_INSTALLED
    // avcodec_register_all();
    pCodecCtx = avcodec_alloc_context3(nullptr);
    if (!pCodecCtx) {
        return false;
    }

    pCodecCtx->thread_count = 4;
    pCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!pCodec || avcodec_open2(pCodecCtx, pCodec, nullptr) < 0) {
        return false;
    }

    pCodecParserCtx = av_parser_init(AV_CODEC_ID_H264);
    if (!pCodecParserCtx) {
        return false;
    }

    pFrameYUV = av_frame_alloc();
    if (!pFrameYUV) {
        return false;
    }

    pFrameRGB = av_frame_alloc();
    if (!pFrameRGB) {
        return false;
    }

    pSwsCtx = nullptr;

    pCodecCtx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
#endif
    initSuccess = true;

    //init_rtsp_pusher();

    m_rtsp_pusher = new TTARtspServer(8554);
    //m_rtsp_puser->setExtraData();
    m_rtsp_pusher->start();

    pthread_mutex_unlock(&decodemutex);

    return true;
}

void DJICameraStreamDecoder::cleanup()
{
    pthread_mutex_lock(&decodemutex);

    initSuccess = false;

#ifdef FFMPEG_INSTALLED
    if (nullptr != pSwsCtx) {
        sws_freeContext(pSwsCtx);
        pSwsCtx = nullptr;
    }

    if (nullptr != pFrameYUV) {
        av_free(pFrameYUV);
        pFrameYUV = nullptr;
    }

    if (nullptr != pCodecParserCtx) {
        av_parser_close(pCodecParserCtx);
        pCodecParserCtx = nullptr;
    }

    if (nullptr != pCodec) {
        avcodec_close(pCodecCtx);
        pCodec = nullptr;
    }

    if (nullptr != pCodecCtx) {
        av_free(pCodecCtx);
        pCodecCtx = nullptr;
    }

    if (nullptr != rgbBuf) {
        av_free(rgbBuf);
        rgbBuf = nullptr;
    }

    if (nullptr != pFrameRGB) {
        av_free(pFrameRGB);
        pFrameRGB = nullptr;
    }
#endif
    pthread_mutex_unlock(&decodemutex);
}

void *DJICameraStreamDecoder::callbackThreadEntry(void *p)
{
    //DSTATUS_PRIVATE("****** Decoder Callback Thread Start ******\n");
    usleep(50 * 1000);
    static_cast<DJICameraStreamDecoder *>(p)->callbackThreadFunc();
    return nullptr;
}

void DJICameraStreamDecoder::callbackThreadFunc()
{
    while (cbThreadIsRunning) {
        CameraRGBImage copyOfImage;
        if (!decodedImageHandler.getNewImageWithLock(copyOfImage, 1000)) {
            //DDEBUG_PRIVATE("Decoder Callback Thread: Get image time out\n");
            continue;
        }

        if (cb) {
            (*cb)(copyOfImage, cbUserParam);
        }
    }
}

void DJICameraStreamDecoder::decodeBuffer(const uint8_t *buf, int bufLen)
{
    const uint8_t *pData = buf;
    int remainingLen = bufLen;
    int processedLen = 0;
    int ret = 0;

#ifdef FFMPEG_INSTALLED

    m_rtsp_pusher->pushH264((uint8_t*)buf, bufLen);
/*
    AVPacket pkt;
    av_init_packet(&pkt);
    pthread_mutex_lock(&decodemutex);
    while (remainingLen > 0) {
        if (!pCodecParserCtx || !pCodecCtx) {
            //DSTATUS("Invalid decoder ctx.");
            break;
        }
        processedLen = av_parser_parse2(pCodecParserCtx, pCodecCtx,
                                        &pkt.data, &pkt.size,
                                        pData, remainingLen,
                                        AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
        remainingLen -= processedLen;
        pData += processedLen;

        if (pkt.size > 0) {
            int gotPicture = 0;
            avcodec_decode_video2(pCodecCtx, pFrameYUV, &gotPicture, &pkt);

            if (!gotPicture) {
                ////DSTATUS_PRIVATE("Got Frame, but no picture\n");
                continue;
            } else {
                int w = pFrameYUV->width;
                int h = pFrameYUV->height;
                ////DSTATUS_PRIVATE("Got picture! size=%dx%d\n", w, h);

                if (nullptr == pSwsCtx) {
                    pSwsCtx = sws_getContext(w, h, pCodecCtx->pix_fmt,
                                             w, h, AV_PIX_FMT_RGB24,
                                             4, nullptr, nullptr, nullptr);
                }

                if (nullptr == rgbBuf) {
                    bufSize = avpicture_get_size(AV_PIX_FMT_RGB24, w, h);
                    rgbBuf = (uint8_t *) av_malloc(bufSize);
                    avpicture_fill((AVPicture *) pFrameRGB, rgbBuf, AV_PIX_FMT_RGB24, w, h);
                }

                if (nullptr != pSwsCtx && nullptr != rgbBuf) {
                    sws_scale(pSwsCtx,
                              (uint8_t const *const *) pFrameYUV->data, pFrameYUV->linesize, 0, pFrameYUV->height,
                              pFrameRGB->data, pFrameRGB->linesize);

                    pFrameRGB->height = h;
                    pFrameRGB->width = w;

                    decodedImageHandler.writeNewImageWithLock(pFrameRGB->data[0], bufSize, w, h);
                }
            }
        }
    }
    pthread_mutex_unlock(&decodemutex);
    av_free_packet(&pkt);
    */
#endif
}

bool DJICameraStreamDecoder::registerCallback(CameraImageCallback f, void *param)
{
    cb = f;
    cbUserParam = param;

    /* When users register a non-nullptr callback, we will start the callback thread. */
    if (nullptr != cb) {
        if (!cbThreadIsRunning) {
            cbThreadStatus = pthread_create(&callbackThread, nullptr, callbackThreadEntry, this);
            if (0 == cbThreadStatus) {
                //DSTATUS_PRIVATE("User callback thread created successfully!\n");
                cbThreadIsRunning = true;
                return true;
            } else {
                //DERROR_PRIVATE("User called thread creation failed!\n");
                cbThreadIsRunning = false;
                return false;
            }
        } else {
            //DERROR_PRIVATE("Callback thread already running!\n");
            return true;
        }
    } else {
        if (cbThreadStatus == 0) {
            cbThreadIsRunning = false;
            pthread_join(callbackThread, nullptr);
            cbThreadStatus = -1;
        }
        return true;
    }
}

/* Private functions definition-----------------------------------------------*/
#if 0
bool DJICameraStreamDecoder::init_YUV_to_H264(){
    //寻找编码器
    codec_h264 = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec_h264){
        printf("Fail: avcodec_find_encoder\n");
        return false;
    }

    //编码器上下文
    codec_ctx_h264 = avcodec_alloc_context3(codec_h264);
    if (!codec_ctx_h264){
        printf("Fail: avcodec_alloc_context3\n");
        return false;
    }
    codec_ctx_h264->pix_fmt = AV_PIX_FMT_YUV420P;
    codec_ctx_h264->codec_type = AVMEDIA_TYPE_VIDEO;
    codec_ctx_h264->width = 1920;
    codec_ctx_h264->height = 1080;
    codec_ctx_h264->channels = 3;
    codec_ctx_h264->time_base = { 1, 25 };
    codec_ctx_h264->gop_size = 5;   //图像组两个关键帧（I帧）的距离
    codec_ctx_h264->max_b_frames = 0;
    //codec_ctx_h264->qcompress = 0.6;
    //codec_ctx_h264->bit_rate = 90000;
    codec_ctx_h264->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;   //添加PPS、SPS

    av_opt_set(codec_ctx_h264->priv_data, "preset", "ultrafast", 0);    //快速编码，但会损失质量
    //av_opt_set(codec_ctx_h264->priv_data, "tune", "zerolatency", 0);  //适用于快速编码和低延迟流式传输,但是会出现绿屏
    //av_opt_set(codec_ctx_h264->priv_data, "x264opts", "crf=26:vbv-maxrate=728:vbv-bufsize=3640:keyint=25", 0);


    //打开编码器
    if (avcodec_open2(codec_ctx_h264, codec_h264, NULL) < 0){
        printf("Fail: avcodec_open2\n");
        return false;
    }

    //用于接收编码好的H264
    pkt_h264 = av_packet_alloc();

    return true;
}

bool DJICameraStreamDecoder::init_rtsp_pusher(){

    //RTSP
    if (avformat_alloc_output_context2(&fmt_ctx, NULL, "RTSP", "rtsp://127.0.0.1:8554/test") < 0){
        printf("Fail: avformat_alloc_output_context2\n");
        return false;
    }

    //使用tcp协议传输
    av_opt_set(fmt_ctx->priv_data, "rtsp_transport", "tcp", 0);

    //检查所有流是否都有数据，如果没有数据会等待max_interleave_delta微秒
    fmt_ctx->max_interleave_delta = 1000000;

    //输出视频流
    AVStream *video_s = avformat_new_stream(fmt_ctx, codec_h264);
    if (!video_s){
        printf("Fail: avformat_new_stream\n");
        return false;
    }
    video_s->time_base = { 1, 25 };
    //videoindex = video_s->id = fmt_ctx->nb_streams - 1;  //加入到fmt_ctx流
    video_s->id = fmt_ctx->nb_streams - 1;  //加入到fmt_ctx流

    videoindex = video_s->id;

    init_YUV_to_H264();

    //复制AVCodecContext的设置
    if (avcodec_copy_context(video_s->codec, codec_ctx_h264) < 0) {
        printf("Fail: avcodec_copy_context\n");
        return false;
    }
    video_s->codec->codec_tag = 0;
    if (fmt_ctx->oformat->flags & AVFMT_GLOBALHEADER)
        video_s->codec->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    avcodec_parameters_from_context(video_s->codecpar, codec_ctx_h264);

    av_dump_format(fmt_ctx, 0, fmt_ctx->filename, 1);
    if (!(fmt_ctx->oformat->flags & AVFMT_NOFILE)) {    //???
        //打开输出URL（Open output URL）
        if (avio_open(&fmt_ctx->pb, fmt_ctx->filename, AVIO_FLAG_WRITE) < 0) {
            printf("Fail: avio_open('%s')\n", fmt_ctx->filename);
            return false;
        }
    }
    return true;
}

#endif
/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
