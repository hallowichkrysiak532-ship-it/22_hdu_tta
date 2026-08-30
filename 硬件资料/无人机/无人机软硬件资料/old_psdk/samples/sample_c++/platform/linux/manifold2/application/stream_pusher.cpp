
#include <utils/util_misc.h>
#include <widget_interaction_test/test_widget_interaction.h>
#include "dji_liveview.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "dji_aircraft_info.h"
#include "time.h"
//#include "gst/gst.h"

// #include "camera_h264_hardware_pusher.hpp"

// #include "ffmpeg_h264_pusher.h"
// #include "dji_camera_stream_decoder.hpp"

#ifdef OPEN_CV_INSTALLED

#include "opencv2/opencv.hpp"
//#include "opencv2/dnn.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "../../../sample_c/module_sample/utils/util_misc.h"

using namespace cv;

const char *classNames[] = {"background", "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
                            "boat", "traffic light",
                            "fire hydrant", "background", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
                            "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "background", "backpack",
                            "umbrella", "background", "background", "handbag", "tie", "suitcase", "frisbee", "skis",
                            "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
                            "surfboard", "tennis racket",
                            "bottle", "background", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
                            "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut",
                            "cake", "chair", "couch", "potted plant", "bed", "background", "dining table", "background",
                            "background", "toilet", "background", "tv", "laptop", "mouse", "remote", "keyboard",
                            "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "background", "book",
                            "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

const size_t inWidth = 320;
const size_t inHeight = 300;
const float WHRatio = inWidth / (float) inHeight;
static int32_t s_demoIndex = -1;
char curFileDirPath[DJI_FILE_PATH_SIZE_MAX];
char tempFileDirPath[DJI_FILE_PATH_SIZE_MAX];
char prototxtFileDirPath[DJI_FILE_PATH_SIZE_MAX];
char weightsFileDirPath[DJI_FILE_PATH_SIZE_MAX];

#endif

#include "test_liveview.hpp"

/* Private constants ---------------------------------------------------------*/
#define TEST_LIVEVIEW_STREAM_FILE_PATH_STR_MAX_SIZE             256
#define TEST_LIVEVIEW_STREAM_STROING_TIME_IN_SECONDS            20

#define TEST_LIVEVIEW_STREAM_REQUEST_I_FRAME_ON                 1
#define TEST_LIVEVIEW_STREAM_REQUEST_I_FRAME_TICK_IN_SECONDS    5

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/
static char s_fpvCameraStreamFilePath[TEST_LIVEVIEW_STREAM_FILE_PATH_STR_MAX_SIZE];
static char s_payloadCameraStreamFilePath[TEST_LIVEVIEW_STREAM_FILE_PATH_STR_MAX_SIZE];

/* Private functions declaration ---------------------------------------------*/
static void DjiTest_FpvCameraStreamCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf,
                                            uint32_t bufLen);
static void DjiTest_PayloadCameraStreamCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf,
                                                uint32_t bufLen);

static void DjiUser_ShowRgbImageCallback(CameraRGBImage img, void *userData);

// ffmpegH264Puser *g_pffmpegH264Puser = NULL;
// DJICameraStreamDecoder  *g_pStreamDecoder = NULL;

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_LiveviewRunSample(E_DjiMountPosition mountPosition)
{
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    time_t currentTime = time(NULL);
    struct tm *localTime = NULL;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;

    USER_LOG_INFO("Liveview sample start");
    //DjiTest_WidgetLogAppend("Liveview sample start");

    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    USER_LOG_INFO("--> Step 1: Init liveview module");
    //DjiTest_WidgetLogAppend("--> Step 1: Init liveview module");
    returnCode = DjiLiveview_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Liveview init failed, error code: 0x%08X", returnCode);
        goto out;
    }

    USER_LOG_INFO("--> Step 2: Start h264 stream of the fpv and selected payload\r\n");
    //DjiTest_WidgetLogAppend("--> Step 2: Start h264 stream of the fpv and selected payload\r\n");

    if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3E) {
        //TODO: how to use on M3E
    } else if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3T) {
        //TODO: how to use on M3T
    } else {
        localTime = localtime(&currentTime);
        sprintf(s_fpvCameraStreamFilePath, "fpv_stream_%04d%02d%02d_%02d-%02d-%02d.h264",
                localTime->tm_year + 1900, localTime->tm_mon + 1, localTime->tm_mday,
                localTime->tm_hour, localTime->tm_min, localTime->tm_sec);

        returnCode = DjiLiveview_StartH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_FPV, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT,
                                                 DjiTest_FpvCameraStreamCallback);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Request h264 of fpv failed, error code: 0x%08X", returnCode);
            goto out;
        }
    }

    localTime = localtime(&currentTime);
    sprintf(s_payloadCameraStreamFilePath, "payload%d_vis_stream_%04d%02d%02d_%02d-%02d-%02d.h264",
            mountPosition, localTime->tm_year + 1900, localTime->tm_mon + 1, localTime->tm_mday,
            localTime->tm_hour, localTime->tm_min, localTime->tm_sec);

    returnCode = DjiLiveview_StartH264Stream((E_DjiLiveViewCameraPosition) mountPosition,
                                             DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT,
                                             DjiTest_PayloadCameraStreamCallback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request h264 of payload %d failed, error code: 0x%08X", mountPosition, returnCode);
    }

    for (int i = 0; i < TEST_LIVEVIEW_STREAM_STROING_TIME_IN_SECONDS; ++i) {
        USER_LOG_INFO("Storing camera h264 stream, second: %d.", i + 1);
#if TEST_LIVEVIEW_STREAM_REQUEST_I_FRAME_ON
        if (i % TEST_LIVEVIEW_STREAM_REQUEST_I_FRAME_TICK_IN_SECONDS == 0) {
            returnCode = DjiLiveview_RequestIntraframeFrameData((E_DjiLiveViewCameraPosition) mountPosition,
                                                                DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Request stream I frame of payload %d failed, error code: 0x%08X", mountPosition,
                               returnCode);
            }
        }
#endif
        osalHandler->TaskSleepMs(1000);
    }

    USER_LOG_INFO("--> Step 3: Stop h264 stream of the fpv and selected payload\r\n");
    //DjiTest_WidgetLogAppend("--> Step 3: Stop h264 stream of the fpv and selected payload");
    if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3E) {

    } else if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3T) {

    } else {
        returnCode = DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_FPV, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Request to stop h264 of fpv failed, error code: 0x%08X", returnCode);
            goto out;
        }
    }

    returnCode = DjiLiveview_StopH264Stream((E_DjiLiveViewCameraPosition) mountPosition,
                                            DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to stop h264 of payload %d failed, error code: 0x%08X", mountPosition, returnCode);
        goto out;
    }

    USER_LOG_INFO("Fpv stream is saved to file: %s", s_fpvCameraStreamFilePath);
    USER_LOG_INFO("Payload%d stream is saved to file: %s\r\n", mountPosition, s_payloadCameraStreamFilePath);

    if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3T) {
        USER_LOG_INFO("--> Start h264 stream of the fpv and selected payload\r\n");

        localTime = localtime(&currentTime);
        sprintf(s_payloadCameraStreamFilePath, "payload%d_ir_stream_%04d%02d%02d_%02d-%02d-%02d.h264",
                mountPosition, localTime->tm_year + 1900, localTime->tm_mon + 1, localTime->tm_mday,
                localTime->tm_hour, localTime->tm_min, localTime->tm_sec);

        returnCode = DjiLiveview_StartH264Stream((E_DjiLiveViewCameraPosition) mountPosition,
                                                 DJI_LIVEVIEW_CAMERA_SOURCE_M3T_IR,
                                                 DjiTest_PayloadCameraStreamCallback);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Request h264 of payload %d failed, error code: 0x%08X", mountPosition, returnCode);
        }

        for (int i = 0; i < TEST_LIVEVIEW_STREAM_STROING_TIME_IN_SECONDS; ++i) {
            USER_LOG_INFO("Storing camera h264 stream, second: %d.", i + 1);
            osalHandler->TaskSleepMs(1000);
        }

        returnCode = DjiLiveview_StopH264Stream((E_DjiLiveViewCameraPosition) mountPosition,
                                                DJI_LIVEVIEW_CAMERA_SOURCE_M3T_IR);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Request to stop h264 of payload %d failed, error code: 0x%08X", mountPosition, returnCode);
            goto out;
        }
    }

    USER_LOG_INFO("Fpv stream is saved to file: %s", s_fpvCameraStreamFilePath);
    USER_LOG_INFO("Payload%d stream is saved to file: %s\r\n", mountPosition, s_payloadCameraStreamFilePath);

    USER_LOG_INFO("--> Step 4: Deinit liveview module");
    //DjiTest_WidgetLogAppend("--> Step 4: Deinit liveview module");
    returnCode = DjiLiveview_Deinit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Liveview deinit failed, error code: 0x%08X", returnCode);
        goto out;
    }

out:
    USER_LOG_INFO("Liveview sample end");

    return returnCode;
}

/* Private functions definition-----------------------------------------------*/
static void DjiTest_FpvCameraStreamCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf,
                                            uint32_t bufLen)
{
    FILE *fp = NULL;
    size_t size;

    USER_LOG_INFO("DjiTest_FpvCameraStreamCallback");

    fp = fopen(s_fpvCameraStreamFilePath, "ab+");
    if (fp == NULL) {
        printf("fopen failed!\n");
        return;
    }

    size = fwrite(buf, 1, bufLen, fp);
    if (size != bufLen) {
        fclose(fp);
        return;
    }

    USER_LOG_INFO("DjiTest_FpvCameraStreamCallback ---------------->data len :%d", bufLen);

    fflush(fp);
    fclose(fp);
}

static void DjiTest_PayloadCameraStreamCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf,
                                                uint32_t bufLen)
{
    FILE *fp = NULL;
    size_t size;

    USER_LOG_INFO("DjiTest_PayloadCameraStreamCallback");

    // fp = fopen(s_payloadCameraStreamFilePath, "ab+");
    // if (fp == NULL) {
    //     printf("fopen failed!\n");
    //     return;
    // }

    // size = fwrite(buf, 1, bufLen, fp);
    // if (size != bufLen) {
    //     fclose(fp);
    //     return;
    // }

    //g_pffmpegH264Puser->decode((uint8_t*)buf, bufLen);


    // g_pStreamDecoder->decodeBuffer((uint8_t*)buf, bufLen);

    USER_LOG_INFO("DjiTest_PayloadCameraStreamCallback ---------------->data len :%d", bufLen);

    // fflush(fp);
    // fclose(fp);
}

// CameraStreamShow *s_cameraStreamShow = nullptr;

// static void cameraRgbImageCallback(const T_Image *image)
// {
// 	//DERROR("cameraRgbImageCallback = %lld", image->imageDataLen);
//     //s_cameraStreamShow->ShowImage(image);
// }

T_DjiReturnCode tta_dji_LiveviewInit(void)
{
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    time_t currentTime = time(NULL);
    struct tm *localTime = NULL;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;

    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    USER_LOG_INFO("--> Step 1: Init liveview module");
    //DjiTest_WidgetLogAppend("--> Step 1: Init liveview module");
    returnCode = DjiLiveview_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Liveview init failed, error code: 0x%08X", returnCode);
    }

    return returnCode;
}

/* Private functions definition-----------------------------------------------*/
static void DjiUser_ShowRgbImageCallback(CameraRGBImage img, void *userData)
{
    string name = string(reinterpret_cast<char *>(userData));
/*
#ifdef OPEN_CV_INSTALLED
    Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width * 3);

    if (s_demoIndex == 0) {
        cvtColor(mat, mat, COLOR_RGB2BGR);
        imshow(name, mat);
    } else if (s_demoIndex == 1) {
        cvtColor(mat, mat, COLOR_RGB2GRAY);
        Mat mask;
        cv::threshold(mat, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        imshow(name, mask);
    } else if (s_demoIndex == 2) {
        cvtColor(mat, mat, COLOR_RGB2BGR);
        snprintf(tempFileDirPath, DJI_FILE_PATH_SIZE_MAX, "%s/data/haarcascade_frontalface_alt.xml", curFileDirPath);
        auto faceDetector = cv::CascadeClassifier(tempFileDirPath);
        std::vector<Rect> faces;
        faceDetector.detectMultiScale(mat, faces, 1.1, 3, 0, Size(50, 50));

        for (int i = 0; i < faces.size(); ++i) {
            cout << "index: " << i;
            cout << "  x: " << faces[i].x;
            cout << "  y: " << faces[i].y << endl;

#ifdef OPEN_CV_VERSION_3
            cv::rectangle(mat, cvPoint(faces[i].x, faces[i].y),
                          cvPoint(faces[i].x + faces[i].width, faces[i].y + faces[i].height),
                          Scalar(0, 0, 255), 2, 1, 0);
#endif

#ifdef OPEN_CV_VERSION_4
            cv::rectangle(mat, cv::Point(faces[i].x, faces[i].y),
                          cv::Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height),
                          Scalar(0, 0, 255), 2, 1, 0);
#endif
        }
        imshow(name, mat);
    } else if (s_demoIndex == 3) {
        snprintf(prototxtFileDirPath, DJI_FILE_PATH_SIZE_MAX,
                 "%s/data/tensorflow/ssd_inception_v2_coco_2017_11_17.pbtxt",
                 curFileDirPath);
        //Attention: If you want to run the Tensorflow Object detection demo, Please download the tensorflow model.
        //Download Url: http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_2017_11_17.tar.gz
        snprintf(weightsFileDirPath, DJI_FILE_PATH_SIZE_MAX, "%s/data/tensorflow/frozen_inference_graph.pb",
                 curFileDirPath);

        dnn::Net net = cv::dnn::readNetFromTensorflow(weightsFileDirPath, prototxtFileDirPath);
        Size frame_size = mat.size();

        Size cropSize;
        if (frame_size.width / (float) frame_size.height > WHRatio) {
            cropSize = Size(static_cast<int>(frame_size.height * WHRatio),
                            frame_size.height);
        } else {
            cropSize = Size(frame_size.width,
                            static_cast<int>(frame_size.width / WHRatio));
        }

        Rect crop(Point((frame_size.width - cropSize.width) / 2,
                        (frame_size.height - cropSize.height) / 2),
                  cropSize);

        cv::Mat blob = cv::dnn::blobFromImage(mat, 1, Size(300, 300));
        net.setInput(blob);
        Mat output = net.forward();
        Mat detectionMat(output.size[2], output.size[3], CV_32F, output.ptr<float>());

        mat = mat(crop);
        float confidenceThreshold = 0.50;

        for (int i = 0; i < detectionMat.rows; i++) {
            float confidence = detectionMat.at<float>(i, 2);
            if (confidence > confidenceThreshold) {
                auto objectClass = (size_t) (detectionMat.at<float>(i, 1));

                int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * mat.cols);
                int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * mat.rows);
                int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * mat.cols);
                int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * mat.rows);

                ostringstream ss;
                ss << confidence;
                String conf(ss.str());

                Rect object((int) xLeftBottom, (int) yLeftBottom,
                            (int) (xRightTop - xLeftBottom),
                            (int) (yRightTop - yLeftBottom));

                rectangle(mat, object, Scalar(0, 255, 0), 2);
                String label = String(classNames[objectClass]) + ": " + conf;

                int baseLine = 0;
                Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
#ifdef OPEN_CV_VERSION_3
                rectangle(mat, Rect(Point(xLeftBottom, yLeftBottom - labelSize.height),
                                    Size(labelSize.width, labelSize.height + baseLine)), Scalar(0, 255, 0), CV_FILLED);
#endif

#ifdef OPEN_CV_VERSION_4
                rectangle(mat, Rect(Point(xLeftBottom, yLeftBottom - labelSize.height),
                                    Size(labelSize.width, labelSize.height + baseLine)), Scalar(0, 255, 0), cv::FILLED);
#endif
                putText(mat, label, Point(xLeftBottom, yLeftBottom), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));
            }
        }
        imshow(name, mat);
    }

    cv::waitKey(1);
#endif
    */
}

// CameraH264HardwarePusher *s_cameraH264HardwarePusher = NULL;// = new CameraH264HardwarePusher(vel, payloadIndex,cameraPosition, "224.1.1.1",5000);
// CameraH264HardwarePusher *s_cameraH264HardwarePusher_FPV = NULL;

void *stream_pusher(void*)
{
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    E_DjiLiveViewCameraPosition cameraPosition = DJI_LIVEVIEW_CAMERA_POSITION_NO_1;

    auto *liveviewSample = new LiveviewSample();
    char mainName[] = "MAIN_CAM";

    liveviewSample->StartMainCameraStream(DjiUser_ShowRgbImageCallback, &mainName);

    while(1)
    {
        returnCode = DjiLiveview_RequestIntraframeFrameData((E_DjiLiveViewCameraPosition) cameraPosition,
                                                                DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Request stream I frame of payload %d failed, error code: 0x%08X", cameraPosition,
                            returnCode);
        }

        osalHandler->TaskSleepMs(1000);

    }


    // g_pStreamDecoder = new DJICameraStreamDecoder();  
    // g_pStreamDecoder->init();

    // // g_pffmpegH264Puser = new ffmpegH264Puser("ffmpeg");
    // // g_pffmpegH264Puser->init();
    // // g_pffmpegH264Puser->start();

    // DjiTest_LiveviewRunSample(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1);





    // tta_dji_LiveviewInit();

    // /* Initialize GStreamer */
	// gst_init(nullptr, nullptr);

    // //DjiTest_LiveviewRunSample(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1);

    // cameraPosition = DJI_LIVEVIEW_CAMERA_POSITION_NO_1;
    // s_cameraH264HardwarePusher = new CameraH264HardwarePusher(cameraPosition, "192.168.44.255",5000);
    // //s_cameraH264HardwarePusher->RegisterRgbImageCallback(cameraRgbImageCallback);

    // // // cameraPosition = DJI_LIVEVIEW_CAMERA_POSITION_FPV;
    // // // s_cameraH264HardwarePusher_FPV = new CameraH264HardwarePusher(cameraPosition, "192.168.44.255",5001);
    // // // s_cameraH264HardwarePusher_FPV->RegisterRgbImageCallback(cameraRgbImageCallback);

    // osalHandler->TaskSleepMs(200);
    // s_cameraH264HardwarePusher->StartGstPipeline();

    // s_cameraH264HardwarePusher_FPV->StartGstPipeline();


}





