/**
 ********************************************************************
 * @file    mmap_normal.h
 * @brief   This is the header file for "mmap_normal.cpp", defining the structure and
 * (exported) function prototypes.
 *
 * @copyright (c) 2018 DJI. All rights reserved.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MMAP_NORMAL_H
#define __MMAP_NORMAL_H

/* Includes ------------------------------------------------------------------*/
#include<iostream>
#include<ctime>
#include<time.h>
#include<stdio.h>
#include<sys/mman.h>
#include<fcntl.h>
#include<unistd.h>
#if 0
#include "rapidjson/document.h"

using namespace std;
//using namespace rapidjson;

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
class mmap_normal {
public:
	mmap_normal();
	~mmap_normal();

    void mmap_init();
    std::string mmap_read(const char*key);
    void mmap_write(const char* key, const char* value);

private:
    std::string m_filename;
    char* m_mmap_buf;
    Document m_document;

    std::string key1;
    std::string key2;
    std::string key3;
	
};

#endif
#endif // CAMERA_STREAM_SHOW_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/


