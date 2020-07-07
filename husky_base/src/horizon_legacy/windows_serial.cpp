/**
*      _____
*     /  _  \
*    / _/ \  \
*   / / \_/   \
*  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
*  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
*   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
*    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
*     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
*             ROBOTICS�
*
*  File: windows_serial.cpp
*  Desc: Windows-reimplementation of linux-compatible serial commands 
*        for linking with generic functions defined in serial.h
*  Auth: C. Iverach-Brereton
*
*  Copyright (c) 2020, Clearpath Robotics, Inc.
*  All Rights Reserved
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to skynet@clearpathrobotics.com
*
*/

#if defined(_WIN32)

#include <ros/ros.h>

#include "husky_base/horizon_legacy/serial.h"  /* Std. function protos */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <stdlib.h>  /* Malloc */
#include <assert.h>
#include <windows.h>

//#define TX_DEBUG 1
//#define RX_DEBUG 1

int OpenSerial(void **handle, const char *port_name)
{
  HANDLE fd;
  fd = CreateFile(port_name, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
 
  if (fd == INVALID_HANDLE_VALUE)
  {
    fprintf(stderr, "Unable to open %s\n\r", port_name);
    return -3;
  }

#if 0
  // Verify it is a serial port
  if (!isatty(fd))
  {
    CloseHandle(fd);
    fprintf(stderr, "%s is not a serial port\n", port_name);
    return -3;
  }
#endif

  *handle = (HANDLE*) malloc(sizeof(HANDLE));
  **(HANDLE**) handle = fd;
  return 1; // return something positive so that checks on the return value succeed
}

int SetupSerial(void *handleptr)
{
  HANDLE *handle = (HANDLE*)handleptr;
  DCB serialParams = {0};
  serialParams.DCBlength = sizeof(serialParams);
  serialParams.fBinary = TRUE;

  // configure for 115200 baud 8n1
  if(GetCommState(*handle, &serialParams))
  {
    COMMPROP cm;
    GetCommProperties(*handle, &cm); 

    serialParams.BaudRate = CBR_115200;
    serialParams.ByteSize = 8;
    serialParams.StopBits = ONESTOPBIT;
    serialParams.Parity = 0;
    if(!SetCommState(*handle, &serialParams))
    {
      //ROS_ERROR("Failed to set serial port configuration");
      return -1;
    }
  }
  else
  {
    //ROS_ERROR("Failed to read serial port configuration");
    return -1;
  }
  

  // TODO: necessary? correct? ????
  COMMTIMEOUTS timeout = {0};
  timeout.ReadIntervalTimeout = MAXDWORD;
  timeout.ReadTotalTimeoutConstant = 0;
  timeout.ReadTotalTimeoutMultiplier = 0;
  timeout.WriteTotalTimeoutConstant = 0;
  timeout.WriteTotalTimeoutMultiplier = 0;
  SetCommTimeouts(*handle, &timeout);

  return 0;
}

int WriteData(void *handleptr, const char *buffer, int length)
{
  HANDLE *handle = (HANDLE*)handleptr;

  DWORD nBytesWritten = 0;
  WriteFile(*handle, buffer, length, &nBytesWritten, NULL);

  if (nBytesWritten != length)
  {
    fprintf(stderr, "Error in serial write\r\n");
    return -1;
  }

  // serial port output monitor
#ifdef TX_DEBUG
	printf("TX:");
	int i;
	for (i=0; i<length; ++i) printf(" %x", (unsigned char)(buffer[i]));
	printf("\r\n");
#endif

  return (int)nBytesWritten;
}

int ReadData(void *handleptr, char *buffer, int length)
{
  HANDLE *handle = (HANDLE*)handleptr;

  DWORD nBytesRead = 0;

  BOOL ok = ReadFile(*handle, buffer, length, &nBytesRead, NULL);

  if (nBytesRead <= 0)
  {
#ifdef RX_DEBUG
    printf("RX: nil");
    printf("\r\n");
#endif
    return 0;
  }

  // serial port input monitor
#ifdef RX_DEBUG
	printf("RX:");
	DWORD i;
	for (i=0; i<nBytesRead; ++i) printf(" %x", (unsigned char)buffer[i]);
	printf("\r\n");
#endif

  return (int)nBytesRead;
}

int CloseSerial(void *handle)
{
  if (NULL == handle)
  {
    return 0;
  }
  HANDLE *h = (HANDLE*)handle;

  if(*h == INVALID_HANDLE_VALUE)
  {
    return 0;
  }

  CloseHandle(*h);
  free(handle);
  return 0;
}

#endif // _WIN32