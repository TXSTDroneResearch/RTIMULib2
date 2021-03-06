////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//  The MPU-9250 and SPI driver code is based on code generously supplied by
//  staslock@gmail.com (www.clickdrive.io)

#ifndef _RTIMUHAL_H
#define	_RTIMUHAL_H

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>

#ifndef HAL_QUIET
#define HAL_INFO(...)                                             \
    do {                                                          \
        printf("%s:%d:%s(): ", __FILE__, __LINE__, __FUNCTION__); \
        printf(__VA_ARGS__);                                      \
    } while (0)

#define HAL_INFO1(m, x) { printf(m, x); fflush(stdout); }
#define HAL_INFO2(m, x, y) { printf(m, x, y); fflush(stdout); }
#define HAL_INFO3(m, x, y, z) { printf(m, x, y, z); fflush(stdout); }
#define HAL_INFO4(m, x, y, z, a) { printf(m, x, y, z, a); fflush(stdout); }
#define HAL_INFO5(m, x, y, z, a, b) { printf(m, x, y, z, a, b); fflush(stdout); }

#define HAL_ERROR(...)                                                     \
    do {                                                                   \
        fprintf(stderr, "%s:%d:%s(): ", __FILE__, __LINE__, __FUNCTION__); \
        fprintf(stderr, __VA_ARGS__);                                      \
    } while (0)

#define HAL_ERROR1(m, x)    fprintf(stderr, m, x);
#define HAL_ERROR2(m, x, y)    fprintf(stderr, m, x, y);
#define HAL_ERROR3(m, x, y, z)    fprintf(stderr, m, x, y, z);
#define HAL_ERROR4(m, x, y, z, a)    fprintf(stderr, m, x, y, z, a);

#else

#define HAL_INFO(...)
#define HAL_INFO1(m, x)
#define HAL_INFO2(m, x, y)
#define HAL_INFO3(m, x, y, z)
#define HAL_INFO4(m, x, y, z, a)
#define HAL_INFO5(m, x, y, z, a, b)
#define HAL_ERROR(...)
#define HAL_ERROR1(m, x)
#define HAL_ERROR2(m, x, y)
#define HAL_ERROR3(m, x, y, z)
#define HAL_ERROR4(m, x, y, z, a)

#endif

#define MAX_WRITE_LEN                   512
#define MAX_READ_LEN                    512

enum HALMode {
  MODE_INVALID = 0,
  MODE_I2C,
  MODE_SPI,
  MODE_I2C_FTDI,
  MODE_SPI_FTDI
};

class RTIMUHal
{
public:
    RTIMUHal();
    virtual ~RTIMUHal();

    bool m_busIsI2C;                                        // true if I2C bus in use, false if SPI in use
    unsigned char m_I2CBus;                                 // I2C bus of the imu (eg 1 for Raspberry Pi usually)
                                                            // numbers >= 8 specify an FTDI channel
    unsigned char m_SPIBus;                                 // SPI bus of the imu (eg 0 for Raspberry Pi usually)
                                                            // numbers >= 8 specify an FTDI channel
    unsigned char m_SPISelect;                              // SPI select line - defaults to CE0
    unsigned int m_SPISpeed;                                // speed of interface

    virtual bool HALOpen();
    virtual void HALClose();
    virtual bool HALRead(unsigned char slaveAddr, unsigned char regAddr, uint16_t length, unsigned char *data,
                         const char *errorMsg);  // normal read with register select
    virtual bool HALRead(unsigned char slaveAddr, uint16_t length, unsigned char *data,
                         const char *errorMsg);  // read without register select
    virtual bool HALWrite(unsigned char slaveAddr, unsigned char regAddr, unsigned char length,
                          unsigned char const *data, const char *errorMsg);
    virtual bool HALWrite(unsigned char slaveAddr, unsigned char regAddr, unsigned char const data,
                          const char *errorMsg);

    void delayMs(int milliSeconds);

protected:
    void I2CClose();
    bool I2CSelectSlave(unsigned char slaveAddr, const char *errorMsg);
    void SPIClose();
    int ifWrite(unsigned char *data, unsigned char length);

private:
    uintptr_t m_I2C;
    unsigned char m_currentSlave;

    uintptr_t m_SPI;
};

#endif // _RTIMUHAL_H
