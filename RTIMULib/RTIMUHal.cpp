////////////////////////////////////////////////////////////////////////////
////  This file is part of RTIMULib
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

#include "IMUDrivers/RTIMU.h"

#ifdef __linux__
#include <linux/spi/spidev.h>
#elif defined(_WIN32)
#include <Windows.h>
#define _WINDOWS
#endif

#if !defined(WIN32) && !defined(__APPLE__)
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#endif

#if !defined(WIN32)
#include <sys/time.h>
#include <unistd.h>
#endif

#include "third_party/libMPSSE/libMPSSE_spi.h"

RTIMUHal::RTIMUHal()
{
#ifdef _WIN32
    // HACK: Do the one-time init here.
    Init_libMPSSE();
#endif

    m_I2CBus = 255;
    m_SPIBus = 255;
    m_currentSlave = 255;
    m_I2C = -1;
    m_SPI = -1;
    m_SPISpeed = 500000;
}

RTIMUHal::~RTIMUHal()
{
#ifdef _WIN32
    // HACK: Do the one-time cleanup here.
    Cleanup_libMPSSE();
#endif

    HALClose();
}

bool RTIMUHal::HALOpen()
{
#ifdef __linux__
    char buf[32];
#endif  // __linux__

    if (m_busIsI2C) {
        if (m_I2C >= 0) {
            // Already open.
            return true;
        }

        if (m_I2CBus == 255) {
            HAL_ERROR("No I2C bus has been set\n");
            return false;
        }

#ifdef __linux__
        sprintf(buf, "/dev/i2c-%d", m_I2CBus);
        m_I2C = open(buf, O_RDWR);
        if (m_I2C < 0) {
            HAL_ERROR1("Failed to open I2C bus %d\n", m_I2CBus);
            m_I2C = -1;
            return false;
        }
#else   // __linux__
        return false;
#endif  // __linux__
    } else {
        uint32_t SPISpeed = m_SPISpeed;
        unsigned char SPIBits = 8;

        if (m_SPIBus == 255) {
            HAL_ERROR("No SPI bus has been set\n");
            return false;
        }

        if (m_SPIBus >= 8) {
            // Using libMPSSE.
            ChannelConfig channel_config = {0};
            FT_DEVICE_LIST_INFO_NODE dev_list = {0};
            FT_HANDLE handle;
            FT_STATUS status;
            uint32 channel = m_SPIBus - 8;
            uint32 num_channels = 0;

            status = SPI_GetNumChannels(&num_channels);
            if (FT_SUCCESS(status)) {
                if (num_channels == 0) {
                    HAL_ERROR("No FTDI channels available.\n");
                    return false;
                }

                if (channel >= num_channels) {
                    HAL_ERROR("Invalid FTDI channel\n");
                    return false;
                }
            }

            status = SPI_GetChannelInfo(channel, &dev_list);
            if (FT_SUCCESS(status)) {
                HAL_INFO1("Chosen FTDI SPI channel %d\n", channel);
                HAL_INFO1("		Flags=0x%x\n", dev_list.Flags);
                HAL_INFO1("		Type=0x%x\n", dev_list.Type);
                HAL_INFO1("		ID=0x%x\n", dev_list.ID);
                HAL_INFO1("		LocId=0x%x\n", dev_list.LocId);
                HAL_INFO1("		SerialNumber=%s\n", dev_list.SerialNumber);
                HAL_INFO1("		Description=%s\n", dev_list.Description);
                HAL_INFO1("		ftHandle=0x%p\n", dev_list.ftHandle); /*is 0 unless open*/

                if (dev_list.ftHandle != NULL) {
                    m_SPI = (uintptr_t)dev_list.ftHandle;
                    return true;
                }
            }

            status = SPI_OpenChannel(channel, &handle);
            if (!FT_SUCCESS(status)) {
                HAL_ERROR1("Failed to open FTDI channel (%d)\n", status);
                return false;
            }

            // Configure the channel.
            channel_config.ClockRate = SPISpeed;  // Clock rate in HZ
            channel_config.configOptions = SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_ACTIVELOW;
            channel_config.LatencyTimer = 1;
            channel_config.Pin = 0x00000000;
            status = SPI_InitChannel(handle, &channel_config);
            if (!FT_SUCCESS(status)) {
                SPI_CloseChannel(handle);
                HAL_ERROR1("Failed to initialize SPI channel (%d)", status);
                return false;
            }

            // Successfully connected!
            m_SPI = (uintptr_t)handle;
        } else {
#ifdef __linux__
            unsigned char SPIMode = SPI_MODE_0;

            sprintf(buf, "/dev/spidev%d.%d", m_SPIBus, m_SPISelect);
            m_SPI = open(buf, O_RDWR);
            if (m_SPI < 0) {
                HAL_ERROR2("Failed to open SPI bus %d, select %d\n", m_SPIBus, m_SPISelect);
                m_SPI = -1;
                return false;
            }

            if (ioctl(m_SPI, SPI_IOC_WR_MODE, &SPIMode) < 0) {
                HAL_ERROR2("Failed to set WR SPI_MODE0 on bus %d, select %d\n", m_SPIBus, m_SPISelect);
                close(m_SPIBus);
                return false;
            }

            if (ioctl(m_SPI, SPI_IOC_RD_MODE, &SPIMode) < 0) {
                HAL_ERROR2("Failed to set RD SPI_MODE0 on bus %d, select %d\n", m_SPIBus, m_SPISelect);
                close(m_SPIBus);
                return false;
            }

            if (ioctl(m_SPI, SPI_IOC_WR_BITS_PER_WORD, &SPIBits) < 0) {
                HAL_ERROR2("Failed to set WR 8 bit mode on bus %d, select %d\n", m_SPIBus, m_SPISelect);
                close(m_SPIBus);
                return false;
            }

            if (ioctl(m_SPI, SPI_IOC_RD_BITS_PER_WORD, &SPIBits) < 0) {
                HAL_ERROR2("Failed to set RD 8 bit mode on bus %d, select %d\n", m_SPIBus, m_SPISelect);
                close(m_SPIBus);
                return false;
            }

            if (ioctl(m_SPI, SPI_IOC_WR_MAX_SPEED_HZ, &SPISpeed) < 0) {
                HAL_ERROR3("Failed to set WR %dHz on bus %d, select %d\n", SPISpeed, m_SPIBus, m_SPISelect);
                close(m_SPIBus);
                return false;
            }

            if (ioctl(m_SPI, SPI_IOC_RD_MAX_SPEED_HZ, &SPISpeed) < 0) {
                HAL_ERROR3("Failed to set RD %dHz on bus %d, select %d\n", SPISpeed, m_SPIBus, m_SPISelect);
                close(m_SPIBus);
                return false;
            }
#else   // __linux__
            return false;
#endif  // __linux__
        }
    }
    return true;
}

void RTIMUHal::HALClose()
{
    I2CClose();
    SPIClose();
}

void RTIMUHal::I2CClose()
{
    if (m_I2C >= 0) {
#ifdef __linux__
        close(m_I2C);
#endif  // __linux__
        m_I2C = -1;
        m_currentSlave = 255;
    }
}

void RTIMUHal::SPIClose()
{
    if (m_SPI >= 0) {
        if (m_SPIBus >= 8) {
            // FTDI
            SPI_CloseChannel((FT_HANDLE)m_SPI);
        } else {
#ifdef __linux__
            close(m_SPI);
#endif  // __linux__
        }

        m_SPI = -1;
    }
}

bool RTIMUHal::HALWrite(unsigned char slaveAddr, unsigned char regAddr, unsigned char const data, const char *errorMsg)
{
    return HALWrite(slaveAddr, regAddr, 1, &data, errorMsg);
}

bool RTIMUHal::HALWrite(unsigned char slaveAddr, unsigned char regAddr, unsigned char length, unsigned char const *data,
                        const char *errorMsg)
{
    int result;
    char *ifType;

    if (m_busIsI2C) {
        if (!I2CSelectSlave(slaveAddr, errorMsg))
            return false;
        ifType = (char *)"I2C";
    } else {
        ifType = (char *)"SPI";
    }

    if (length == 0) {
        result = ifWrite(&regAddr, 1);

        if (result < 0) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR2("%s write of regAddr failed - %s\n", ifType, errorMsg);
            return false;
        } else if (result != 1) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR2("%s write of regAddr failed (nothing written) - %s\n", ifType, errorMsg);
            return false;
        }
    } else {
        unsigned char txBuff[MAX_WRITE_LEN + 1];

        txBuff[0] = regAddr;
        memcpy(txBuff + 1, data, length);

        result = ifWrite(txBuff, length + 1);

        if (result < 0) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR3("%s data write of %d bytes failed - %s\n", ifType, length, errorMsg);
            return false;
        } else if (result < (int)length) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR4("%s data write of %d bytes failed, only %d written - %s\n", ifType, length, result,
                           errorMsg);
            return false;
        }
    }

    return true;
}

int RTIMUHal::ifWrite(unsigned char *data, unsigned char length)
{
    if (m_busIsI2C) {
#ifdef __linux__
        return write(m_I2C, data, length);
#endif  // __linux__
    } else {
        if (m_SPIBus >= 8) {
            // FTDI write.
            FT_STATUS status;
            uint32 size_transferred = 0;

            status = SPI_Write((FT_HANDLE *)m_SPI, data, length, &size_transferred,
                               SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                                   SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
            if (!FT_SUCCESS(status)) {
                return -1;
            }

            return size_transferred;
        } else {
#ifdef __linux__
            struct spi_ioc_transfer wrIOC;
            memset(&wrIOC, 0, sizeof(wrIOC));
            wrIOC.tx_buf = (unsigned long)data;
            wrIOC.rx_buf = 0;
            wrIOC.len = length;
            return ioctl(m_SPI, SPI_IOC_MESSAGE(1), &wrIOC);
#else   // __linux__
            return -1;
#endif  // __linux__
        }
    }

    return -1;
}

bool RTIMUHal::HALRead(unsigned char slaveAddr, unsigned char regAddr, unsigned char length, unsigned char *data,
                       const char *errorMsg)
{
    if (m_busIsI2C) {
#ifdef __linux__
        int tries, result, total;
        if (!HALWrite(slaveAddr, regAddr, 0, NULL, errorMsg))
            return false;

        total = 0;
        tries = 0;

        while ((total < length) && (tries < 5)) {
            result = read(m_I2C, data + total, length - total);

            if (result < 0) {
                if (strlen(errorMsg) > 0)
                    HAL_ERROR3("I2C read error from %d, %.2X - %s\n", slaveAddr, regAddr, errorMsg);
                return false;
            }

            total += result;

            if (total == length)
                break;

            delayMs(10);
            tries++;
        }

        if (total < length) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR3("I2C read from %d, %.2X failed - %s\n", slaveAddr, regAddr, errorMsg);
            return false;
        }
#endif  // __linux__
    } else {
        if (length > MAX_READ_LEN) {
            HAL_ERROR1("HALRead exceeds maximum read length (%d)", length);
            return false;
        }

        unsigned char rxBuff[MAX_READ_LEN + 1];
        rxBuff[0] = regAddr | 0x80;  // Read operation
        memset(rxBuff + 1, 0, length);

        if (m_SPIBus >= 8) {
            FT_STATUS status;
            uint32 size_transferred = 0;

            // SPI reads are done by transferring a reg address and junk for the expected return length.
            status = SPI_ReadWrite((FT_HANDLE *)m_SPI, rxBuff, rxBuff, length + 1, &size_transferred,
                                   SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                                       SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
            if (!FT_SUCCESS(status)) {
                HAL_ERROR3("FTDI SPI read error (%d, reg %.2X, %s)", status, regAddr, errorMsg);
                return false;
            }
        } else {
#ifdef __linux__
            struct spi_ioc_transfer rdIOC;
            memset(&rdIOC, 0, sizeof(rdIOC));
            rdIOC.tx_buf = (unsigned long)rxBuff;
            rdIOC.rx_buf = (unsigned long)rxBuff;
            rdIOC.len = length + 1;

            if (ioctl(m_SPI, SPI_IOC_MESSAGE(1), &rdIOC) < 0) {
                if (strlen(errorMsg) > 0)
                    HAL_ERROR2("SPI read error from %.2X - %s\n", regAddr, errorMsg);
                return false;
            }
#endif  // __linux__
        }

        memcpy(data, rxBuff + 1, length);
    }

    return true;
}

bool RTIMUHal::HALRead(unsigned char slaveAddr, unsigned char length, unsigned char *data, const char *errorMsg)
{
    if (m_busIsI2C) {
        int tries, result, total;
        if (!I2CSelectSlave(slaveAddr, errorMsg))
            return false;

        total = 0;
        tries = 0;

#ifdef __linux__
        while ((total < length) && (tries < 5)) {
            result = read(m_I2C, data + total, length - total);

            if (result < 0) {
                if (strlen(errorMsg) > 0)
                    HAL_ERROR2("I2C read error from %d - %s\n", slaveAddr, errorMsg);
                return false;
            }

            total += result;

            if (total == length)
                break;

            delayMs(10);
            tries++;
        }

        if (total < length) {
            if (strlen(errorMsg) > 0)
                HAL_ERROR2("I2C read from %d failed - %s\n", slaveAddr, errorMsg);
            return false;
        }
#endif  // __linux__
    } else {
        unsigned char rxBuff[MAX_READ_LEN + 1];

        if (m_SPIBus >= 8) {
            FT_STATUS status;
            uint32 size_transferred = 0;

            status = SPI_Read((FT_HANDLE *)m_SPI, rxBuff, length, &size_transferred,
                              SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
                                  SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
            if (!FT_SUCCESS(status)) {
                HAL_ERROR2("FTDI SPI Read failure %d - %s\n", status, errorMsg);
                return false;
            }
        } else {
#ifdef __linux__
            struct spi_ioc_transfer rdIOC;
            memset(&rdIOC, 0, sizeof(rdIOC));
            rdIOC.tx_buf = 0;
            rdIOC.rx_buf = (unsigned long)rxBuff;
            rdIOC.len = length;

            if (ioctl(m_SPI, SPI_IOC_MESSAGE(1), &rdIOC) < 0) {
                if (strlen(errorMsg) > 0)
                    HAL_ERROR1("SPI read error from - %s\n", errorMsg);
                return false;
            }
            memcpy(data, rxBuff, length);
#endif  // __linux__
        }
    }

    return true;
}

bool RTIMUHal::I2CSelectSlave(unsigned char slaveAddr, const char *errorMsg)
{
    if (m_currentSlave == slaveAddr)
        return true;

    if (!HALOpen()) {
        HAL_ERROR1("Failed to open I2C port - %s\n", errorMsg);
        return false;
    }

#ifdef __linux__
    if (ioctl(m_I2C, I2C_SLAVE, slaveAddr) < 0) {
        HAL_ERROR2("I2C slave select %d failed - %s\n", slaveAddr, errorMsg);
        return false;
    }
#endif  // __linux__

    m_currentSlave = slaveAddr;

    return true;
}

void RTIMUHal::delayMs(int milliSeconds)
{
#ifdef __linux__
    usleep(1000 * milliSeconds);
#elif defined(_WIN32)  // __linux__
    Sleep(milliSeconds);
#endif                 // _WIN32
}
