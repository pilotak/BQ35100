/* mbed Microcontroller Library
 * Copyright (c) 2017 u-blox
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file bq35100.cpp
 * This file defines the API to the TI BQ35100 battery gauge chip.
 */

/** Define these to print debug information. */
//#define DEBUG_BQ35100
//#define DEBUG_BQ35100_BLOCK_DATA

#include <mbed.h>
#include <battery_gauge_bq35100.h>

#ifdef DEBUG_BQ35100
# include <stdio.h>
#endif

// ----------------------------------------------------------------
// COMPILE-TIME MACROS
// ----------------------------------------------------------------

/** How long to wait for a security mode change to succeed. */
#define SET_SECURITY_MODE_RETRY_SECONDS 5

/** How long to wait for accumulated capacity data to be written
 * to data flash when gauging is disabled */
#define GAUGE_COMPLETE_WAIT_MS 10000
 
// ----------------------------------------------------------------
// GENERIC PRIVATE FUNCTIONS
// ----------------------------------------------------------------

// Read two bytes from an address.
// Note: gpI2c should be locked before this is called.
bool BatteryGaugeBq35100::getTwoBytes(uint8_t registerAddress, uint16_t *pBytes)
{
    bool success = false;
    char data[3];

    if (gpI2c != NULL) {
        data[0] = registerAddress;
        data[1] = 0;
        data[2] = 0;

        // Send a command to read from registerAddress
        if ((gpI2c->write(gAddress, &(data[0]), 1, true) == 0) &&
            (gpI2c->read(gAddress, &(data[1]), 2) == 0)) {
            success = true;
            if (pBytes) {
                *pBytes = (((uint16_t) data[2]) << 8) + data[1];
            }
        }
    }

    return success;
}

// Compute the checksum over an address plus the block of data.
uint8_t BatteryGaugeBq35100::computeChecksum(const char * pData, int32_t length)
{
    uint8_t checkSum = 0;
    uint8_t x = 0;

    if (pData != NULL) {
#ifdef DEBUG_BQ35100_BLOCK_DATA
        printf ("BatteryGaugeBq35100 (I2C 0x%02x): computing check sum on data block.\n", gAddress >> 1);
        printf (" 0  1  2  3  4  5  6  7   8  9  A  B  C  D  E  F\n");
#endif
        for (x = 1; x <= length; x++) {
            checkSum += *pData;
            
#ifdef DEBUG_BQ35100_BLOCK_DATA
            if (x % 16 == 8) {
                printf ("%02x  ", *pData);
            } else if (x % 16 == 0) {
                printf ("%02x\n", *pData);
            } else {
                printf ("%02x-", *pData);
            }
#endif
            pData++;
        }

        checkSum = 0xff - checkSum;
    }

#ifdef DEBUG_BQ35100_BLOCK_DATA
    if (x % 16 !=  1) {
        printf("\n");
    }
    
    printf ("BatteryGaugeBq35100 (I2C 0x%02x): check sum is 0x%02x.\n", gAddress >> 1, checkSum);
#endif    
    
    return checkSum;
}

// Read data of a given length from a given address.
// Note: gpI2c should be locked before this is called.
bool BatteryGaugeBq35100::readExtendedData(int32_t address, char * pData, int32_t length)
{
    int32_t lengthRead;
    bool success = false;
    SecurityMode securityMode = getSecurityMode();
    char block[32 + 2 + 2]; // 32 bytes of data, 2 bytes of address,
                            // 1 byte of MACDataSum and 1 byte of MACDataLen
    char data[3];

    // Handle security mode
    if (setSecurityMode(SECURITY_MODE_UNSEALED)) {
        if ((gpI2c != NULL) && (length <= 32) && (address >= 0x4000) && (address < 0x4400) && (pData != NULL)) {
#ifdef DEBUG_BQ35100
            printf("BatteryGaugeBq35100 (I2C 0x%02x): preparing to read %d byte(s) from address 0x%04x.\n", gAddress >> 1, (int) length, (unsigned int) address);
#endif
            // Enable Block Data Control (0x61)
            data[0] = 0x61;
            data[1] = 0;

            if (gpI2c->write(gAddress, &(data[0]), 2) == 0) {
                // Write address to ManufacturerAccessControl (0x3e)
                data[0] = 0x3e;
                data[1] = (char) address;
                data[2] = (char) (address >> 8);

                if (gpI2c->write(gAddress, &(data[0]), 3) == 0) {
                    // Read the address from ManufacturerAccessControl (0x3e then 0x3f),
                    // data from MACData (0x40 to 0x5f), checksum from MACDataSum (0x60) 
                    // and length from MACDataLen (0x61)
                    if ((gpI2c->write(gAddress, &(data[0]), 1, true) == 0) &&
                        (gpI2c->read(gAddress, &(block[0]), sizeof (block)) == 0)) {
                        // Check that the address matches
                        if ((block[0] == (char) address) && (block[1] == (char) (address >> 8))) {
                            // Check that the checksum matches (-2 on MACDataLen as it includes MACDataSum and itself)
                            if (block[34] == computeChecksum (&(block[0]), block[35] - 2)) {
                                // All is good, copy the data to the user
                                lengthRead = block[35] - 4; // -4 rather than -2 to remove the two bytes of address as well
                                 if (lengthRead > length) {
                                    lengthRead = length;
                                }
                                memcpy(pData, &(block[2]), lengthRead);
                                success = true;
#ifdef DEBUG_BQ35100
                                printf("BatteryGaugeBq35100 (I2C 0x%02x): %d byte(s) read successfully.\n", gAddress >> 1, (int) lengthRead);
#endif
                            } else {
#ifdef DEBUG_BQ35100
                                printf("BatteryGaugeBq35100 (I2C 0x%02x): checksum didn't match (0x%02x expected).\n", gAddress >> 1, block[34]);
#endif
                            }
                        } else {
#ifdef DEBUG_BQ35100
                            printf("BatteryGaugeBq35100 (I2C 0x%02x): address didn't match (expected 0x%04x, received 0x%02x%02x).\n", gAddress >> 1, (unsigned int) address, block[1], block[0]);
#endif
                        }
                    } else {
#ifdef DEBUG_BQ35100
                        printf("BatteryGaugeBq35100 (I2C 0x%02x): unable to read %d bytes from ManufacturerAccessControl.\n", gAddress >> 1, sizeof (block));
#endif
                    }
                } else {
#ifdef DEBUG_BQ35100
                    printf("BatteryGaugeBq35100 (I2C 0x%02x): unable to write %d bytes to ManufacturerAccessControl.\r", gAddress >> 1, 3);
#endif
                }
            } else {
#ifdef DEBUG_BQ35100
                printf("BatteryGaugeBq35100 (I2C 0x%02x): unable to set Block Data Control.\n", gAddress >> 1);
#endif
            }
        } else {
#ifdef DEBUG_BQ35100
            printf("BatteryGaugeBq35100 (I2C 0x%02x): unable to set the security mode of the chip.\n", gAddress >> 1);
#endif
        }
    }

    // Put the security mode back to what it was
    if (!setSecurityMode(securityMode)) {
        success = false;
    }

    return success;
}

// Write data of a given length to a given address.
// Note: gpI2c should be locked before this is called.
bool BatteryGaugeBq35100::writeExtendedData(int32_t address, const char * pData, int32_t length)
{
    bool success = false;
    SecurityMode securityMode = getSecurityMode();
    char data[3 + 32];
    uint16_t controlStatus;

    if ((gpI2c != NULL) && (length <= 32) && (address >= 0x4000) && (address < 0x4400) && (pData != NULL)) {
#ifdef DEBUG_BQ35100
        printf("BatteryGaugeBq35100 (I2C 0x%02x): preparing to write %d byte(s) to address 0x%04x.\n", gAddress >> 1, (int) length, (unsigned int) address);
#endif
        // Handle security mode
        if (setSecurityMode(SECURITY_MODE_UNSEALED)) {
            // Enable Block Data Control (0x61)
            data[0] = 0x61;
            data[1] = 0;

            if (gpI2c->write(gAddress, &(data[0]), 2) == 0) {
                // Start write at ManufacturerAccessControl (0x3e)
                data[0] = 0x3e;
                // Next two bytes are the address we will write to
                data[1] = (char) address;
                data[2] = (char) (address >> 8);
                // Remaining bytes are the data bytes we wish to write
                memcpy (&(data[3]), pData, length);

                if (gpI2c->write(gAddress, &(data[0]), 3 + length) == 0) {
                    // Compute the checksum and write it to MACDataSum (0x60)
                    data[1] = computeChecksum (&(data[1]), length + 2);
                    data[0] = 0x60;
                    
                    if (gpI2c->write(gAddress, &(data[0]), 2) == 0) {
                        // Write 4 + length to MACDataLen (0x61)
                        data[1] = length + 4;
                        data[0] = 0x61;

                        if (gpI2c->write(gAddress, &(data[0]), 2) == 0) {
                            // Read the control status register to see if a bad
                            // flash write has been detected (bit 15)
                            data[0] = 0;
                            if ((gpI2c->write(gAddress, &(data[0]), 1) == 0) &&
                                getTwoBytes(0, &controlStatus) &&
                                (((controlStatus >> 15) & 0x01) != 0x01)) {
                                success = true;
                            }
#ifdef DEBUG_BQ35100
                            printf("BatteryGaugeBq35100 (I2C 0x%02x): write successful.\n", gAddress >> 1);
#endif
                        } else {
#ifdef DEBUG_BQ35100
                            printf("BatteryGaugeBq35100 (I2C 0x%02x): unable to read write to MACDataLen.\n", gAddress >> 1);
#endif
                        }
                    } else {
#ifdef DEBUG_BQ35100
                        printf("BatteryGaugeBq35100 (I2C 0x%02x): unable to write to MACDataSum.\n", gAddress >> 1);
#endif
                    }
                } else {
#ifdef DEBUG_BQ35100
                    printf("BatteryGaugeBq35100 (I2C 0x%02x): unable to write %d bytes to ManufacturerAccessControl.\r", gAddress >> 1, (int) length + 2);
#endif
                }
            } else {
#ifdef DEBUG_BQ35100
                printf("BatteryGaugeBq35100 (I2C 0x%02x): unable to set Block Data Control.\n", gAddress >> 1);
#endif
            }
        } else {
#ifdef DEBUG_BQ35100
            printf("BatteryGaugeBq35100 (I2C 0x%02x): unable to set the security mode of the chip.\n", gAddress >> 1);
#endif
        }
    }
    
    // Put the security mode back to what it was
    if (!setSecurityMode(securityMode)) {
        success = false;
    }

    return success;
}

// Get the security mode of the chip.
// Note: gpI2c should be locked before this is called.
BatteryGaugeBq35100::SecurityMode BatteryGaugeBq35100::getSecurityMode(void)
{
    SecurityMode securityMode = SECURITY_MODE_UNKNOWN;
    char data[1];
    uint16_t controlStatus;

    // Read the control status register
    data[0] = 0;
    if ((gpI2c->write(gAddress, &(data[0]), 1) == 0) &&
        getTwoBytes(0, &controlStatus)) {
        // Bits 13 and 14 of the high byte represent the security status,
        // 01 = full access
        // 10 = unsealed access
        // 11 = sealed access
        securityMode = (SecurityMode) ((controlStatus >> 13) & 0x03);
#ifdef DEBUG_BQ35100
        printf("BatteryGaugeBq35100 (I2C 0x%02x): security mode is 0x%02x (control status 0x%04x).\r\n", gAddress >> 1, securityMode, controlStatus);
#endif
    }

    return securityMode;
}

// Set the security mode of the chip.
// Note: gpI2c should be locked before this is called.
bool BatteryGaugeBq35100::setSecurityMode(SecurityMode securityMode)
{
    bool success = false;
    char data[3];
    SecurityMode currentSecurityMode = getSecurityMode();
    
    if (securityMode != SECURITY_MODE_UNKNOWN) {
        if (securityMode != currentSecurityMode) {
            // For reasons that aren't clear, the BQ35100 sometimes refuses
            // to change security mode if a previous security mode change
            // happend only a few seconds ago, hence the retry here
            for (int32_t x = 0; (x < SET_SECURITY_MODE_RETRY_SECONDS) && !success; x++) {
                data[0] = 0x3e;  // Set address to ManufacturerAccessControl
                switch (securityMode) {
                    case SECURITY_MODE_SEALED:
                        // Just seal the chip
                        data[1] = 0x20;  // First byte of SEALED sub-command (0x20)
                        data[2] = 0x00;  // Second byte of SEALED sub-command (0x00) (register address will auto-increment)
                        gpI2c->write(gAddress, &(data[0]), 3);
                    break;
                    case SECURITY_MODE_FULL_ACCESS:
                        // Send the full access code with endianness conversion
                        // in TWO writes
                        data[2] = (char) (gFullAccessCodes >> 24);
                        data[1] = (char) (gFullAccessCodes >> 16);
                        gpI2c->write(gAddress, &(data[0]), 3);
                        data[2] = (char) (gFullAccessCodes >> 8);
                        data[1] = (char) gFullAccessCodes;
                        gpI2c->write(gAddress, &(data[0]), 3);
                    break;
                    case SECURITY_MODE_UNSEALED:
                        data[2] = (char) (gSealCodes >> 24);
                        data[1] = (char) (gSealCodes >> 16);
                        gpI2c->write(gAddress, &(data[0]), 3);
                        data[2] = (char) (gSealCodes >> 8);
                        data[1] = (char) gSealCodes;
                        gpI2c->write(gAddress, &(data[0]), 3);
                    break;
                    case SECURITY_MODE_UNKNOWN:
                    default:
                        MBED_ASSERT(false);
                    break;
                }

                currentSecurityMode = getSecurityMode();
                if (currentSecurityMode == securityMode) {
                    success = true;
#ifdef DEBUG_BQ35100
                    printf("BatteryGaugeBq35100 (I2C 0x%02x): security mode is now 0x%02x.\n", gAddress >> 1, currentSecurityMode);
#endif
                } else {
                    wait_ms(1000);
#ifdef DEBUG_BQ35100
                    printf("BatteryGaugeBq35100 (I2C 0x%02x): security mode set failed (wanted 0x%02x, got 0x%02x), will retry.\n", gAddress >> 1, securityMode, currentSecurityMode);
#endif
                }
            }
        } else {
            success = true;
        }
    }
    
    return success;
}

// Make sure that the device is awake and has taken a reading.
bool BatteryGaugeBq35100::makeAdcReading(void)
{
    bool success = false;
    
    if (isGaugeEnabled()) {
        success = true;
    } else {
        if (enableGauge() && disableGauge()) {
            success = true;
        }
    }
    
    return success;
}

//----------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------

// Constructor.
BatteryGaugeBq35100::BatteryGaugeBq35100(void)
{
    gpI2c = NULL;
    pGaugeEnable = NULL;
    gReady = false;
    gSealCodes = 0;
    gFullAccessCodes = 0;
}

// Destructor.
BatteryGaugeBq35100::~BatteryGaugeBq35100(void)
{
}

// Initialise ourselves.
bool BatteryGaugeBq35100::init(I2C * pI2c, PinName gaugeEnable, uint8_t address, uint32_t sealCodes)
{
    uint16_t answer;
    char data[4];

    gpI2c = pI2c;
    gAddress = address << 1;
    gSealCodes = sealCodes;
    
    if (gaugeEnable != NC) {
        pGaugeEnable = new DigitalOut(gaugeEnable, 1);
        wait_ms(GAUGE_ENABLE_SETTLING_TIME_MS);
    }

    if (gpI2c != NULL) {
        gpI2c->lock();
        gpI2c->frequency(I2C_CLOCK_FREQUENCY);
        
        // Send a control command to read the device type
        data[0] = 0x3e;  // Set address to ManufacturerAccessControl
        data[1] = 0x03;  // First byte of HW_VERSION sub-command (0x03)
        data[2] = 0x00;  // Second byte of HW_VERSION sub-command (0x00) (register address will auto-increment)

        if ((gpI2c->write(gAddress, &(data[0]), 3) == 0) &&
            getTwoBytes(0x40, &answer)) {  // Read from MACData address
            if (answer == 0x00a8) {
                // Read the full access codes, in case we need them
                if (readExtendedData(0x41d0, &(data[0]), sizeof (data))) {
                    // The four bytes are the full access codes
                    gFullAccessCodes = ((uint32_t) data[0] << 24) + ((uint32_t) data[1] << 16) + ((uint32_t) data[2] << 8) + data[3];
#ifdef DEBUG_BQ35100
                    printf("BatteryGaugeBq35100 (I2C 0x%02x): full access code is 0x%08x.\n", gAddress >> 1,
                           (unsigned int) gFullAccessCodes);
#endif
                    gReady = true;
                }
            }

#ifdef DEBUG_BQ35100
            printf("BatteryGaugeBq35100 (I2C 0x%02x): read 0x%04x as HW_VERSION, expected 0x00a8.\n", gAddress >> 1, answer);
#endif
        }

        disableGauge();
        gpI2c->unlock();
    }

#ifdef DEBUG_BQ35100
    if (gReady) {
        printf("BatteryGaugeBq35100 (I2C 0x%02x): handler initialised.\n", gAddress >> 1);
    } else {
        printf("BatteryGaugeBq35100 (I2C 0x%02x): init NOT successful.\n", gAddress >> 1);
    }
#endif

    return gReady;
}

// Switch on the battery capacity monitor.
bool BatteryGaugeBq35100::enableGauge(bool nonVolatile)
{
    bool accumulatedCapacityOn = false;
    bool success = false;
    char data[3];
    char opConfig;
    uint16_t controlStatus;

    if (gReady) {
        if (nonVolatile) {
            // Read the OpConfig register which is at address 0x41b1
            if (readExtendedData(0x41b1, &opConfig, sizeof (opConfig))) {
#ifdef DEBUG_BQ35100
                printf("BatteryGaugeBq35100 (I2C 0x%02x): OpConfig is 0x%02x.\n", gAddress >> 1, opConfig);
#endif                        
                // AccumulatedCapacity is achieved by setting GMSEL 1:0 (in bits 0 and 1) to 00
                if ((opConfig & 0x03) != 0) {
                    opConfig &= ~0x03;
                    // Write the new value back
                    accumulatedCapacityOn = writeExtendedData(0x41b1, &opConfig, sizeof (opConfig));
#ifdef DEBUG_BQ35100
                    if (accumulatedCapacityOn) {
                        printf("BatteryGaugeBq35100 (I2C 0x%02x): AccumulatedCapacity enabled, OpConfig becomes 0x%02x.\r\n", gAddress >> 1, opConfig);
                    }
#endif
                } else {
                    accumulatedCapacityOn = true;
                }
            }
        }

        if (accumulatedCapacityOn || !nonVolatile) {
            if (pGaugeEnable) {
                *pGaugeEnable = 1;
                wait_ms(GAUGE_ENABLE_SETTLING_TIME_MS);
            }
            gpI2c->lock();
            data[0] = 0x3e;  // Set address to ManufacturerAccessControl
            data[1] = 0x11;  // First byte of GAUGE_START sub-command (0x11)
            data[2] = 0x00;  // Second byte of GAUGE_START sub-command (0x00) (register address will auto-increment)
            if (gpI2c->write(gAddress, &(data[0]), 3) == 0) {
                // Wait for GA bit of CONTROL_STATUS (bit 0) to become 1
                data[0] = 0;
                if (gpI2c->write(gAddress, &(data[0]), 1) == 0) {
                    for (int x = 0; (x < GAUGE_COMPLETE_WAIT_MS / 10) && !success; x++) {
                        if (getTwoBytes(0, &controlStatus)) {
                            if ((controlStatus & 0x01) == 0x01) {
                                success = true;
                            } else {
                                wait_ms(10);
                            }
                        } else {
                            wait_ms(10);
                        }
                    }
                }
            }
            gpI2c->unlock();
        }
    }
    
    return success;
}

// Switch off the battery capacity monitor.
bool BatteryGaugeBq35100::disableGauge(void)
{
    bool accumulatedCapacityOn = false;
    bool success = false;
    char data[3];
    char opConfig;
    uint16_t controlStatus;
    
    if (gReady) {
        if (isGaugeEnabled()) {
            // Read the OpConfig register which is at address 0x41b1
            if (readExtendedData(0x41b1, &opConfig, sizeof (opConfig))) {
#ifdef DEBUG_BQ35100
                printf("BatteryGaugeBq35100 (I2C 0x%02x): OpConfig is 0x%02x.\n", gAddress >> 1, opConfig);
#endif                        
                // Check if AccumulatedCapacity is on
                if ((opConfig & 0x03) == 0) {
                    accumulatedCapacityOn = true;
                }
                
                // Send GAUGE_STOP
                gpI2c->lock();
                data[0] = 0x3e;  // Set address to ManufacturerAccessControl
                data[1] = 0x12;  // First byte of GAUGE_STOP sub-command (0x12)
                data[2] = 0x00;  // Second byte of GAUGE_STOP sub-command (0x00) (register address will auto-increment)
                if (gpI2c->write(gAddress, &(data[0]), 3) == 0) {
                    // Wait for GA bit of CONTROL_STATUS (bit 0) to become 0 and,
                    // if AccumulatedCapacity was on, wait for G_DONE
                    // (bit 6 in CONTROL_STATUS) to be set
                    data[0] = 0;
                    if (gpI2c->write(gAddress, &(data[0]), 1) == 0) {
                        for (int x = 0; (x < GAUGE_COMPLETE_WAIT_MS / 10) && !success; x++) {
                            if (getTwoBytes(0, &controlStatus)) {
                                if ((controlStatus & 0x01) == 0) {
                                    if (accumulatedCapacityOn) {
                                        if (((controlStatus >> 6) & 0x01) == 0x01) {
                                            success = true;
#ifdef DEBUG_BQ35100
                                            printf("BatteryGaugeBq35100 (I2C 0x%02x): AccumulatedCapacity data written to non-volatile memory.\r\n", gAddress >> 1);
#endif
                                        } else {
                                            wait_ms(10);
                                        }
                                    } else {
                                        success = true;
                                    }
                                } else {
                                    wait_ms(10);
                                }
                            } else {
                                wait_ms(10);
                            }
                        }
                    }
                }
                gpI2c->unlock();
            }
        } else {
            success = true;
        }

        if (pGaugeEnable) {
            *pGaugeEnable = 0;
        }
    }
    
    return success;
}

// Determine whether battery gauging is enabled.
bool BatteryGaugeBq35100::isGaugeEnabled(void)
{
    bool gaugeEnabled = false;
    char data[1];
    uint16_t controlStatus;

    if (gReady) {
        // Check the GA bit (bit 0) in CONTROL_STATUS
        data[0] = 0;
        if ((gpI2c->write(gAddress, &(data[0]), 1) == 0) &&
            getTwoBytes(0, &controlStatus)) {
#ifdef DEBUG_BQ35100
            printf("BatteryGaugeBq35100 (I2C 0x%02x): read 0x%04x as CONTROL_STATUS.\n", gAddress >> 1, controlStatus);
#endif
            if ((controlStatus & 0x01) == 0x01) {
                gaugeEnabled = true;
            }
        }
    }
    
    return gaugeEnabled;
}

// Set the designed capacity of the cell.
bool BatteryGaugeBq35100::setDesignCapacity(uint32_t capacityMAh)
{
    bool success = false;
    char data[2];

    if (gReady) {
        gpI2c->lock();
        
        data[0] = capacityMAh >> 8;  // Upper byte of design capacity 
        data[1] = capacityMAh;  // Lower byte of design capacity

        // Write to the "Cell Design Capacity mAh" address in data flash
        if (writeExtendedData(0x41fe, &(data[0]), sizeof(data))) {
            success = true;
 
#ifdef DEBUG_BQ35100
            printf("BatteryGaugeBq35100 (I2C 0x%02x): designed cell capacity set to %d mAh.\n", gAddress >> 1,
                   (unsigned int) capacityMAh);
#endif
        }

        gpI2c->unlock();
    }

    return success;
}

// Get the designed capacity of the cell.
bool BatteryGaugeBq35100::getDesignCapacity(uint32_t *pCapacityMAh)
{
    bool success = false;
    uint16_t data;
    
    if (gReady) {
        gpI2c->lock();

        // Read from the DesignCapacity address
        if (getTwoBytes (0x3c, &data)) {
            success = true;

            // The answer is in mAh
            if (pCapacityMAh) {
                *pCapacityMAh = data;
            }

#ifdef DEBUG_BQ35100
            printf("BatteryGaugeBq35100 (I2C 0x%02x): designed cell capacity is %d mAh.\n", gAddress >> 1, data);
#endif
        }

    }
    
    return success;
}

// Get the temperature of the chip.
bool BatteryGaugeBq35100::getTemperature(int32_t *pTemperatureC)
{
    bool success = false;
    int32_t temperatureC = 0;
    uint16_t data;

    if (gReady && makeAdcReading()) {
        gpI2c->lock();
        // Read from the temperature register address
        if (getTwoBytes (0x06, &data)) {
            success = true;

            // The answer is in units of 0.1 K, so convert to C
            temperatureC = ((int32_t) data / 10) - 273;

            if (pTemperatureC) {
                *pTemperatureC = temperatureC;
            }

#ifdef DEBUG_BQ35100
            printf("BatteryGaugeBq35100 (I2C 0x%02x): chip temperature %.1f K, so %d C.\n", gAddress >> 1, ((float) data) / 10, (int) temperatureC);
#endif
        }
        
        gpI2c->unlock();
    }

    return success;
}

// Get the voltage of the battery.
bool BatteryGaugeBq35100::getVoltage(int32_t *pVoltageMV)
{
    bool success = false;
    uint16_t data = 0;

    if (gReady && makeAdcReading()) {
        gpI2c->lock();
        // Read from the voltage register address
        if (getTwoBytes (0x08, &data)) {
            success = true;

            // The answer is in mV
            if (pVoltageMV) {
                *pVoltageMV = (int32_t) data;
            }

#ifdef DEBUG_BQ35100
            printf("BatteryGaugeBq35100 (I2C 0x%02x): battery voltage %.3f V.\n", gAddress >> 1, ((float) data) / 1000);
#endif
        }

        gpI2c->unlock();
    }
    
    return success;
}

// Get the current flowing from the battery.
bool BatteryGaugeBq35100::getCurrent(int32_t *pCurrentMA)
{
    bool success = false;
    int32_t currentMA = 0;
    uint16_t data = 0;

    if (gReady && makeAdcReading()) {
        gpI2c->lock();            
        // Read from the average current register address
        if (getTwoBytes (0x0c, &data)) {
            success = true;

            if (pCurrentMA) {
                *pCurrentMA = currentMA;
            }

#ifdef DEBUG_BQ35100
            printf("BatteryGaugeBq35100 (I2C 0x%02x): current %d mA.\n", gAddress >> 1, (int) currentMA);
#endif
        }

        gpI2c->unlock();
    }
    
    return success;
}

// Get the battery capacity used.
bool BatteryGaugeBq35100::getUsedCapacity(uint32_t *pCapacityUAh)
{
    bool success = false;
    char bytes[5];
    uint32_t data;

    if (gReady && makeAdcReading()) {
        gpI2c->lock();
        // Read four bytes from the AccummulatedCapacity register address
        
        // Send a command to read from registerAddress
        bytes[0] = 0x02;
        bytes[1] = 0;
        bytes[2] = 0;
        bytes[3] = 0;
        bytes[4] = 0;

        if ((gpI2c->write(gAddress, &(bytes[0]), 1) == 0) &&
            (gpI2c->read(gAddress, &(bytes[1]), 4) == 0)) {
            success = true;
            data = (((uint32_t) bytes[4]) << 24) + (((uint32_t) bytes[3]) << 16) + (((uint32_t) bytes[2]) << 8) + bytes[1];
 
            // The answer is in uAh
            if (pCapacityUAh) {
                *pCapacityUAh = data;
            }

#ifdef DEBUG_BQ35100
            printf("BatteryGaugeBq35100 (I2C 0x%02x): battery capacity used %u uAh.\n", gAddress >> 1, (unsigned int) data);
#endif
        }

        gpI2c->unlock();
    }
    
    return success;
}

// Get the battery capacity remaining.
bool BatteryGaugeBq35100::getRemainingCapacity(uint32_t *pCapacityUAh)
{
    bool success = false;
    uint32_t designCapacityUAh;
    uint32_t usedCapacityUAh;

    // First, get the designed capacity
    if (getDesignCapacity(&designCapacityUAh)) {
        designCapacityUAh *= 1000;
        // Then get the used capacity
        if (getUsedCapacity(&usedCapacityUAh)) {
            success = true;
            // Limit the result
            if (usedCapacityUAh > designCapacityUAh) {
                usedCapacityUAh = designCapacityUAh;
            }

            // The answer is in uAh
            if (pCapacityUAh) {
                *pCapacityUAh = designCapacityUAh - usedCapacityUAh;
            }

#ifdef DEBUG_BQ35100
            printf("BatteryGaugeBq35100 (I2C 0x%02x): battery capacity remaining %u uAh (from a designed capacity of %d uAh).\n", gAddress >> 1,
                   (unsigned int) (designCapacityUAh - usedCapacityUAh), (unsigned int) designCapacityUAh);
#endif
        }
    }
    
    return success;
}

// Get the percentage capacity remaining.
bool BatteryGaugeBq35100::getRemainingPercentage(int32_t *pBatteryPercentage)
{
    bool success = false;
    uint32_t designCapacityUAh;
    uint32_t usedCapacityUAh;
    int32_t batteryPercentage;

    // First, get the designed capacity (which is actually in mAh)
    if (getDesignCapacity(&designCapacityUAh)) {
        // Convert to uAh
        designCapacityUAh *= 1000;
        // Then get the used capacity
        if (getUsedCapacity(&usedCapacityUAh)) {
            success = true;
            // Limit the result
            if (usedCapacityUAh > designCapacityUAh) {
                usedCapacityUAh = designCapacityUAh;
            }
            batteryPercentage = (uint64_t) (designCapacityUAh - usedCapacityUAh) * 100 / designCapacityUAh;

            if (pBatteryPercentage) {
                *pBatteryPercentage = batteryPercentage;
            }

#ifdef DEBUG_BQ35100
            printf("BatteryGaugeBq35100 (I2C 0x%02x): battery capacity remaining %d%%.\n", gAddress >> 1,
                   (unsigned int) batteryPercentage);
#endif
        }
    }
    
    return success;
}

// Indicate that a new battery has been inserted.
bool BatteryGaugeBq35100::newBattery(uint32_t capacityMAh)
{
    bool success = false;
    char data[3];
    
    if (gReady) {
        if (setDesignCapacity(capacityMAh)) {
            gpI2c->lock();
            // Send a control command to indicate NEW_BATTERY
            data[0] = 0x3e;  // Set address to ManufacturerAccessControl
            data[1] = 0x13;  // First byte of NEW_BATTERY sub-command (0x13)
            data[2] = 0xA6;  // Second byte of NEW_BATTERY sub-command (0xA6) (register address will auto-increment)
            if (gpI2c->write(gAddress, &(data[0]), 3) == 0) {
                success = true;
#ifdef DEBUG_BQ35100
                printf("BatteryGaugeBq35100 (I2C 0x%02x): new battery set.\n", gAddress >> 1);
#endif
            }

            gpI2c->unlock();
        }
    }

    return success;
}

// Read configuration data.
bool BatteryGaugeBq35100::advancedGetConfig(int32_t address, char * pData, int32_t length)
{
    bool success = false;

    if (gReady) {
        gpI2c->lock();

        success =  readExtendedData(address, pData, length);

        gpI2c->unlock();
    }

    return success;
}

// Write configuration data.
bool BatteryGaugeBq35100::advancedSetConfig(int32_t address, const char * pData, int32_t length)
{
    bool success = false;

    if (gReady) {
        gpI2c->lock();

        success =  writeExtendedData(address, pData, length);

        gpI2c->unlock();
    }

    return success;
}

// Send a control word.
bool BatteryGaugeBq35100::advancedSendControlWord(uint16_t controlWord, uint16_t *pDataReturned)
{
    bool success = false;
    char data[3];

    if (gReady) {
        gpI2c->lock();

        // Send the control command
        data[0] = 0x3e;  // Set address to ManufacturerAccessControl
        data[1] = (char) controlWord;        // First byte of controlWord
        data[2] = (char) (controlWord >> 8); // Second byte of controlWord
        if (gpI2c->write(gAddress, &(data[0]), 3) == 0) {
            // Read the two bytes returned if requested
            if (pDataReturned != NULL) {
                if (getTwoBytes(0x40, pDataReturned)) { // Read from MACData
                    success = true;
#ifdef DEBUG_BQ35100
                    printf("BatteryGaugeBq35100 (I2C 0x%02x): sent control word 0x%04x, read back 0x%04x.\n", gAddress >> 1, controlWord, *pDataReturned);
#endif
                }
            } else {
                success = true;
#ifdef DEBUG_BQ35100
                printf("BatteryGaugeBq35100 (I2C 0x%02x): sent control word 0x%04x.\n", gAddress >> 1, controlWord);
#endif
            }
        }

        gpI2c->unlock();
    }

    return success;
}
    
// Read two bytes starting at a given address on the chip.
bool BatteryGaugeBq35100::advancedGet(uint8_t address, uint16_t *pDataReturned)
{
    bool success = false;
    uint16_t value = 0;

    if (gReady) {
        // Make sure there's a recent reading, as most
        // of these commands involve the chip having done one
        if (makeAdcReading()) {            
            gpI2c->lock();
            // Read the data
            if (getTwoBytes(address, &value)) {
                success = true;
#ifdef DEBUG_BQ35100
                printf("BatteryGaugeBq35100 (I2C 0x%02x): read 0x%04x from addresses 0x%02x and 0x%02x.\n", gAddress >> 1, value, address, address + 1);
#endif
                if (pDataReturned != NULL) {
                    *pDataReturned = value;
                }
            }
            gpI2c->unlock();
        }
    }

    return success;
}

// Get the security mode of the chip.
BatteryGaugeBq35100::SecurityMode BatteryGaugeBq35100::advancedGetSecurityMode(void)
{
    SecurityMode securityMode = SECURITY_MODE_UNKNOWN;

    if (gReady) {
        gpI2c->lock();

        securityMode = getSecurityMode();

        gpI2c->unlock();
    }

    return securityMode;
}

// Set the security mode of the chip.
bool BatteryGaugeBq35100::advancedSetSecurityMode(SecurityMode securityMode)
{
    bool success = false;

    if (gReady) {
        gpI2c->lock();

        success = setSecurityMode(securityMode);

        gpI2c->unlock();
    }

    return success;
}

// Do a hard reset of the chip.
bool BatteryGaugeBq35100::advancedReset(void)
{
    bool success = false;
    SecurityMode securityMode;
    char data[3];

    if (gReady) {
        gpI2c->lock();

        securityMode = getSecurityMode(); // Must be inside lock()
        // Handle unsealing, as this command only works when unsealed
        if (setSecurityMode(SECURITY_MODE_UNSEALED)) {
            // Send a RESET sub-command
            data[0] = 0x3e;  // Set address to ManufacturerAccessControl
            data[1] = 0x41;  // First byte of RESET sub-command (0x41)
            data[2] = 0x00;  // Second byte of RESET sub-command (0x00) (register address will auto-increment)

            if (gpI2c->write(gAddress, &(data[0]), 3) == 0) {
                success = true;
#ifdef DEBUG_BQ35100
                printf("BatteryGaugeBq35100 (I2C 0x%02x): chip hard reset.\n", gAddress >> 1);
#endif
            }
            
            // Set the security mode back to what it was
            if (!setSecurityMode(securityMode)) {
                success = false;
            }
        }

        gpI2c->unlock();
    }

    return success;
}

/* End Of File */
