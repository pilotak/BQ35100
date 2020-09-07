/* BQ35100 mbed library
 * Copyright (c) 2020 Pavel Slama
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

#include "BQ35100.h"

BQ35100::BQ35100(uint8_t address):
    _address(address) {
}

BQ35100::BQ35100(PinName sda, PinName scl, uint8_t address, uint32_t frequency):
    _address(address) {
    _i2c = new (_i2c_obj) I2C(sda, scl);
    _i2c->frequency(frequency);
}

BQ35100::~BQ35100(void) {
    if (_i2c == reinterpret_cast<I2C *>(_i2c_obj)) {
        _i2c->~I2C();
    }
}

bool BQ35100::write(const char *data, size_t len, bool stop) {
    tr_debug("Sending data[%u]: %s", len, tr_array(reinterpret_cast<const uint8_t *>(data), len));

    int ack = -1;

    _i2c->lock();
    ack = _i2c->write(_address, data, len, !stop);
    _i2c->unlock();

    if (ack != 0) {
        tr_error("Write failed");
        return false;
    }

    return true;
}

bool BQ35100::read(char *data, size_t len, bool stop) {
    int ack = -1;

    _i2c->lock();
    ack = _i2c->read(_address, data, len, !stop);
    _i2c->unlock();

    if (ack != 0) {
        return false;
    }

    tr_debug("Read data(%u): %s", len, tr_array(reinterpret_cast<uint8_t *>(data), len));

    return true;
}

bool BQ35100::sendData(bq35100_cmd_t cmd, const char *data, size_t len) {
    char *d = new char[len + 1];

    if (!d) {
        tr_error("No memory");
        return false;
    }

    d[0] = (char)cmd;
    memcpy(d + 1, data, len);

    bool success = write(d, len + 1);
    delete[] d;

    return success;
}

bool BQ35100::getData(bq35100_cmd_t cmd, char *data, size_t len) {
    char d[1];
    d[0] = (char)cmd;

    if (!write(d, 1, false)) {
        return false;
    }

    return read(data, len);
}

bool BQ35100::getCntl(bq35100_cntl_t cntl, uint16_t *answer) {
    char data[2];
    data[0] = cntl & UCHAR_MAX;
    data[1] = cntl >> 8;

    if (!sendData(CMD_CONTROL, data, sizeof(data))) {
        return false;
    }

    if (cntl == CNTL_CONTROL_STATUS) {
        getData(CMD_CONTROL, data, sizeof(data));

    } else {
        getData(CMD_MAC_DATA, data, sizeof(data));
    }

    if (answer) {
        *answer = (((uint16_t) data[1]) << 8) + data[0];
        tr_debug("Answer: %04X", *answer);
    }

    return true;
}

bool BQ35100::sendCntl(bq35100_cntl_t cntl) {
    char data[2];
    data[0] = cntl & UCHAR_MAX;
    data[1] = cntl >> 8;

    return sendData(CMD_CONTROL, data, sizeof(data));
}

bool BQ35100::getInternalTemperature(int16_t *temp) {
    char data[2];
    int16_t temp_c = SHRT_MIN;

    if (!getData(CMD_INTERNAL_TEMPERATURE, data, 2)) {
        return false;
    }

    temp_c = ((data[1] << 8) | data[0]) / 10 - 273;

    if (temp) {
        *temp = temp_c;
    }

    tr_debug("Internal temperature: %uK = %i*C", (data[1] << 8) | data[0], temp_c);

    return true;
}

bool BQ35100::init(I2C *i2c_obj, PinName gaugeEnable, uint32_t sealCodes) {
    uint16_t answer;
    _sealCodes = sealCodes;

    if (i2c_obj != nullptr) {
        _i2c = i2c_obj;
    }

    MBED_ASSERT(_i2c);

    if (gaugeEnable != NC) {
        _gaugeEnable = new DigitalOut(gaugeEnable, 1);
        ThisThread::sleep_for(1s);
    }

    // Get device type
    if (!getCntl(CNTL_HW_VERSION, &answer)) {
        tr_error("Couldn't get device type");
        return false;
    }

    if (answer != 0x00A8) {
        tr_error("Different device type");
        return false;
    }

    // Get security mode
    if (!getCntl(CNTL_CONTROL_STATUS, &answer)) {
        tr_error("Couldn't get security mode");
        return false;
    }

    switch (answer >> 13 & 0b011) {
        case SECURITY_MODE_FULL_ACCESS:
            tr_info("Device is in FULL ACCESS mode");
            break;

        case SECURITY_MODE_UNSEALED:
            tr_info("Device is in UNSEALED mode");
            break;

        case SECURITY_MODE_SEALED:
            tr_info("Device is in SEALED mode");
            break;

        default:
            tr_error("Invalid device mode");
            return false;
    }

    _enabled = answer & 0b1;

    return true;
}

bool BQ35100::enableGauge() {
    uint16_t answer;

    if (_enabled) {
        tr_debug("Gauge already enabled");
        return true;
    }

    if (!sendCntl(CNTL_GAUGE_START)) {
        tr_error("Error enabling gauge");
        return false;
    }

    for (auto i = 0; i < 10; i++) {
        if (!getCntl(CNTL_CONTROL_STATUS, &answer)) {
            tr_error("Couldn't get device mode");
            return false;
        }

        if (answer & 0b1) {
            tr_info("Gauge is now enabled");
            break;

        } else {
            tr_debug("Not yet enabled");
            ThisThread::sleep_for(10ms);
        }
    }

    _enabled = answer & 0b1;

    if (!_enabled) {
        tr_error("Gauge not enabled");
    }

    return _enabled;
}

bool BQ35100::disableGauge(void) {
    uint16_t answer;

    if (!_enabled) {
        tr_debug("Gauge already disabled");
        return true;
    }

    if (!sendCntl(CNTL_GAUGE_STOP)) {
        tr_error("Error disabling gauge");
        return false;
    }

    for (auto i = 0; i < 10; i++) {
        if (!getCntl(CNTL_CONTROL_STATUS, &answer)) {
            tr_error("Couldn't get device mode");
            return false;
        }

        if (!(answer & 0b1)) {
            tr_info("Gauge is now disabled");
            break;

        } else {
            tr_debug("Not yet disabled");
            ThisThread::sleep_for(10ms);
        }
    }

    _enabled = answer & 0b1;

    if (_enabled) {
        tr_error("Gauge not disabled");
    }

    return !_enabled;
}

bool BQ35100::isGaugeEnabled(void) {
    return _enabled;
}

bool BQ35100::setDesignCapacity(uint32_t capacityMAh) {
    bool success = false;
    char data[2];

    if (_ready) {
        _i2c->lock();

        data[0] = capacityMAh >> 8;  // Upper byte of design capacity
        data[1] = capacityMAh;  // Lower byte of design capacity

        // Write to the "Cell Design Capacity mAh" address in data flash
        if (writeExtendedData(0x41fe, data, sizeof(data))) {
            success = true;
            tr_debug("designed cell capacity set to %lu mAh.", capacityMAh);
        }

        _i2c->unlock();
    }

    return success;
}

bool BQ35100::getDesignCapacity(uint16_t *capacity) {
    char data[2];

    if (!getData(CMD_DESIGN_CAPACITY, data, 2)) {
        return false;
    }

    if (capacity) {
        *capacity = (data[1] << 8) | data[0];
    }

    tr_info("Designed cell capacity is %u mAh.", (data[1] << 8) | data[0]);

    return true;
}

bool BQ35100::getTemperature(int16_t *temp) {
    char data[2];
    int16_t temp_c = SHRT_MIN;

    if (!getData(CMD_TEMPERATURE, data, 2)) {
        return false;
    }

    temp_c = ((data[1] << 8) | data[0]) / 10 - 273;

    if (temp) {
        *temp = temp_c;
    }

    tr_debug("Internal temperature: %uK = %i*C", (data[1] << 8) | data[0], temp_c);

    return true;
}

bool BQ35100::getVoltage(uint16_t *voltage) {
    char data[2];

    if (!getData(CMD_VOLTAGE, data, 2)) {
        return false;
    }

    if (voltage) {
        *voltage = (data[1] << 8) | data[0];
    }

    tr_debug("Battery voltage: %umV", (data[1] << 8) | data[0]);

    return true;
}

bool BQ35100::getCurrent(uint16_t *current) {
    char data[2];

    if (!getData(CMD_CURRENT, data, 2)) {
        return false;
    }

    if (current) {
        *current = (data[1] << 8) | data[0];
    }

    tr_debug("Current: %umA", (data[1] << 8) | data[0]);

    return true;
}

bool BQ35100::getUsedCapacity(uint32_t *pCapacityUAh) {
    bool success = false;
    char bytes[5];
    uint32_t data;

    if (_ready && makeAdcReading()) {
        _i2c->lock();
        // Read four bytes from the AccummulatedCapacity register address

        // Send a command to read from registerAddress
        bytes[0] = 0x02;
        bytes[1] = 0;
        bytes[2] = 0;
        bytes[3] = 0;
        bytes[4] = 0;

        if ((_i2c->write(_address, &(bytes[0]), 1) == 0) &&
                (_i2c->read(_address, &(bytes[1]), 4) == 0)) {
            success = true;
            data = (((uint32_t) bytes[4]) << 24) + (((uint32_t) bytes[3]) << 16) + (((uint32_t) bytes[2]) << 8) + bytes[1];

            // The answer is in uAh
            if (pCapacityUAh) {
                *pCapacityUAh = data;
            }

            tr_info("Battery capacity used %u uAh.", (unsigned int) data);
        }

        _i2c->unlock();
    }

    return success;
}

bool BQ35100::getRemainingCapacity(uint32_t *pCapacityUAh) {
    bool success = false;
    /* uint32_t designCapacityUAh;
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

            tr_info("Battery capacity remaining %lu uAh (from a designed capacity of %lu uAh).",
                    designCapacityUAh - usedCapacityUAh, designCapacityUAh);
        }
    } */

    return success;
}

bool BQ35100::getRemainingPercentage(int32_t *pBatteryPercentage) {
    bool success = false;
    /* uint32_t designCapacityUAh;
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

            batteryPercentage = (uint64_t)(designCapacityUAh - usedCapacityUAh) * 100 / designCapacityUAh;

            if (pBatteryPercentage) {
                *pBatteryPercentage = batteryPercentage;
            }

            tr_info("Battery capacity remaining %li%%.", batteryPercentage);
        }
    } */

    return success;
}

bool BQ35100::newBattery(uint32_t capacityMAh) {
    bool success = false;
    char data[3];

    if (_ready) {
        if (setDesignCapacity(capacityMAh)) {
            _i2c->lock();
            // Send a control command to indicate NEW_BATTERY
            data[0] = 0x3e;  // Set address to ManufacturerAccessControl
            data[1] = 0x13;  // First byte of NEW_BATTERY sub-command (0x13)
            data[2] = 0xA6;  // Second byte of NEW_BATTERY sub-command (0xA6) (register address will auto-increment)

            if (_i2c->write(_address, data, 3) == 0) {
                success = true;

                tr_info("New battery set.");
            }

            _i2c->unlock();
        }
    }

    return success;
}

bool BQ35100::advancedGetConfig(int32_t address, char *pData, int32_t length) {
    bool success = false;

    if (_ready) {
        _i2c->lock();

        success =  readExtendedData(address, pData, length);

        _i2c->unlock();
    }

    return success;
}

bool BQ35100::advancedSetConfig(int32_t address, const char *pData, int32_t length) {
    bool success = false;

    if (_ready) {
        _i2c->lock();

        success =  writeExtendedData(address, pData, length);

        _i2c->unlock();
    }

    return success;
}

bool BQ35100::advancedSendControlWord(uint16_t controlWord, uint16_t *pDataReturned) {
    bool success = false;
    char data[3];

    if (_ready) {
        _i2c->lock();

        // Send the control command
        data[0] = 0x3e;  // Set address to ManufacturerAccessControl
        data[1] = (char) controlWord;        // First byte of controlWord
        data[2] = (char)(controlWord >> 8);  // Second byte of controlWord

        if (_i2c->write(_address, data, 3) == 0) {
            // Read the two bytes returned if requested
            if (pDataReturned != NULL) {
                if (getTwoBytes(0x40, pDataReturned)) { // Read from MACData
                    success = true;
                    tr_debug("Sent control word 0x%04x, read back 0x%04x.", controlWord, *pDataReturned);
                }

            } else {
                success = true;
                tr_debug("Sent control word 0x%04x.", controlWord);
            }
        }

        _i2c->unlock();
    }

    return success;
}

bool BQ35100::advancedGet(uint8_t address, uint16_t *pDataReturned) {
    bool success = false;
    uint16_t value = 0;

    if (_ready) {
        // Make sure there's a recent reading, as most
        // of these commands involve the chip having done one
        if (makeAdcReading()) {
            _i2c->lock();

            // Read the data
            if (getTwoBytes(address, &value)) {
                success = true;
                tr_debug("Read 0x%04x from addresses 0x%02x and 0x%02x.", value, address, address + 1);

                if (pDataReturned != NULL) {
                    *pDataReturned = value;
                }
            }

            _i2c->unlock();
        }
    }

    return success;
}

BQ35100::security_mode_t BQ35100::advancedGetSecurityMode(void) {
    security_mode_t securityMode = SECURITY_MODE_UNKNOWN;

    if (_ready) {
        _i2c->lock();

        securityMode = getSecurityMode();

        _i2c->unlock();
    }

    return securityMode;
}

bool BQ35100::advancedSetSecurityMode(security_mode_t securityMode) {
    bool success = false;

    if (_ready) {
        _i2c->lock();

        success = setSecurityMode(securityMode);

        _i2c->unlock();
    }

    return success;
}

bool BQ35100::advancedReset(void) {
    bool success = false;
    security_mode_t securityMode;
    char data[3];

    if (_ready) {
        _i2c->lock();

        securityMode = getSecurityMode(); // Must be inside lock()

        // Handle unsealing, as this command only works when unsealed
        if (setSecurityMode(SECURITY_MODE_UNSEALED)) {
            // Send a RESET sub-command
            data[0] = 0x3e;  // Set address to ManufacturerAccessControl
            data[1] = 0x41;  // First byte of RESET sub-command (0x41)
            data[2] = 0x00;  // Second byte of RESET sub-command (0x00) (register address will auto-increment)

            if (_i2c->write(_address, data, 3) == 0) {
                success = true;
                tr_info("Chip hard reset.");
            }

            // Set the security mode back to what it was
            if (!setSecurityMode(securityMode)) {
                success = false;
            }
        }

        _i2c->unlock();
    }

    return success;
}

bool BQ35100::getTwoBytes(uint8_t registerAddress, uint16_t *pBytes) {
    bool success = false;
    char data[3];

    if (_i2c != NULL) {
        data[0] = registerAddress;
        data[1] = 0;
        data[2] = 0;

        // Send a command to read from registerAddress
        if ((_i2c->write(_address, data, 1, true) == 0) &&
                (_i2c->read(_address, &(data[1]), 2) == 0)) {
            success = true;

            if (pBytes) {
                *pBytes = (((uint16_t) data[2]) << 8) + data[1];
            }
        }
    }

    return success;
}

uint8_t BQ35100::computeChecksum(const char *pData, int32_t length) {
    uint8_t checkSum = 0;
    uint8_t x = 0;

    if (pData != NULL) {
#ifdef DEBUG_BQ35100_BLOCK_DATA
        printf("computing check sum on data block.");
        printf(" 0  1  2  3  4  5  6  7   8  9  A  B  C  D  E  F");
#endif

        for (x = 1; x <= length; x++) {
            checkSum += *pData;

#ifdef DEBUG_BQ35100_BLOCK_DATA

            if (x % 16 == 8) {
                printf("%02x  ", *pData);

            } else if (x % 16 == 0) {
                printf("%02x", *pData);

            } else {
                printf("%02x-", *pData);
            }

#endif
            pData++;
        }

        checkSum = 0xff - checkSum;
    }

#ifdef DEBUG_BQ35100_BLOCK_DATA

    if (x % 16 !=  1) {
        printf("");
    }

#endif

    tr_debug("Checksum is 0x%02x.", checkSum);

    return checkSum;
}

bool BQ35100::readExtendedData(uint16_t address, char *pData, int32_t length) {
    int32_t lengthRead;
    bool success = false;
    security_mode_t securityMode = getSecurityMode();
    char block[32 + 2 + 2]; // 32 bytes of data, 2 bytes of address,
    // 1 byte of MACDataSum and 1 byte of MACDataLen
    char data[3];

    // Handle security mode
    if (setSecurityMode(SECURITY_MODE_UNSEALED)) {
        if ((_i2c != NULL) && (length <= 32) && (address >= 0x4000) && (address < 0x4400) && (pData != NULL)) {
            tr_debug("Preparing to read %li byte(s) from address 0x%04x.", length, address);

            // Enable Block Data Control (0x61)
            data[0] = 0x61;
            data[1] = 0;

            if (_i2c->write(_address, data, 2) == 0) {
                // Write address to ManufacturerAccessControl (0x3e)
                data[0] = 0x3e;
                data[1] = (char) address;
                data[2] = (char)(address >> 8);

                if (_i2c->write(_address, data, 3) == 0) {
                    // Read the address from ManufacturerAccessControl (0x3e then 0x3f),
                    // data from MACData (0x40 to 0x5f), checksum from MACDataSum (0x60)
                    // and length from MACDataLen (0x61)
                    if ((_i2c->write(_address, data, 1, true) == 0) &&
                            (_i2c->read(_address, &(block[0]), sizeof(block)) == 0)) {
                        // Check that the address matches
                        if ((block[0] == (char) address) && (block[1] == (char)(address >> 8))) {
                            // Check that the checksum matches (-2 on MACDataLen as it includes MACDataSum and itself)
                            if (block[34] == computeChecksum(&(block[0]), block[35] - 2)) {
                                // All is good, copy the data to the user
                                lengthRead = block[35] - 4; // -4 rather than -2 to remove the two bytes of address as well

                                if (lengthRead > length) {
                                    lengthRead = length;
                                }

                                memcpy(pData, &(block[2]), lengthRead);
                                success = true;
                                tr_debug("%li byte(s) read successfully.", lengthRead);

                            } else {
                                tr_error("Checksum didn't match (0x%02x expected).", block[34]);
                            }

                        } else {
                            tr_error("address didn't match (expected 0x%04x, received 0x%02x%02x).", address, block[1], block[0]);
                        }

                    } else {
                        tr_error("Unable to read %d bytes from ManufacturerAccessControl.", sizeof(block));
                    }

                } else {
                    tr_error("Unable to write %d bytes to ManufacturerAccessControl.", 3);
                }

            } else {
                tr_error("Unable to set Block Data Control.");
            }

        } else {
            tr_error("Unable to set the security mode of the chip.");
        }
    }

    // Put the security mode back to what it was
    if (!setSecurityMode(securityMode)) {
        success = false;
    }

    return success;
}

bool BQ35100::writeExtendedData(uint16_t address, const char *pData, int32_t length) {
    bool success = false;
    security_mode_t securityMode = getSecurityMode();
    char data[3 + 32];
    uint16_t controlStatus;

    if ((_i2c != NULL) && (length <= 32) && (address >= 0x4000) && (address < 0x4400) && (pData != NULL)) {
        tr_debug("Preparing to write %li byte(s) to address 0x%04x.", length, address);

        // Handle security mode
        if (setSecurityMode(SECURITY_MODE_UNSEALED)) {
            // Enable Block Data Control (0x61)
            data[0] = 0x61;
            data[1] = 0;

            if (_i2c->write(_address, data, 2) == 0) {
                // Start write at ManufacturerAccessControl (0x3e)
                data[0] = 0x3e;
                // Next two bytes are the address we will write to
                data[1] = (char) address;
                data[2] = (char)(address >> 8);
                // Remaining bytes are the data bytes we wish to write
                memcpy(&(data[3]), pData, length);

                if (_i2c->write(_address, data, 3 + length) == 0) {
                    // Compute the checksum and write it to MACDataSum (0x60)
                    data[1] = computeChecksum(&(data[1]), length + 2);
                    data[0] = 0x60;

                    if (_i2c->write(_address, data, 2) == 0) {
                        // Write 4 + length to MACDataLen (0x61)
                        data[1] = length + 4;
                        data[0] = 0x61;

                        if (_i2c->write(_address, data, 2) == 0) {
                            // Read the control status register to see if a bad
                            // flash write has been detected (bit 15)
                            data[0] = 0;

                            if ((_i2c->write(_address, data, 1) == 0) && getTwoBytes(0, &controlStatus) &&
                                    (((controlStatus >> 15) & 0x01) != 0x01)) {
                                success = true;
                            }

                            tr_info("Write successful.");

                        } else {
                            tr_error("Unable to read write to MACDataLen.");
                        }

                    } else {
                        tr_error("Unable to write to MACDataSum.");
                    }

                } else {
                    tr_error("Unable to write %li bytes to ManufacturerAccessControl.", length + 2);
                }

            } else {
                tr_error("Unable to set Block Data Control.");
            }

        } else {
            printf("Unable to set the security mode of the chip.");
        }
    }

    // Put the security mode back to what it was
    if (!setSecurityMode(securityMode)) {
        success = false;
    }

    return success;
}

BQ35100::security_mode_t BQ35100::getSecurityMode(void) {
    security_mode_t securityMode = SECURITY_MODE_UNKNOWN;
    char data[1];
    uint16_t controlStatus;

    // Read the control status register
    data[0] = 0;

    if ((_i2c->write(_address, data, 1) == 0) &&
            getTwoBytes(0, &controlStatus)) {
        // Bits 13 and 14 of the high byte represent the security status,
        // 01 = full access
        // 10 = unsealed access
        // 11 = sealed access
        securityMode = (security_mode_t)((controlStatus >> 13) & 0x03);
        tr_info("Security mode is 0x%02x (control status 0x%04x).", securityMode, controlStatus);
    }

    return securityMode;
}

bool BQ35100::setSecurityMode(security_mode_t securityMode) {
    bool success = false;
    char data[3];
    security_mode_t currentSecurityMode = getSecurityMode();

    if (securityMode != SECURITY_MODE_UNKNOWN) {
        if (securityMode != currentSecurityMode) {
            // For reasons that aren't clear, the BQ35100 sometimes refuses
            // to change security mode if a previous security mode change
            // happend only a few seconds ago, hence the retry here
            for (int32_t x = 0; (x < SET_SECURITY_MODE_RETRY) && !success; x++) {
                data[0] = 0x3e;  // Set address to ManufacturerAccessControl

                switch (securityMode) {
                    case SECURITY_MODE_SEALED:
                        // Just seal the chip
                        data[1] = 0x20;  // First byte of SEALED sub-command (0x20)
                        data[2] = 0x00;  // Second byte of SEALED sub-command (0x00) (register address will auto-increment)
                        _i2c->write(_address, data, 3);
                        break;

                    case SECURITY_MODE_FULL_ACCESS:
                        // Send the full access code with endianness conversion
                        // in TWO writes
                        data[2] = (char)(_fullAccessCodes >> 24);
                        data[1] = (char)(_fullAccessCodes >> 16);
                        _i2c->write(_address, data, 3);
                        data[2] = (char)(_fullAccessCodes >> 8);
                        data[1] = (char) _fullAccessCodes;
                        _i2c->write(_address, data, 3);
                        break;

                    case SECURITY_MODE_UNSEALED:
                        data[2] = (char)(_sealCodes >> 24);
                        data[1] = (char)(_sealCodes >> 16);
                        _i2c->write(_address, data, 3);
                        data[2] = (char)(_sealCodes >> 8);
                        data[1] = (char) _sealCodes;
                        _i2c->write(_address, data, 3);
                        break;

                    case SECURITY_MODE_UNKNOWN:
                    default:
                        MBED_ASSERT(false);
                        break;
                }

                currentSecurityMode = getSecurityMode();

                if (currentSecurityMode == securityMode) {
                    success = true;
                    tr_info("Security mode is now 0x%02x.", currentSecurityMode);

                } else {
                    ThisThread::sleep_for(1s);
                    tr_error("Security mode set failed (wanted 0x%02x, got 0x%02x), will retry.",
                             securityMode, currentSecurityMode);
                }
            }

        } else {
            success = true;
        }
    }

    return success;
}

bool BQ35100::makeAdcReading(void) {
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
