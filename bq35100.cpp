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

BQ35100::BQ35100(uint32_t seal_codes, uint8_t address):
    _seal_codes(seal_codes),
    _address(address) {
}

BQ35100::BQ35100(PinName sda, PinName scl, uint32_t seal_codes, uint8_t address, uint32_t frequency):
    _seal_codes(seal_codes),
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

bool BQ35100::init(I2C *i2c_obj, PinName gauge_enable_pin) {
    uint16_t answer;

    if (i2c_obj != nullptr) {
        _i2c = i2c_obj;
    }

    MBED_ASSERT(_i2c);

    if (gauge_enable_pin != NC) {
        _gauge_enable_pin = new DigitalOut(gauge_enable_pin, 1);
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

    // Get security mode & status
    if (!getCntl(CNTL_CONTROL_STATUS, &answer)) {
        tr_error("Couldn't get security mode");
        return false;
    }

    _security_mode = (security_mode_t)((answer >> 13) & 0b011);

    switch ((answer >> 13) & 0b011) {
        case SECURITY_FULL_ACCESS:
            tr_info("Device is in FULL ACCESS mode");
            break;

        case SECURITY_UNSEALED:
            tr_info("Device is in UNSEALED mode");
            break;

        case SECURITY_SEALED:
            tr_info("Device is in SEALED mode");
            break;

        default:
            tr_error("Invalid device mode");
            return false;
    }

    // Status
    _enabled = answer & 0b1;

    return true;
}

bool BQ35100::enableGauge() {
    uint16_t answer;

    if (_enabled) {
        tr_warning("Gauge already enabled");
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
        tr_warning("Gauge already disabled");
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

    if (_gauge_enable_pin) {
        _gauge_enable_pin->write(0);
    }

    return !_enabled;
}

bool BQ35100::isGaugeEnabled(void) {
    return _enabled;
}

bool BQ35100::setDesignCapacity(uint16_t capacity) {
    char data[2];

    data[0] = capacity >> 8;
    data[1] = capacity;

    tr_info("Setting designed cell capacity to %u mAh", capacity);

    if (!writeExtendedData(0x41FE, data, sizeof(data))) {
        tr_error("Could not set the capacity");
        return false;
    }

    return true;
}

bool BQ35100::getDesignCapacity(uint16_t *capacity) {
    char data[2];

    if (!getData(CMD_DESIGN_CAPACITY, data, 2)) {
        return false;
    }

    if (capacity) {
        *capacity = (data[1] << 8) | data[0];
    }

    tr_info("Designed cell capacity is %u mAh", (data[1] << 8) | data[0]);

    return true;
}

bool BQ35100::getTemperature(int16_t *temp) {
    char data[2];
    int16_t temp_c = SHRT_MIN;

    if (!getData(CMD_TEMPERATURE, data, 2)) {
        tr_error("Could not get temperature");
        return false;
    }

    temp_c = ((data[1] << 8) | data[0]) / 10 - 273;

    if (temp) {
        *temp = temp_c;
    }

    tr_info("Temperature: %uK = %i*C", (data[1] << 8) | data[0], temp_c);

    return true;
}


bool BQ35100::getInternalTemperature(int16_t *temp) {
    char data[2];
    int16_t temp_c = SHRT_MIN;

    if (!getData(CMD_INTERNAL_TEMPERATURE, data, 2)) {
        tr_error("Could not get internal temperature");
        return false;
    }

    temp_c = ((data[1] << 8) | data[0]) / 10 - 273;

    if (temp) {
        *temp = temp_c;
    }

    tr_info("Internal temperature: %uK = %i*C", (data[1] << 8) | data[0], temp_c);

    return true;
}

bool BQ35100::getVoltage(uint16_t *voltage) {
    char data[2];

    if (!getData(CMD_VOLTAGE, data, 2)) {
        tr_error("Could not voltage reading");
        return false;
    }

    if (voltage) {
        *voltage = (data[1] << 8) | data[0];
    }

    tr_info("Battery voltage: %umV", (data[1] << 8) | data[0]);

    return true;
}

bool BQ35100::getCurrent(uint16_t *current) {
    char data[2];

    if (!getData(CMD_CURRENT, data, 2)) {
        tr_error("Could not current reading");
        return false;
    }

    if (current) {
        *current = (data[1] << 8) | data[0];
    }

    tr_info("Current: %umA", (data[1] << 8) | data[0]);

    return true;
}

bool BQ35100::getUsedCapacity(uint32_t *capacity_used) {
    char data[4];

    if (!getData(CMD_ACCUMULATED_CAPACITY, data, 4)) {
        tr_error("Could not get used capacity");
        return false;
    }

    if (capacity_used) {
        *capacity_used = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
    }

    tr_info("Battery capacity used %i uAh", (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);
    return true;
}

bool BQ35100::getRemainingCapacity(uint16_t *remaining_capacity) {
    uint16_t design_capacity;
    uint32_t used_capacity;

    // First, get the designed capacity
    if (!getDesignCapacity(&design_capacity)) {
        return false;
    }

    // Then get the used capacity
    if (!getUsedCapacity(&used_capacity)) {
        return false;
    }

    // Correct units
    used_capacity += 500; // round up
    used_capacity /= 1000; // uAh to mAh

    // Limit the result
    if (used_capacity > design_capacity) {
        used_capacity = design_capacity;
    }

    // Copy
    if (remaining_capacity) {
        *remaining_capacity = design_capacity - used_capacity;
    }

    tr_info("Battery capacity remaining %lu uAh (from a designed capacity of %u mAh)",
            design_capacity - used_capacity, design_capacity);

    return true;
}

bool BQ35100::getRemainingPercentage(uint8_t *battery_percentage) {
    uint16_t design_capacity;
    uint32_t used_capacity;

    // First, get the designed capacity
    if (!getDesignCapacity(&design_capacity)) {
        return false;
    }

    // Then get the used capacity
    if (!getUsedCapacity(&used_capacity)) {
        return false;
    }

    // Correct units
    used_capacity += 500; // round up
    used_capacity /= 1000; // uAh to mAh

    // Limit the result
    if (used_capacity > design_capacity) {
        used_capacity = design_capacity;
    }

    // Copy
    if (battery_percentage) {
        *battery_percentage = (design_capacity - used_capacity) * 100 / design_capacity;
    }

    tr_info("Battery capacity remaining %lu%%", (design_capacity - used_capacity) * 100 / design_capacity);

    return true;
}

bool BQ35100::newBattery(uint16_t capacity) {
    if (capacity != 0 && !setDesignCapacity(capacity)) {
        return false;
    }

    if (!sendCntl(CNTL_NEW_BATTERY)) {
        tr_error("Could not set new battery");
        return false;
    }

    tr_info("New battery set");

    return true;
}

bool BQ35100::reset(void) {
    bool success = false;
    security_mode_t prev_security_mode = _security_mode;

    if (_security_mode == SECURITY_UNKNOWN) {
        tr_error("Security mode unknown");
        return false;
    }

    if (_security_mode == SECURITY_SEALED && !setSecurityMode(SECURITY_UNSEALED)) {
        return false;
    }

    if (sendCntl(CNTL_RESET)) {
        success = true;
    }

    if (prev_security_mode != _security_mode) { // in case we changed the mode
        success = setSecurityMode(prev_security_mode);
    }

    return success;
}

bool BQ35100::setGaugeMode(gauge_mode_t gauge_mode) {
    char op[1];

    if (gauge_mode == UNKNOWN_MODE) {
        tr_error("Invalid gauge mode");
        MBED_ASSERT(false);
    }

    // Get the Operation Config A
    if (!readExtendedData(0x41B1, op, sizeof(op))) {
        return false;
    }

    // Set the mode (GMSEL 1:0)
    if ((gauge_mode_t)(op[0] & 0b11) != gauge_mode) {
        op[0] &= ~0b11;
        op[0] |= (char)gauge_mode;

        if (!writeExtendedData(0x41B1, op, sizeof(op))) {
            return false;
        }

        tr_info("Gauge mode set");

    } else {
        tr_warning("Gauge mode already set");
    }

    return true;
}

bool BQ35100::useInternalTemp(bool use) {
    char op[1];

    // Get the Operation Config A
    if (!readExtendedData(0x41B1, op, sizeof(op))) {
        return false;
    }

    if (!(op[0] >> 7) != use) {
        if (use) {
            op[0] &= ~0b10000000;

        } else {
            op[0] |= 0b10000000;
        }

        if (!writeExtendedData(0x41B1, op, sizeof(op))) {
            return false;
        }

        tr_info("Temperature setting set");

    } else {
        tr_warning("Temperature setting already set");
    }

    return true;
}

uint8_t BQ35100::computeChecksum(const char *data, size_t length) {
    uint8_t checksum = 0;
    uint8_t x = 0;

    if (data) {
        for (x = 1; x <= length; x++) {
            checksum += *data;
            data++;
        }

        checksum = 0xff - checksum;
    }

    tr_debug("Checksum is 0x%02X", checksum);

    return checksum;
}

bool BQ35100::readExtendedData(uint16_t address, char *response, size_t len) {
    size_t length_read;
    char data[32 + 2 + 2]; // 32 bytes of data, 2 bytes of address,

    bool success = false;
    security_mode_t prev_security_mode = _security_mode;

    if (_security_mode == SECURITY_UNKNOWN) {
        tr_error("Security mode unknown");
        return false;
    }

    if (address < 0x4000 || address > 0x43FF || !response) {
        tr_error("Invalid input data");
        return false;
    }

    if (_security_mode == SECURITY_SEALED && !setSecurityMode(SECURITY_UNSEALED)) {
        return false;
    }

    tr_debug("Preparing to read %u byte(s) from address 0x%04X", len, address);

    data[0] = address & UCHAR_MAX;
    data[1] = address >> 8;

    if (!sendData(CMD_MAC, data, 2)) {
        tr_error("Unable to write to ManufacturerAccessControl");
        goto END;
    }

    data[0] = CMD_MAC;

    if (!write(data, 1, true) || !read(data, sizeof(data))) {
        tr_error("Unable to read from ManufacturerAccessControl");
        goto END;
    }

    // Check that the address matches
    if (data[0] != (char)address || data[1] != (char)(address >> 8)) {
        tr_error("Address didn't match (expected 0x%04X, received 0x%02X%02X)", address, data[1], data[0]);
        goto END;
    }

    // Check that the checksum matches (-2 on MACDataLen as it includes MACDataSum and itself)
    if (data[34] != computeChecksum(data, data[35] - 2)) {
        tr_error("Checksum didn't match (0x%02X expected)", data[34]);
        goto END;
    }

    // All is good
    length_read = data[35] - 4; // -4 rather than -2 to remove the two bytes of address as well

    if (length_read > len) {
        length_read = len;
    }

    memcpy(response, data + 2, length_read);
    success = true;
    tr_debug("Success data read(%u): %s", length_read, tr_array(reinterpret_cast<uint8_t *>(response), length_read));

END:

    if (prev_security_mode != _security_mode) { // in case we changed the mode
        success = setSecurityMode(prev_security_mode);
    }

    return success;
}

bool BQ35100::writeExtendedData(uint16_t address, const char *data, size_t len) {
    uint16_t answer;
    char d[32 + 3]; // Max data len + header

    bool success = false;
    security_mode_t prev_security_mode = _security_mode;

    if (_security_mode == SECURITY_UNKNOWN) {
        tr_error("Security mode unknown");
        return false;
    }

    if (address < 0x4000 || address > 0x43FF || len < 1 || len > 32 || !data) {
        tr_error("Invalid input data");
        return false;
    }

    if (_security_mode == SECURITY_SEALED && !setSecurityMode(SECURITY_UNSEALED)) {
        return false;
    }

    tr_debug("Preparing to write %u byte(s) to address 0x%04X %s", len, address,
             tr_array(reinterpret_cast<uint8_t *>(d), len));

    d[0] = CMD_MAC;
    d[1] = address & UCHAR_MAX;
    d[2] = address >> 8;

    // Remaining bytes are the d bytes we wish to write
    memcpy(d + 3, data, len);

    if (!write(d, 3 + len)) {
        tr_error("Unable to write to ManufacturerAccessControl");
        goto END;
    }

    // Compute the checksum and write it to MACDataSum (0x60)
    d[0] = CMD_MAC_DATA_SUM;
    d[1] = computeChecksum(d + 1, len + 2);

    if (!write(d, 2)) {
        tr_error("Unable to write to MACDataSum");
    }

    // Write 4 + len to MACDataLen (0x61)
    d[0] = CMD_MAC_DATA_LEN;
    d[1] = len + 4;

    if (!write(d, 2)) {
        tr_error("Unable to write to MACDataLen");
        goto END;
    }

    if (!getCntl(CNTL_CONTROL_STATUS, &answer)) {
        tr_error("Get status failed");
        goto END;
    }

    if (answer & 0b1000000000000000) {
        tr_error("Write failed");
        goto END;
    }

    success = true;
    tr_info("Write successful");

END:

    if (prev_security_mode != _security_mode) { // in case we changed the mode
        success = setSecurityMode(prev_security_mode);
    }

    return success;
}

BQ35100::security_mode_t BQ35100::getSecurityMode(void) {
    uint16_t answer;

    if (!getCntl(CNTL_CONTROL_STATUS, &answer)) {
        tr_error("Couldn't get security mode");
        return SECURITY_UNKNOWN;
    }

    switch ((answer >> 13) & 0b011) {
        case SECURITY_FULL_ACCESS:
            tr_info("Device is in FULL ACCESS mode");
            break;

        case SECURITY_UNSEALED:
            tr_info("Device is in UNSEALED mode");
            break;

        case SECURITY_SEALED:
            tr_info("Device is in SEALED mode");
            break;

        default:
            tr_error("Invalid device mode");
            return SECURITY_UNKNOWN;
    }

    return (security_mode_t)((answer >> 13) & 0b011);
}

bool BQ35100::setSecurityMode(security_mode_t new_security) {
    bool success = false;
    char data[4];

    if (new_security == _security_mode) {
        return true; // We are already in this mode
    }

    if (new_security == SECURITY_UNKNOWN) {
        tr_error("Invalid access mode");
        return false;
    }

    // For reasons that aren't clear, the BQ35100 sometimes refuses
    // to change security mode if a previous security mode change
    // happend only a few seconds ago, hence the retry here
    for (auto x = 0; (x < SET_SECURITY_MODE_RETRY) && !success; x++) {
        data[0] = CMD_MAC;

        switch (new_security) {
            case SECURITY_SEALED:
                sendCntl(CNTL_SEAL);
                break;

            case SECURITY_FULL_ACCESS: {
                // Unseal first if in Sealed mode
                if (_security_mode == SECURITY_SEALED && !setSecurityMode(SECURITY_UNSEALED)) {
                    return false;
                }

                if (!readExtendedData(0X41D0, data, sizeof(data))) {
                    tr_error("Could not get full access codes");
                    return false;
                }

                uint32_t full_access_codes = (data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3];

                // Send the full access code with endianness conversion
                // in TWO writes
                data[2] = (full_access_codes >> 24) & 0xFF;
                data[1] = (full_access_codes >> 16) & 0xFF;
                write(data, 3);

                data[2] = (full_access_codes >> 8) & 0xFF;
                data[1] = full_access_codes & 0xFF;
                write(data, 3);
            }
            break;

            case SECURITY_UNSEALED: {
                // Seal first if in Full Access mode
                if (_security_mode == SECURITY_FULL_ACCESS && !setSecurityMode(SECURITY_SEALED)) {
                    return false;
                }

                data[2] = (_seal_codes >> 24) & 0xFF;
                data[1] = (_seal_codes >> 16) & 0xFF;
                write(data, 3);

                data[2] = (_seal_codes >> 8) & 0xFF;
                data[1] = _seal_codes & 0xFF;
                write(data, 3);
            }
            break;

            case SECURITY_UNKNOWN:
            default:
                MBED_ASSERT(false);

                break;
        }

        _security_mode = getSecurityMode();

        if (_security_mode == new_security) {
            success = true;
            tr_info("Security mode set");

        } else {
            ThisThread::sleep_for(1s);
            tr_error("Security mode set failed (wanted 0x%02X, got 0x%02X), will retry", new_security, _security_mode);
        }
    }

    return success;
}
