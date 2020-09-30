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

BQ35100::BQ35100(PinName gauge_enable_pin, uint32_t seal_codes, uint8_t address):
    _seal_codes(seal_codes),
    _address(address) {

    if (gauge_enable_pin != NC) {
        _gauge_enable_pin = new DigitalOut(gauge_enable_pin, 0);
    }
}

BQ35100::BQ35100(PinName sda, PinName scl, PinName gauge_enable_pin, uint32_t seal_codes, uint8_t address,
                 uint32_t frequency):
    _seal_codes(seal_codes),
    _address(address) {
    _i2c = new (_i2c_obj) I2C(sda, scl);
    _i2c->frequency(frequency);

    if (gauge_enable_pin != NC) {
        _gauge_enable_pin = new DigitalOut(gauge_enable_pin, 0);
    }
}

BQ35100::~BQ35100(void) {
    if (_i2c == reinterpret_cast<I2C *>(_i2c_obj)) {
        _i2c->~I2C();
    }
}

bool BQ35100::init(I2C *i2c_obj) {
    uint16_t answer;
    bool success = false;

    if (i2c_obj != nullptr) {
        _i2c = i2c_obj;
    }

    MBED_ASSERT(_i2c);

    if (_gauge_enable_pin) {
        _gauge_enable_pin->write(1);
        ThisThread::sleep_for(100ms);
    }

    // Get device type
    for (auto i = 0; (i < MBED_CONF_BQ35100_INIT_RETRY) && !success; i++) {
        success = getCntl(CNTL_HW_VERSION, &answer);

        if (!success) {
            ThisThread::sleep_for(100ms);
        }
    }

    if (answer != 0x00A8) {
        tr_error("Different device type");
        return false;
    }

    // Get security mode & status
    if (!getStatus(&answer)) {
        return false;
    }

    _security_mode = (bq35100_security_t)((answer >> 13) & 0b011);

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

    // Wait for initialization
    if (!(answer & BQ3500_INITCOMP_BIT_MASK)) {
        tr_debug("Device initialization not complete");

        if (!waitforStatus(BQ3500_INITCOMP_BIT_MASK, BQ3500_INITCOMP_BIT_MASK)) {
            tr_error("Device initialization failed");
            return false;
        }
    }

    // Status
    _enabled = answer & 0b1;

    return true;
}

bool BQ35100::startGauge(void) {
    if (_enabled) {
        tr_warning("Gauge already enabled");
        return true;
    }

    if (!sendCntl(CNTL_GAUGE_START)) {
        tr_error("Error enabling gauge");
        return false;
    }

    _enabled = waitforStatus(BQ3500_GA_BIT_MASK, BQ3500_GA_BIT_MASK);

    if (_enabled) {
        tr_info("Gauge enabled");

    } else {
        tr_error("Gauge not enabled");
    }

    return _enabled;
}

bool BQ35100::stopGauge(void) {
    if (!_enabled) {
        tr_warning("Gauge already disabled");
        return true;
    }

    if (!sendCntl(CNTL_GAUGE_STOP)) {
        tr_error("Error disabling gauge");
        return false;
    }

    // Stopping takes a lot of time
    if (waitforStatus(0, BQ3500_GA_BIT_MASK, 500ms)) {
        tr_info("Gauge stopped");
        _enabled = false;

    } else {
        tr_error("Gauge not stopped");
    }

    return !_enabled;
}

bool BQ35100::disableGauge(void) {
    bool success = false;

    if (!stopGauge()) {
        return false;
    }

    success = waitforStatus(BQ3500_G_DONE_BIT_MASK, BQ3500_G_DONE_BIT_MASK);

    if (success) {
        tr_info("Gauge disabled");

    } else {
        tr_error("Gauge not disabled");
    }

    if (_gauge_enable_pin) {
        _gauge_enable_pin->write(0);
    }

    return success;
}

bool BQ35100::setGaugeMode(bq35100_gauge_mode_t gauge_mode) {
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
    if ((bq35100_gauge_mode_t)(op[0] & 0b11) != gauge_mode) {
        op[0] &= ~0b11;
        op[0] |= (char)gauge_mode;

        ThisThread::sleep_for(40ms);

        if (!writeExtendedData(0x41B1, op, sizeof(op))) {
            return false;
        }

        tr_info("Gauge mode set");

    } else {
        tr_warning("Gauge mode already set");
    }

    return true;
}

bool BQ35100::enableLifetime(bool enable) {
    char op[1];

    // Get the Operation Config A
    if (!readExtendedData(0x41B1, op, sizeof(op))) {
        return false;
    }

    bool state = op[0] & 0b10000;

    if (state == enable) {
        tr_warning("LF already set");
        return true;
    }

    // Set the LF_EN
    op[0] &= ~0b10000;

    if (enable) {
        op[0] |= 0b10000;
    }

    ThisThread::sleep_for(40ms);

    if (!writeExtendedData(0x41B1, op, sizeof(op))) {
        return false;
    }

    tr_info("LF set");

    return true;
}

bool BQ35100::getStatus(uint16_t *status) {
    char data[2];
    data[0] = (char)CMD_CONTROL;

    if (!write(data, 1)) {
        tr_error("Error writing data");
        return false;
    }

    if (!read(data, sizeof(data))) {
        tr_error("Couldn't get device mode");
        return false;
    }

    if (status) {
        *status = (data[1] << 8) | data[0];
    }

    return true;
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

bool BQ35100::useInternalTemp(bool internal) {
    char op[1];

    // Get the Operation Config A
    if (!readExtendedData(0x41B1, op, sizeof(op))) {
        return false;
    }

    if (!(op[0] >> 7) != internal) {
        if (internal) {
            op[0] &= ~0b10000000;

        } else {
            op[0] |= 0b10000000;
        }

        ThisThread::sleep_for(40ms);

        if (!writeExtendedData(0x41B1, op, sizeof(op))) {
            return false;
        }

        tr_info("Temperature setting set");

    } else {
        tr_warning("Temperature setting already set");
    }

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

bool BQ35100::setUnderTemperatureThreshold(int16_t min) {
    char data[2];
    data[0] = min & UCHAR_MAX;
    data[1] = min >> 8;

    if (!writeExtendedData(0x41E0, data, sizeof(data))) {
        tr_error("Sending under temperature temp failed");
        return false;
    }

    tr_info("Under temperature set to: %i", min);

    return true;
}

bool BQ35100::setUnderTemperatureClear(int16_t clear) {
    char data[2];
    data[0] = clear & UCHAR_MAX;
    data[1] = clear >> 8;

    if (!writeExtendedData(0x41E3, data, sizeof(data))) {
        tr_error("Sending clear under temperature temp failed");
        return false;
    }

    tr_info("Clear under temperature set to: %i", clear);

    return true;
}

bool BQ35100::setLowBatteryThreshold(uint16_t voltage) {
    char data[2];
    data[0] = voltage & UCHAR_MAX;
    data[1] = voltage >> 8;

    if (!writeExtendedData(0x41DB, data, sizeof(data))) {
        tr_error("Sending low battery threshold failed");
        return false;
    }

    tr_info("Low battery threshold set to: %u", voltage);

    return true;
}

bool BQ35100::getVoltage(uint16_t *voltage) {
    char data[2];

    if (!getData(CMD_VOLTAGE, data, sizeof(data))) {
        tr_error("Could not voltage reading");
        return false;
    }

    if (voltage) {
        *voltage = (data[1] << 8) | data[0];
    }

    tr_info("Battery voltage: %umV", (data[1] << 8) | data[0]);

    return true;
}

bool BQ35100::getCurrent(int16_t *current) {
    char data[2];

    if (!_enabled) {
        tr_warning("Gauge not enabled");
        return false;
    }

    if (!getData(CMD_CURRENT, data, 2)) {
        tr_error("Could not make current reading");
        return false;
    }

    if (current) {
        *current = (data[1] << 8) | data[0];
    }

    tr_info("Current: %imA", (int16_t)((data[1] << 8) | data[0]));

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

    tr_info("Battery capacity used %u uAh", (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);
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

    if (used_capacity != 0) {
        used_capacity = ULONG_MAX - used_capacity + 1; // used capacity is counted from zero - used uA

        // Correct units
        used_capacity += 500; // round up
        used_capacity /= 1000; // uAh to mAh
    }

    // Limit the result
    if (used_capacity > design_capacity) {
        used_capacity = design_capacity;
    }

    // Copy
    if (remaining_capacity) {
        *remaining_capacity = design_capacity - used_capacity;
    }

    tr_info("Battery capacity remaining %lu mAh (from a designed capacity of %u mAh)",
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

    if (used_capacity != 0) {
        // used capacity is counted from zero - used uAh
        used_capacity = ULONG_MAX - used_capacity + 1;

        // Correct units
        used_capacity += 500; // round up
        used_capacity /= 1000; // uAh to mAh
    }

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

bool BQ35100::getBatteryStatus(uint8_t *status) {
    char data[1];

    if (!getData(CMD_BATTERY_STATUS, data, sizeof(data))) {
        tr_error("Could not get battery status");
        return false;
    }

    if (status) {
        *status = data[0];
    }

    tr_info("Battery status: %02X", data[0]);

    return true;
}

bool BQ35100::setBatteryAlert(uint8_t alert) {
    char data[1];
    data[0] = alert;

    if (!writeExtendedData(0x41B2, data, sizeof(data))) {
        tr_error("Could not set battery alert");
        return false;
    }

    tr_info("Battery alert set to: %02X", alert);

    return true;
}

bool BQ35100::getBatteryAlert(uint8_t *alert) {
    char data[1];

    if (!getData(CMD_BATTERY_ALERT, data, sizeof(data))) {
        tr_error("Could not battery alert");
        return false;
    }

    if (alert) {
        *alert = data[0];
    }

    tr_info("Battery alert: %02X", data[0]);

    return true;
}

bool BQ35100::setEosDataSeconds(uint8_t seconds) {
    char data[1];
    data[0] = seconds;

    if (!writeExtendedData(0x4255, data, sizeof(data))) {
        tr_error("Could not set R data seconds");
        return false;
    }

    return true;
}

bool BQ35100::reset(void) {
    if (_security_mode == SECURITY_UNKNOWN) {
        tr_error("Security mode unknown");
        return false;
    }

    if (_security_mode == SECURITY_SEALED && !setSecurityMode(SECURITY_UNSEALED)) {
        return false;
    }

    if (!sendCntl(CNTL_RESET)) {
        return false;
    }

    _security_mode = SECURITY_UNKNOWN;

    return true;
}

bool BQ35100::setSecurityMode(bq35100_security_t new_security) {
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
    for (auto x = 0; (x < MBED_CONF_BQ35100_RETRY) && !success; x++) {
        data[0] = CMD_MAC;

        switch (new_security) {
            case SECURITY_SEALED:
                tr_debug("Setting security to SEALED");
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

                tr_debug("Setting security to FULL ACCESS");

                // Send the full access code with endianness conversion
                // in TWO writes
                data[2] = (full_access_codes >> 24) & 0xFF;
                data[1] = (full_access_codes >> 16) & 0xFF;

                if (write(data, 3)) {
                    data[2] = (full_access_codes >> 8) & 0xFF;
                    data[1] = full_access_codes & 0xFF;
                    write(data, 3);
                }
            }
            break;

            case SECURITY_UNSEALED: {
                // Seal first if in Full Access mode
                if (_security_mode == SECURITY_FULL_ACCESS && !setSecurityMode(SECURITY_SEALED)) {
                    return false;
                }

                tr_debug("Setting security to UNSEALED");

                data[2] = (_seal_codes >> 24) & 0xFF;
                data[1] = (_seal_codes >> 16) & 0xFF;

                if (write(data, 3)) {
                    data[2] = (_seal_codes >> 8) & 0xFF;
                    data[1] = _seal_codes & 0xFF;

                    write(data, 3);
                }
            }
            break;

            case SECURITY_UNKNOWN:
            default:
                MBED_ASSERT(false);

                break;
        }

        ThisThread::sleep_for(40ms); // always wait after writing codes
        _security_mode = getSecurityMode();

        if (_security_mode == new_security) {
            success = true;
            tr_info("Security mode set");

        } else {
            tr_error("Security mode set failed (wanted 0x%02X, got 0x%02X), will retry", new_security, _security_mode);
            ThisThread::sleep_for(40ms);
        }
    }

    return success;
}

BQ35100::bq35100_security_t BQ35100::getSecurityMode(void) {
    uint16_t answer;

    if (!getStatus(&answer)) {
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

    return (bq35100_security_t)((answer >> 13) & 0b011);
}

bool BQ35100::calibrateVoltage(int16_t voltage) {
    char data[1];
    int16_t avg_voltage;
    int32_t offset;

    // Get avg raw voltage
    if (!getRawCalibrationData(CAL_VOLTAGE, &avg_voltage)) {
        return false;
    }

    offset = voltage - avg_voltage;

    tr_info("Voltage calibration difference: %li", offset);

    if (offset < -128 || offset > 127) {
        tr_error("Invalid voltage offset");
        return false;
    }

    data[0] = (int8_t)offset;

    // Save offset
    ThisThread::sleep_for(1s);

    if (!writeExtendedData(0x400F, data, sizeof(data))) {
        return false;
    }

    return true;
}

bool BQ35100::performCCOffset(void) {
    if (!enterCalibrationMode(true)) {
        return false;
    }

    ThisThread::sleep_for(1s);

    while (true) {
        if (!sendCntl(CNTL_CC_OFFSET)) {
            tr_error("Error sending CC offset");
            return false;
        }

        if (!waitforStatus(BQ3500_CCA_BIT_MASK, BQ3500_CCA_BIT_MASK)) { // wait for CCA == 1
            tr_error("Status: CCA != 1");
            continue;

        } else {
            tr_debug("CCA set");
            break;
        }
    }

    tr_info("Performing CC offset");
    ThisThread::sleep_for(500ms);

    if (!waitforStatus(0, BQ3500_CCA_BIT_MASK, 1s)) { // wait for CCA == 0
        tr_error("Status: CCA != 0");
        return false;
    }

    if (!sendCntl(CNTL_CC_OFFSET_SAVE)) {
        tr_error("Error sending CC offset save");
        return false;
    }

    tr_info("CC offset saved");

    if (!enterCalibrationMode(false)) {
        return false;
    }

    return true;
}

bool BQ35100::performBoardOffset(void) {
    if (!enterCalibrationMode(true)) {
        return false;
    }

    ThisThread::sleep_for(1s);

    while (true) {
        if (!sendCntl(CNTL_BOARD_OFFSET)) {
            tr_error("Error sending board offset");
            return false;
        }

        // wait for BCA == 1
        if (!waitforStatus(BQ3500_BCA_BIT_MASK, BQ3500_BCA_BIT_MASK)) {
            tr_error("Status: BCA != 1, will try again");
            continue;

        } else {
            tr_debug("Status OK");
            break;
        }
    }

    tr_info("Performing board offset");
    ThisThread::sleep_for(500ms);

    if (!waitforStatus(0, BQ3500_BCA_BIT_MASK, 1s)) { // wait for BCA == 0
        tr_error("Status: BCA != 0");
        return false;
    }

    tr_info("Board offset OK");

    if (!enterCalibrationMode(false)) {
        return false;
    }

    return true;
}

bool BQ35100::calibrateTemperature(int16_t temp) {
    char data[1];
    int16_t avg_temp;
    bool external;
    int32_t offset;

    // Get the Operation Config A
    if (!readExtendedData(0x41B1, data, sizeof(data))) {
        return false;
    }

    // Determine which temperature source is selected
    external = (data[0] & 0b10000000);

    tr_debug("Calibrating %s temperature", external ? "external" : "internal");

    ThisThread::sleep_for(1s);

    // Get avg raw temperature
    if (!getRawCalibrationData(CAL_TEMPERATURE, &avg_temp)) {
        return false;
    }

    offset = temp - avg_temp;

    tr_info("Temperature calibration difference: %li", offset);

    if (offset < 128 || offset > 127) {
        tr_error("Invalid temperature offset");
        return false;
    }

    data[0] = (int8_t)offset;

    // Save offset
    ThisThread::sleep_for(1s);

    if (!writeExtendedData(external ? 0x400E : 0x400D, data, sizeof(data))) {
        return false;
    }

    return true;
}

bool BQ35100::calibrateCurrent(int16_t current) {
    char data[4];
    int16_t cc_offset;
    int16_t board_offset;
    int16_t avg_current;

    tr_info("Performing current calibration");

    // Get CC offset
    if (!readExtendedData(0x4008, data, 2)) {
        return false;
    }

    cc_offset = (data[1] << 8) | data[0];

    // Get board offset
    if (!readExtendedData(0x400C, data, 2)) {
        return false;
    }

    board_offset = (data[1] << 8) | data[0];

    ThisThread::sleep_for(1s);

    // Get avg raw current
    if (!getRawCalibrationData(CAL_CURRENT, &avg_current)) {
        return false;
    }

    float cc_gain = current / (avg_current - (cc_offset + board_offset) / 16);

    if (cc_gain < 2.00E-02 || cc_gain > 10.00E+00) {
        tr_error("Invalid CC gain result");
        return false;
    }

    float cc_delta = cc_gain * 1193046;

    if (cc_delta < 2.98262E+04 || cc_delta > 5.677445E+06) {
        tr_error("Invalid CC delta result");
        return false;
    }

    // Save CC gain
    floatToDF(cc_gain, data);
    ThisThread::sleep_for(1s);

    if (!writeExtendedData(0x4000, data, sizeof(data))) {
        return false;
    }

    // Save CC delta
    floatToDF(cc_delta, data);
    ThisThread::sleep_for(40ms);

    if (!writeExtendedData(0x4004, data, sizeof(data))) {
        return false;
    }

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

bool BQ35100::getData(bq35100_cmd_t cmd, char *buffer, size_t len) {
    char d[1];
    d[0] = (char)cmd;

    if (!write(d, 1, false)) {
        return false;
    }

    return read(buffer, len);
}

bool BQ35100::sendCntl(bq35100_cntl_t cntl) {
    char data[2];
    data[0] = cntl & UCHAR_MAX;
    data[1] = cntl >> 8;

    return sendData(CMD_CONTROL, data, sizeof(data));
}

bool BQ35100::getCntl(bq35100_cntl_t cntl, uint16_t *answer) {
    bool success = false;
    char data[2];
    data[0] = cntl & UCHAR_MAX;
    data[1] = cntl >> 8;

    if (!sendData(CMD_CONTROL, data, sizeof(data))) {
        return false;
    }

    success = getData(CMD_MAC_DATA, data, sizeof(data));

    if (success && answer) {
        *answer = (((uint16_t) data[1]) << 8) + data[0];
        tr_debug("Answer: %04X", *answer);
    }

    return success;
}

bool BQ35100::writeExtendedData(uint16_t address, const char *data, size_t len) {
    uint16_t answer;
    char d[32 + 3]; // Max data len + header

    bool success = false;
    bq35100_security_t prev_security_mode = _security_mode;

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
    ThisThread::sleep_for(1s);

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

    if (!getStatus(&answer)) {
        goto END;
    }

    if (answer & BQ3500_FLASHF_BIT_MASK) {
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

bool BQ35100::readExtendedData(uint16_t address, char *buffer, size_t len) {
    size_t length_read;
    char data[32 + 2 + 2]; // 32 bytes of data, 2 bytes of address,

    bool success = false;
    bq35100_security_t prev_security_mode = _security_mode;

    if (_security_mode == SECURITY_UNKNOWN) {
        tr_error("Security mode unknown");
        return false;
    }

    if (address < 0x4000 || address > 0x43FF || !buffer) {
        tr_error("Invalid input data");
        return false;
    }

    if (_security_mode == SECURITY_SEALED && !setSecurityMode(SECURITY_UNSEALED)) {
        return false;
    }

    tr_debug("Preparing to read %u byte(s) from address 0x%04X", len, address);

    data[0] = address & UCHAR_MAX;
    data[1] = address >> 8;

    ThisThread::sleep_for(1s);

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

    memcpy(buffer, data + 2, length_read);
    success = true;
    tr_debug("Success data read(%u): %s", length_read, tr_array(reinterpret_cast<uint8_t *>(buffer), length_read));

END:

    if (prev_security_mode != _security_mode) { // in case we changed the mode
        success = setSecurityMode(prev_security_mode);
    }

    return success;
}

bool BQ35100::waitforStatus(uint16_t expected, uint16_t mask, milliseconds wait) {
    uint16_t answer;

    for (auto i = 0; i < MBED_CONF_BQ35100_RETRY; i++) {
        if (!getStatus(&answer)) {
            return false;
        }

        if ((answer & mask) == expected) {
            tr_info("Status match");
            return true;

        } else {
            tr_error("Status not yet in requested state read: %04X expected: %04X", answer, expected);
            ThisThread::sleep_for(wait);
        }
    }

    return false;
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

    ThisThread::sleep_for(2ms); // wait after each write

    return true;
}

bool BQ35100::read(char *buffer, size_t len) {
    int ack = -1;

    _i2c->lock();
    ack = _i2c->read(_address, buffer, len);
    _i2c->unlock();

    if (ack != 0) {
        return false;
    }

    tr_debug("Read data(%u): %s", len, tr_array(reinterpret_cast<uint8_t *>(buffer), len));

    return true;
}

bool BQ35100::enterCalibrationMode(bool enable) {
    ThisThread::sleep_for(1s);

    if (!sendCntl(enable ? CNTL_ENTER_CAL : CNTL_EXIT_CAL)) {
        tr_error("Error entering calibration mode");
        return false;
    }

    if (!waitforStatus(enable ? BQ3500_CAL_MODE_BIT_MASK : 0, BQ3500_CAL_MODE_BIT_MASK, 1s)) {
        tr_error("Calibration error/timeout");
        return false;
    }

    tr_info("Calibration mode %s", enable ? "enabled" : "disabled");

    return true;
}

bool BQ35100::getRawCalibrationData(bq35100_calibration_t address, int16_t *result) {
    char data[2];
    uint8_t adc_counter_prev = 0;
    uint8_t counter = 0;
    int32_t avg = 0;

    if (!enterCalibrationMode(true)) {
        return false;
    }

    while (true) {
        ThisThread::sleep_for(200ms);

        if (!getData(CMD_CAL_COUNT, data, 1)) {
            tr_error("Could not get cal count");
            return false;
        }

        if (adc_counter_prev == data[0]) {
            continue;
        }

        adc_counter_prev = data[0];

        if (!getData((bq35100_cmd_t)address, data, sizeof(data))) {
            tr_error("Could not get data");
            return false;
        }

        avg += ((data[1]) << 8) + data[0];

        tr_debug("Calibration avg: %i", ((data[0]) << 8) + data[1]);
        counter++;

        if (counter == 4) {
            break;
        }
    }

    avg /= 4;

    tr_debug("Calibration avg: %li", avg);

    if (result) {
        *result = (int16_t)avg;
    }

    if (!enterCalibrationMode(false)) {
        return false;
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

void BQ35100::floatToDF(float val, char *result) {
    int32_t exp = 0;
    float mod_val = val;
    float tmp_val = 0;
    char data[4];

    if (val < 0.0) {
        mod_val *= -1;
    }

    tmp_val = mod_val;

    tmp_val *= (1 + pow(2, -25));

    if (tmp_val < 0.5) {
        while (tmp_val < 0.5) {
            tmp_val *= 2;
            exp--;
        }

    } else if (tmp_val >= 1.0) {
        while (tmp_val >= 1.0) {
            tmp_val /= 2;
            exp--;
        }
    }

    if (exp > 127) {
        exp = 127;

    } else if (exp < -128) {
        exp = 128;
    }

    tmp_val = pow(2, 8 - exp) * val - 128;

    data[2] = (uint8_t)tmp_val;
    tmp_val = pow(2, 8) * (tmp_val - data[2]);
    data[1] = (uint8_t)tmp_val;
    tmp_val = pow(2, 8) * (tmp_val - data[1]);
    data[0] = (uint8_t)tmp_val;

    if (val < 0.0) {
        data[2] |= 0x80;
    }

    data[3] = exp + 128;

    if (result) {
        memcpy(result, data, 4);
    }
}
