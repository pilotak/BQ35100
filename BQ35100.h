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

#ifndef BQ35100_H
#define BQ35100_H

#include "mbed.h"
using namespace std::chrono;

#include "mbed-trace/mbed_trace.h"
#ifndef TRACE_GROUP
    #define TRACE_GROUP  "BATT"
#endif

#define BQ35100_I2C_ADDRESS (0x55 << 1)

/** The default seal codes (step 1 in the higher word, step 2 the lower word), NOT byte reversed */
#define BQ35100_DEFAULT_SEAL_CODES 0x04143672

#define BQ3500_GA_BIT_MASK       0b0000000000000001
#define BQ3500_G_DONE_BIT_MASK   0b0000000001000000
#define BQ3500_CCA_BIT_MASK      0b0000010000000000
#define BQ3500_BCA_BIT_MASK      0b0000100000000000
#define BQ3500_CAL_MODE_BIT_MASK 0b0001000000000000

class BQ35100 {
  public:
    typedef enum {
        SECURITY_UNKNOWN = 0x00,
        SECURITY_FULL_ACCESS = 0x01, // Allows writes to all of memory
        SECURITY_UNSEALED = 0x02, // Allows writes to all of memory apart from the security codes area
        SECURITY_SEALED = 0x03 // Normal operating mode, prevents accidental writes
    } bq35100_security_t;

    typedef enum {
        ACCUMULATOR_MODE = 0b00,
        SOH_MODE = 0b011, // for LiMnO2
        EOS_MODE = 0b10, // for LiSOCl2
        UNKNOWN_MODE = 0b11
    } bq35100_gauge_mode_t;

    typedef enum {
        CAL_CURRENT = 0X7A,
        CAL_VOLTAGE = 0X7C,
        CAL_TEMPERATURE = 0X7E
    } bq35100_calibration_t;

    /**
     * @brief Constructor
     *
     * @param seal_codes the seal codes for the device (step 1 in the higher word, step 2 the lower word), NOT byte reversed
     * @param address I2C address of the battery gauge chip
     */
    BQ35100(uint32_t seal_codes = BQ35100_DEFAULT_SEAL_CODES, uint8_t address = BQ35100_I2C_ADDRESS);

    /**
     * @brief Constructor
     *
     * @param sda I2C SDA pin
     * @param scl I2C SCL pin
     * @param seal_codes the seal codes for the device (step 1 in the higher word, step 2 the lower word), NOT byte reversed
     * @param frequency frequency of I2C bus
     * @param address I2C address of the battery gauge chip
     */
    BQ35100(PinName sda, PinName scl, uint32_t seal_codes = BQ35100_DEFAULT_SEAL_CODES,
            uint8_t address = BQ35100_I2C_ADDRESS, uint32_t frequency = 400000);

    /**
     * @brief Destructor
     *
     */
    ~BQ35100(void);

    /**
     * @brief Initialise the BQ35100 chip
     *
     * @param i2c_obj pass I2C object if you didn't specify pins in constructor
     * @param gauge_enable_pin the gauge enable pin (will be set high to enable the chip)
     * @return true if successful, otherwise false
     */
    bool init(I2C *i2c_obj = nullptr, PinName gauge_enable_pin = NC);

    /**
     * @brief Start battery gauge. Battery gauging must be switched on
     * for the battery capacity and percentage readings to be valid. The
     * chip will consume more when battery gauging is switched on.
     *
     * @return true if successful, otherwise false
     */
    bool startGauge(void);

    /**
     * @brief Wait until everything finishes up (G_DONE) and asserts GE low
     *
     * @return true if successful, otherwise false
     */
    bool disableGauge(void);

    /**
     * @brief Stop battery gauging. It doesn't handle GE pin!
     * If ACCUMULATOR_MODE is activated, the accumulated capacity values
     * will be stored in non-volatile memory. Please see the warning
     * in section 5.1.1 of the TI BQ35100 technical reference manual
     * concerning how frequently this should be done.
     *
     * @return true if successful, otherwise false
     */
    bool stopGauge(void);

    /**
     * @brief Check whether battery gauging is enabled or not
     *
     * @return true if battery gauging is enabled, otherwise false
     */
    bool isGaugeEnabled(void);

    /**
     * @brief Set the designed capacity of the cell
     *
     * @param capacity the capacity in mAh
     * @return true if successful, otherwise false
     */
    bool setDesignCapacity(uint16_t capacity);

    /**
     * @brief Get the designed capacity of the cell
     *
     * @param capacity a place to put the capacity
     * @return true if successful, otherwise false
     */
    bool getDesignCapacity(uint16_t *capacity);

    /**
     * @brief Read the temperature of the BQ35100 chip
     *
     * @param temp place to put the temperature reading
     * @return true if successful, otherwise false
     */
    bool getTemperature(int16_t *temp);

    /**
     * @brief Read the internal temperature of the BQ35100 chip
     *
     * @param temp place to put the temperature reading
     * @return true if successful, otherwise false
     */
    bool getInternalTemperature(int16_t *temp);

    /**
     * @brief Read the voltage of the battery
     *
     * @param voltage place to put the voltage reading
     * @return true if successful, otherwise false
     */
    bool getVoltage(uint16_t *voltage);

    /**
     * @brief Read the current flowing from the battery
     *
     * @param current place to put the current reading
     * @return true if successful, otherwise false
     */
    bool getCurrent(uint16_t *current);

    /**
     * @brief Read the battery capacity used in uAh (NOT mAh)
     *
     * @param capacity_used place to put the capacity reading
     * @return true if successful, otherwise false
     */
    bool getUsedCapacity(uint32_t *capacity_used);

    /**
     * @brief Get the remaining battery capacity in mAh
     *
    * @note this relies on the Design Capacity of the battery
    * having been set correctly
    * @param remaining_capacity place to put the capacity reading
    * @return true if successful, otherwise false
    */
    bool getRemainingCapacity(uint16_t *remaining_capacity);

    /**
     * @brief Get the remaining battery capacity in percent
     *
     * @note this relies on the Design Capacity of the battery
     * having been set correctly
     * @param battery_percentage place to put the reading
     * @return true if successful, otherwise false
     */
    bool getRemainingPercentage(uint8_t *battery_percentage);

    /**
     * @brief Indicate that a new battery has been inserted
     *
     * @param capacity the capacity of the battery (mAh), if zero use the previous set
     * @return true if successful, otherwise false
     */
    bool newBattery(uint16_t capacity);

    /**
     * @brief Advanced function to perform a hard reset of the chip, reinitialising RAM
     *
     * data to defaults from ROM
     * @note the security mode of the chip is unaffected
     * @return true if successful, otherwise false
     */
    bool reset(void);

    /**
     * @brief Set the Gauge Mode object
     *
     * @param gauge_mode the gauging mode
     * @return true if successful, otherwise false
     */
    bool setGaugeMode(bq35100_gauge_mode_t gauge_mode);

    /**
     * @brief Whether to use internal temperature sensor for calculations
     *
     * @param use
     * @return true if successful, otherwise false
     */
    bool useInternalTemp(bool use);

    /**
     * @brief Get the security mode of the chip
     *
     * @note _i2c should be locked before this is called
     * @return the security mode
     */
    bq35100_security_t getSecurityMode(void);

    /**
     * @brief Set the security mode of the chip
     *
     * @note _i2c should be locked before this is called
     * @param new_security the security mode to set
     * @return true if successful, otherwise false
     */
    bool setSecurityMode(bq35100_security_t new_security);

    /**
     * @brief Set measure period in lower power mode for EOS mode
     * (after GAUGE_STOP)
     *
     * @param seconds number of seconds
     * @return true if successful, otherwise false
     */
    bool setEosDataSeconds(uint8_t seconds);

    /**
     * @brief Get status of the battery
     *
     * @param status place to put the battery status reading
     * @return true if successful, otherwise false
     */
    bool getBatteryStatus(uint8_t *status);

    /**
     * @brief Set events when ALERT pin is asserted
     *
     * @param alert binary write enabled events
     * @return true if successful, otherwise false
     */
    bool setBatteryAlert(uint8_t alert);

    /**
     * @brief Get alert of the battery (to know why ALERT pin was asserted)
     *
     * @param status place to put the battery alert reading
     * @return true if successful, otherwise false
     */
    bool getBatteryAlert(uint8_t *alert);

    /**
     * @brief Set the under temperature
     *
     * @param min minimal temperature in 0.1*C
     * @return true if successful, otherwise false
     */
    bool setUnderTemperature(int16_t min);

    /**
     * @brief Get the raw calibration data
     *
     * @param cmd
     * @param address address of the register to read
     * @return true if successful, otherwise false
     */
    bool getRawCalibrationData(bq35100_calibration_t address, uint16_t *result);

    bool calibrateCurrent(uint16_t current);

    /**
     * @brief Perform CC offset (no current should be flowing)
     *
     * @return true if successful, otherwise false
     */
    bool performCCOffset(void);

    /**
     * @brief Perform Board offset (no current should be flowing)
     *
     * @return true if successful, otherwise false
     */
    bool performBoardOffset(void);


  protected:
    typedef enum {
        CMD_CONTROL = 0x00,
        CMD_ACCUMULATED_CAPACITY = 0x02,
        CMD_TEMPERATURE = 0x06,
        CMD_VOLTAGE = 0x08,
        CMD_BATTERY_STATUS = 0x0A,
        CMD_BATTERY_ALERT = 0x0B,
        CMD_CURRENT = 0x0C,
        CMD_SCALED_R = 0x16,
        CMD_MEASURED_Z = 0x22,
        CMD_INTERNAL_TEMPERATURE = 0x28,
        CMD_STATE_OF_HEALTH = 0x2E,
        // extended
        CMD_OP_CONFIG = 0x3A,
        CMD_DESIGN_CAPACITY = 0x3C,
        CMD_MAC = 0x3E,
        CMD_MAC_DATA = 0x40,
        CMD_MAC_DATA_SUM = 0x60,
        CMD_MAC_DATA_LEN = 0x61,
        CMD_MAC_DATA_CONTROL = 0x62,
        CMD_CAL_COUNT = 0x79,
        CMD_CAL_CURRENT = 0X7A,
        CMD_CAL_VOLTAGE = 0X7C,
        CMD_CAL_TEMPERATURE = 0X7E
    } bq35100_cmd_t;

    typedef enum {
        CNTL_CONTROL_STATUS = 0x0000,
        CNTL_DEVICE_TYPE    = 0x0001,
        CNTL_FW_VERSION     = 0x0002,
        CNTL_HW_VERSION     = 0x0003,
        CNTL_CHEM_DF_CHKSUM = 0x0005,
        CNTL_CHEM_ID        = 0x0006,
        CNTL_PREV_MACWRITE  = 0x0007,
        CNTL_BOARD_OFFSET   = 0x0009,
        CNTL_CC_OFFSET      = 0x000A,
        CNTL_CC_OFFSET_SAVE = 0x000B,
        CNTL_GAUGE_START    = 0x0011,
        CNTL_GAUGE_STOP     = 0x0012,
        CNTL_CAL_ENABLE     = 0x002D,
        CNTL_LT_ENABLE      = 0x002E,
        CNTL_SEAL           = 0x0020,
        CNTL_RESET          = 0x0041,
        CNTL_EXIT_CAL       = 0x0080,
        CNTL_ENTER_CAL      = 0x0081,
        CNTL_NEW_BATTERY    = 0xA613
    } bq35100_cntl_t;

    I2C *_i2c;
    DigitalOut *_gauge_enable_pin;
    uint32_t _i2c_obj[sizeof(I2C) / sizeof(uint32_t)] = {0};
    const uint32_t _seal_codes = BQ35100_DEFAULT_SEAL_CODES;
    const uint8_t _address = BQ35100_I2C_ADDRESS;
    bq35100_security_t _security_mode = SECURITY_UNKNOWN;
    bool _enabled = false;

    /**
     * @brief Compute the checksum of a block of memory in the chip
     *
     * @param data a pointer to the data block
     * @parm length the length over which to compute the checksum
     * @return the checksum value
     */

    uint8_t computeChecksum(const char *data, size_t length);

    /**
     * @brief Read data of a given length and class ID
     *
     * @note _i2c should be locked before this is called
     * @param address the data flash address to read from
     * @param response a place to put the read data
     * @param len the size of the place to put the data block
     * @return true if successful, otherwise false
     */
    bool readExtendedData(uint16_t address, char *response, size_t len);

    /**
     * @brief Write an extended data block
     *
     * @note _i2c should be locked before this is called
     * @param address the data flash address to write to
     * @param data a pointer to the data to be written
     * @param len the size of the data to be written
     * @return true if successful, otherwise false
     */
    bool writeExtendedData(uint16_t address, const char *data, size_t len);

    /**
     * @brief Enter calibration mode
     *
     * @param enable enter(1) or exit(0) calibration mode
     * @return true if successful, otherwise false
     */
    bool enterCalibrationMode(bool enable);
    bool waitforStatus(uint16_t expected, uint16_t mask, milliseconds wait = 10ms);

    bool write(const char *data, size_t len, bool stop = true);
    bool read(char *data, size_t len, bool stop = true);
    bool sendData(bq35100_cmd_t cmd, const char *data, size_t len);
    bool getData(bq35100_cmd_t cmd, char *data, size_t len);
    bool sendCntl(bq35100_cntl_t cntl);
    bool getCntl(bq35100_cntl_t cntl, uint16_t *answer);

    bool getCCBoardOffset(uint16_t cc, uint16_t board);
};

#endif // BQ35100_H
