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

#define BATTERY_GAUGE_BQ35100_ADDRESS 0x55 << 1

/** Settling time after gaugeEnable is set high */
#define GAUGE_ENABLE_SETTLING_TIME_MS 10ms

/** The default seal codes (step 1 in the higher word, step 2 the lower word), NOT byte reversed. */
#define SEAL_CODES_DEFAULT 0x04143672

/** How long to wait for a security mode change to succeed. */
#define SET_SECURITY_MODE_RETRY 5 // seconds

/** How long to wait for accumulated capacity data to be written
 * to data flash when gauging is disabled */
#define GAUGE_COMPLETE_WAIT 10000 // ms

class BQ35100 {
  public:
    typedef enum {
        SECURITY_MODE_UNKNOWN = 0x00,
        SECURITY_MODE_FULL_ACCESS = 0x01, // Allows writes to all of memory.
        SECURITY_MODE_UNSEALED = 0x02, // Allows writes to all of memory apart from the security codes area.
        SECURITY_MODE_SEALED = 0x03 // Normal operating mode, prevents accidental writes.
    } security_mode_t;

    /**
     * @brief Construct
     *
     * @param address I2C address of the battery gauge chip.
     */
    BQ35100(int8_t address = BATTERY_GAUGE_BQ35100_ADDRESS);

    /**
     * @brief Construct
     *
     * @param sda I2C SDA pin
     * @param scl I2C SCL pin
     * @param frequency frequency of I2C bus
     * @param address I2C address of the battery gauge chip
     */
    BQ35100(PinName sda, PinName scl, int8_t address = BATTERY_GAUGE_BQ35100_ADDRESS, uint32_t frequency = 400000);

    /**
     * @brief Destructor
     *
     */
    ~BQ35100(void);

    /**
     * @brief Initialise the BQ35100 chip.  Once initialised
     * the chip is put into its lowest power state.  Any API call
     * will awaken the chip from this state and then return it once
     * more to the lowest possible power state.
     *
     * @param i2c_obj pass I2C object if you didn't specify pins in constructor
     * @param gaugeEnable the gauge enable pin (will be set high to enable the chip)
     * @param sealCodes the two 16 bit seal codes (step 1 in the higher word, step 2 in the
     *                  lower word, NOT byte reversed) that will unseal the device if it is sealed.
     * @return true if successful, otherwise false.
     */
    bool init(I2C *i2c_obj = nullptr, PinName gaugeEnable = NC, uint32_t sealCodes = SEAL_CODES_DEFAULT);

    /**
     * @brief Switch on the battery gauge.  Battery gauging must be switched on
     * for the battery capacity and percentage readings to be valid. The
     * chip will consume more when battery gauging is switched on.
     *
     * @param nonVolatile if set to true then the chip will add the
     *                    accumulated capacity values to those taken
     *                    previously in non-volatile memory when
     *                    disableGauge() is called.
     * @return true if successful, otherwise false.
     */
    bool enableGauge(bool nonVolatile = false);

    /**
     * @brief Switch off the battery gauge.  If gauging to non-volatile
     * memory was switched on, the accumulated capacity values
     * will be stored in non-volatile memory.  Please see the warning
     * in section 5.1.1 of the TI BQ35100 technical reference manual
     * concerning how frequently this should be done.
     *
     * @return true if successful, otherwise false.
     */
    bool disableGauge(void);

    /**
     * @brief Check whether battery gauging is enabled or not.
     *
     * @return true if battery gauging is enabled, otherwise false.
     */
    bool isGaugeEnabled(void);

    /**
     * @brief Set the designed capacity of the cell.
     *
     * @param capacityMAh the capacity.
     * @return true if successful, otherwise false.
     */
    bool setDesignCapacity(uint32_t capacityMAh);

    /**
     * @brief Get the designed capacity of the cell.
     *
     * @param pCapacityMAh a place to put the capacity.
     * @return true if successful, otherwise false.
     */
    bool getDesignCapacity(uint32_t *pCapacityMAh);

    /**
     * @brief Read the temperature of the BQ35100 chip.
     *
     * @param pTemperatureC place to put the temperature reading.
     * @return true if successful, otherwise false.
     */
    bool getTemperature(int32_t *pTemperatureC);

    /**
     * @brief Read the voltage of the battery.
     *
     * @param pVoltageMV place to put the voltage reading.
     * @return true if successful, otherwise false.
     */
    bool getVoltage(int32_t *pVoltageMV);

    /**
     * @brief Read the current flowing from the battery.
     *
     * @param pCurrentMA place to put the current reading.
     * @return true if successful, otherwise false.
     */
    bool getCurrent(int32_t *pCurrentMA);

    /**
     * @brief Read the battery capacity used in uAh (NOT mAh).
     *
     * @param pCapacityUAh place to put the capacity reading.
     * @return true if successful, otherwise false.
     */
    bool getUsedCapacity(uint32_t *pCapacityUAh);

    /**
     * @brief Get the remaining battery capacity in uAh (NOT mAh).
     *
    * @note this relies on the Design Capacity of the battery
    * having been set correctly.
    * @param pCapacityUAh place to put the capacity reading.
    * @return true if successful, otherwise false.
    */
    bool getRemainingCapacity(uint32_t *pCapacityUAh);

    /**
     * @brief Get the remaining battery capacity in percent.
     *
     * @note this relies on the Design Capacity of the battery
     * having been set correctly.
     * @param pBatteryPercentage place to put the reading.
     * @return true if successful, otherwise false.
     */
    bool getRemainingPercentage(int32_t *pBatteryPercentage);

    /**
     * @brief Indicate that a new battery has been inserted.
     *
     * @param capacityMAh the capacity of the battery.
     * @return true if successful, otherwise false.
     */
    bool newBattery(uint32_t capacityMAh);

    /**
     * @brief An advanced function to read configuration data from the BQ35100 chip memory.
     * Please refer to the TI BQ35100 technical reference manual for details of the
     * address space.This function will unseal the device (using the seal codes
     * passed into init()) in order to perform the read from data flash and will
     * restore the previous security state afterwards.
     *
     * @param address the address of the data within the class.
     * @param pData a place to put the read data.
     * @param length the size of the place to put the data block.
     * @return true if successful, otherwise false.
     */
    bool advancedGetConfig(int32_t address, char *pData, int32_t length);

    /**
     * @brief An advanced function to write configuration data to the BQ35100 chip memory.
     * Please refer to the TI BQ35100 technical reference manual for details of the
     * address space.  This function will unseal the device (using the seal codes
     * passed into init()) in order to perform the write to data flash and will
     * restore the previous security state afterwards.  However, if the write
     * operation requires full access (e.g. to change the seal codes) then
     * the security mode of the device must be changed (through a call to
     * advancedGetSecurityMode()) first.  If this function is used to change the seal
     * or full access codes for the chip then init() should be called once more to
     * update the codes used by this driver.
     *
     * @param address the address to write to.
     * @param pData a pointer to the data to be written.
     * @param length the size of the data to be written.
     * @return true if successful, otherwise false.
     */
    bool advancedSetConfig(int32_t address, const char *pData, int32_t length);

    /**
     * @brief Send a control word (see section 11.1 of the BQ35100 technical reference manual).
     *
     * @param controlWord the control word to send.
     * @param pDataReturned a place to put the word of data that could be returned,
     *        depending on which control word is used (may be NULL).
     * @return true if successful, otherwise false.
     */
    bool advancedSendControlWord(uint16_t controlWord, uint16_t *pDataReturned);

    /**
     * @brief Read two bytes starting at a given address on the chip.
     * See sections 11.3 to 11.18 of the BQ35100 technical reference manual for the list
     * of addresses.
     *
     * @note this is not a read from data flash, for that you need the advancedGetConfig()
     * method.
     * @param address the start address to read from.  For instance, for temperature this is 0x06.
     * @param pDataReturned a place to put the word of data returned.
     * @return true if successful, otherwise false.
     */
    bool advancedGet(uint8_t address, uint16_t *pDataReturned);

    /** Advanced function to get the security mode of the chip.
     * @return the security mode.
     */
    security_mode_t advancedGetSecurityMode(void);

    /**
     * @brief Advanced function to set the security mode of the chip.
     * SECURITY_MODE_UNSEALED mode allows writes to the chip and read access to
     * certain proteced areas while SECURITY_MODE_FULL_ACCESS, in addition,
     * allows the security codes to be updated.  All of the functions in this
     * class are able to work with a SEALED chip, provided the correct codes
     * are provided to the init() function.
     *
     * @note it is only possible to move to SECURITY_MODE_FULL_ACCESS from
     * SECURITY_MODE_UNSEALED.
     * @return true if successful, otherwise false.
     */
    bool advancedSetSecurityMode(security_mode_t securityMode);

    /**
     * @brief Advanced function to perform a hard reset of the chip, reinitialising RAM
     *
     * data to defaults from ROM.
     * Note: the security mode of the chip is unaffected.
     * @return true if successful, otherwise false.
     */
    bool advancedReset(void);

  protected:
    I2C *_i2c;
    DigitalOut *_gaugeEnable;
    uint32_t _i2c_obj[sizeof(I2C) / sizeof(uint32_t)] = {0};
    const int8_t _address = BATTERY_GAUGE_BQ35100_ADDRESS;
    bool _ready = false;

    /** The seal codes for the device (step 1 in the higher word, step 2 the lower word), NOT byte reversed. . */
    uint32_t _sealCodes = SEAL_CODES_DEFAULT;
    /** The full access codes for the device (step 1 in the higher word, step 2 the lower word), NOT byte reversed. . */
    uint32_t _fullAccessCodes = 0;

    /**
     * @brief Read two bytes starting at a given address.
     *
     * Note: _i2c should be locked before this is called.
     * @param registerAddress the register address to start reading from.
     * @param pBytes place to put the two bytes.
     * @return true if successful, otherwise false.
     */
    bool getTwoBytes(uint8_t registerAddress, uint16_t *pBytes);

    /**
     * @brief Compute the checksum of a block of memory in the chip.
     *
     * @param pData a pointer to the data block.
     * @parm length the length over which to compute the checksum.
     * @return the checksum value.
     */

    uint8_t computeChecksum(const char *pData, int32_t length);

    /**
     * @brief Read data of a given length and class ID.
     *
     * Note: _i2c should be locked before this is called.
     * @param address the address of the data within the class.
     * @param pData a place to put the read data.
     * @param length the size of the place to put the data block.
     * @return true if successful, otherwise false.
     */
    bool readExtendedData(uint16_t address, char *pData, int32_t length);

    /**
     * @brief Write an extended data block.
     *
     * Note: _i2c should be locked before this is called.
     * @param address the address to write to.
     * @param pData a pointer to the data to be written.
     * @param length the size of the data to be written.
     * @return true if successful, otherwise false.
     */
    bool writeExtendedData(uint16_t address, const char *pData, int32_t length);

    /**
     * @brief Get the security mode of the chip.
     *
     * Note: _i2c should be locked before this is called.
     * @return the security mode.
     */
    security_mode_t getSecurityMode(void);

    /**
     * @brief Set the security mode of the chip.
     *
     * Note: _i2c should be locked before this is called.
     * @param securityMode the security mode to set.
     * @return true if successful, otherwise false.
     */
    bool setSecurityMode(security_mode_t securityMode);

    /**
     * @brief Make sure that the chip is awake and has taken a reading.
     *
    * Note: the function does its own locking of gpI2C so that it isn't
    * held for the entire time we wait for ADC readings to complete.
    * @return true if successful, otherwise false.
    */
    bool makeAdcReading(void);
};

#endif // BQ35100_H
