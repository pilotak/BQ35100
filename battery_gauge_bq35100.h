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

#ifndef BATTERY_GAUGE_BQ35100_H
#define BATTERY_GAUGE_BQ35100_H

/**
 * @file battery_gauge_bq35100.h
 * This file defines the API to the TI BQ35100 battery gauge chip.
 */

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

/** Device I2C address. */
#define BATTERY_GAUGE_BQ35100_ADDRESS 0x55

/** I2C clock frequency.
 * NOTE: the battery shield board on the C030 platform will not work
 * at the default I2C clock frequency.
 */
#define I2C_CLOCK_FREQUENCY 100

/** Settling time after gaugeEnable is set high */
#define GAUGE_ENABLE_SETTLING_TIME_MS 10

/** The default seal codes (step 1 in the higher word, step 2 the lower word), NOT byte reversed. */
#define SEAL_CODES_DEFAULT 0x04143672

/** The default full access codes (step 1 in the higher word, step 2 the lower word). */
#define FULL_ACCESS_CODES_DEFAULT 0xFFFFFFFF

/* ----------------------------------------------------------------
 * CLASSES
 * -------------------------------------------------------------- */

/** BQ35100 battery gauge driver. */
class BatteryGaugeBq35100 {
public:

    /** The security mode of the BQ35100 chip. */
    typedef enum {
        SECURITY_MODE_UNKNOWN = 0x00,
        SECURITY_MODE_FULL_ACCESS = 0x01, //!< Allows writes to all of memory.
        SECURITY_MODE_UNSEALED = 0x02, //!< Allows writes to all of memory apart from the security codes area.
        SECURITY_MODE_SEALED = 0x03 //!< Normal operating mode, prevents accidental writes.
    } SecurityMode;

    /** Constructor. */
    BatteryGaugeBq35100(void);
    /** Destructor. */
    ~BatteryGaugeBq35100(void);

    /** Initialise the BQ35100 chip.  Once initialised
     * the chip is put into its lowest power state.  Any API call
     * will awaken the chip from this state and then return it once
     * more to the lowest possible power state.
     * @param pI2c a pointer to the I2C instance to use.
     * @param gaugeEnable the gauge enable pin (will be set high to enable the chip).
     * @param address 7-bit I2C address of the battery gauge chip.
     * @param sealCodes the two 16 bit seal codes (step 1 in the higher word, step 2 in the
     *                  lower word, NOT byte reversed) that will unseal the device if it is sealed.
     * @return true if successful, otherwise false.
     */
    bool init(I2C * pI2c, PinName gaugeEnable = NC, uint8_t address = BATTERY_GAUGE_BQ35100_ADDRESS,
              uint32_t sealCodes = SEAL_CODES_DEFAULT);
    
    /** Switch on the battery gauge.  Battery gauging must be switched on
     * for the battery capacity and percentage readings to be valid. The
     * chip will consume more when battery gauging is switched on.
     * @param nonVolatile if set to true then the chip will add the
     *                    accumulated capacity values to those taken
     *                    previously in non-volatile memory when
     *                    disableGauge() is called.
     * @return true if successful, otherwise false.
     */
    bool enableGauge(bool nonVolatile = false);

    /** Switch off the battery gauge.  If gauging to non-volatile
     * memory was switched on, the accumulated capacity values
     * will be stored in non-volatile memory.  Please see the warning
     * in section 5.1.1 of the TI BQ35100 technical reference manual
     * concerning how frequently this should be done.
     * @return true if successful, otherwise false.
     */
    bool disableGauge(void);

    /** Check whether battery gauging is enabled or not.
     * @return true if battery gauging is enabled, otherwise false.
     */
    bool isGaugeEnabled(void);

    /** Set the designed capacity of the cell.
     * @param capacityMAh the capacity.
     * @return true if successful, otherwise false.
     */
    bool setDesignCapacity(uint32_t capacityMAh);

    /** Get the designed capacity of the cell.
     * @param pCapacityMAh a place to put the capacity.
     * @return true if successful, otherwise false.
     */
    bool getDesignCapacity(uint32_t *pCapacityMAh);

    /** Read the temperature of the BQ35100 chip.
     * @param pTemperatureC place to put the temperature reading.
     * @return true if successful, otherwise false.
     */
    bool getTemperature(int32_t *pTemperatureC);

    /** Read the voltage of the battery.
     * @param pVoltageMV place to put the voltage reading.
     * @return true if successful, otherwise false.
     */
    bool getVoltage(int32_t *pVoltageMV);

    /** Read the current flowing from the battery.
     * @param pCurrentMA place to put the current reading.
     * @return true if successful, otherwise false.
     */
    bool getCurrent(int32_t *pCurrentMA);

    /** Read the battery capacity used in uAh (NOT mAh).
     * @param pCapacityUAh place to put the capacity reading.
     * @return true if successful, otherwise false.
     */
    bool getUsedCapacity(uint32_t *pCapacityUAh);

    /** Get the remaining battery capacity in uAh (NOT mAh).
    * NOTE: this relies on the Design Capacity of the battery
    * having been set correctly.
    * @param pCapacityUAh place to put the capacity reading.
    * @return true if successful, otherwise false.
    */
    bool getRemainingCapacity(uint32_t *pCapacityUAh);

    /** Get the remaining battery capacity in percent.
     * NOTE: this relies on the Design Capacity of the battery
     * having been set correctly.
     * @param pBatteryPercentage place to put the reading.
     * @return true if successful, otherwise false.
     */
    bool getRemainingPercentage(int32_t *pBatteryPercentage);

    /** Indicate that a new battery has been inserted.
     * @param capacityMAh the capacity of the battery.
     * @return true if successful, otherwise false.
     */
    bool newBattery(uint32_t capacityMAh);

    /** An advanced function to read configuration data from the BQ35100 chip memory.
     * Please refer to the TI BQ35100 technical reference manual for details of the
     * address space.This function will unseal the device (using the seal codes
     * passed into init()) in order to perform the read from data flash and will
     * restore the previous security state afterwards.
     * @param address the address of the data within the class.
     * @param pData a place to put the read data.
     * @param length the size of the place to put the data block.
     * @return true if successful, otherwise false.
     */
    bool advancedGetConfig(int32_t address, char * pData, int32_t length);

    /** An advanced function to write configuration data to the BQ35100 chip memory.
     * Please refer to the TI BQ35100 technical reference manual for details of the
     * address space.  This function will unseal the device (using the seal codes
     * passed into init()) in order to perform the write to data flash and will
     * restore the previous security state afterwards.  However, if the write
     * operation requires full access (e.g. to change the seal codes) then
     * the security mode of the device must be changed (through a call to
     * advancedGetSecurityMode()) first.  If this function is used to change the seal
     * or full access codes for the chip then init() should be called once more to
     * update the codes used by this driver.
     * @param address the address to write to.
     * @param pData a pointer to the data to be written.
     * @param length the size of the data to be written.
     * @return true if successful, otherwise false.
     */
    bool advancedSetConfig(int32_t address, const char * pData, int32_t length);

    /** Send a control word (see section 11.1 of the BQ35100 technical reference manual).
     * @param controlWord the control word to send.
     * @param pDataReturned a place to put the word of data that could be returned,
     *        depending on which control word is used (may be NULL).
     * @return true if successful, otherwise false.
     */
    bool advancedSendControlWord(uint16_t controlWord, uint16_t *pDataReturned);
    
    /** Read two bytes starting at a given address on the chip.
     * See sections 11.3 to 11.18 of the BQ35100 technical reference manual for the list
     * of addresses.
     * NOTE: this is not a read from data flash, for that you need the advancedGetConfig()
     * method.
     * @param address the start address to read from.  For instance, for temperature this is 0x06.
     * @param pDataReturned a place to put the word of data returned.
     * @return true if successful, otherwise false.
     */
    bool advancedGet(uint8_t address, uint16_t *pDataReturned);

    /** Advanced function to get the security mode of the chip.
     * @return the security mode.
     */
    SecurityMode advancedGetSecurityMode(void);

    /** Advanced function to set the security mode of the chip.
     * SECURITY_MODE_UNSEALED mode allows writes to the chip and read access to
     * certain proteced areas while SECURITY_MODE_FULL_ACCESS, in addition,
     * allows the security codes to be updated.  All of the functions in this
     * class are able to work with a SEALED chip, provided the correct codes
     * are provided to the init() function.
     * NOTE: it is only possible to move to SECURITY_MODE_FULL_ACCESS from
     * SECURITY_MODE_UNSEALED.
     * @return true if successful, otherwise false.
     */
    bool advancedSetSecurityMode(SecurityMode securityMode);
    
    /** Advanced function to perform a hard reset of the chip, reinitialising RAM
     * data to defaults from ROM.
     * Note: the security mode of the chip is unaffected.
     * @return true if successful, otherwise false.
     */
    bool advancedReset(void);
    
protected:
    /** Pointer to the I2C interface. */
    I2C * gpI2c;
    /** The address of the device. */
    uint8_t gAddress;
    /** The gauge enable pin. */
    DigitalOut *pGaugeEnable;
    /** The seal codes for the device (step 1 in the higher word, step 2 the lower word), NOT byte reversed. . */
    uint32_t gSealCodes;
    /** The full access codes for the device (step 1 in the higher word, step 2 the lower word), NOT byte reversed. . */
    uint32_t gFullAccessCodes;
    /** Flag to indicate device is ready. */
    bool gReady;
    
    /** Read two bytes starting at a given address.
     * Note: gpI2c should be locked before this is called.
     * @param registerAddress the register address to start reading from.
     * @param pBytes place to put the two bytes.
     * @return true if successful, otherwise false.
     */
    bool getTwoBytes(uint8_t registerAddress, uint16_t *pBytes);
    
    /** Compute the checksum of a block of memory in the chip.
     * @param pData a pointer to the data block.
     * @parm length the length over which to compute the checksum.
     * @return the checksum value.
     */
    uint8_t computeChecksum(const char * pData, int32_t length);

    /** Read data of a given length and class ID.
     * Note: gpI2c should be locked before this is called.
     * @param address the address of the data within the class.
     * @param pData a place to put the read data.
     * @param length the size of the place to put the data block.
     * @return true if successful, otherwise false.
     */
    bool readExtendedData(int32_t address, char * pData, int32_t length);
    
    /** Write an extended data block.
     * Note: gpI2c should be locked before this is called.
     * @param address the address to write to.
     * @param pData a pointer to the data to be written.
     * @param length the size of the data to be written.
     * @return true if successful, otherwise false.
     */
    bool writeExtendedData(int32_t address, const char * pData, int32_t length);

    /** Get the security mode of the chip.
     * Note: gpI2c should be locked before this is called.
     * @return the security mode.
     */
    SecurityMode getSecurityMode(void);

    /** Set the security mode of the chip.
     * Note: gpI2c should be locked before this is called.
     * @param securityMode the security mode to set.
     * @return true if successful, otherwise false.
     */
    bool setSecurityMode(SecurityMode securityMode);
    
    /** Make sure that the chip is awake and has taken a reading.
    * Note: the function does its own locking of gpI2C so that it isn't
    * held for the entire time we wait for ADC readings to complete.
    * @return true if successful, otherwise false.
    */
    bool makeAdcReading(void);
};

#endif

/* End Of File */
