# BQ35100

[![Framework Badge mbed](https://img.shields.io/badge/framework-mbed-008fbe.svg)](https://os.mbed.com/)

Mbed library for BQ35100 primary battery fuel gauge. Originally written by [u-blox](https://os.mbed.com/teams/ublox/code/battery-gauge-bq35100) but rewritten (no backwards compatibility) with added calibration functions and other important stuff. Compatible with OS 6, where the original one fails.

## Basic setting

```cpp
#include "mbed.h"
#include "BQ35100.h"

BQ35100 gauge(I2C_SDA, I2C_SCL, D7);

int main() {
    if (!gauge.init()) {
        return 0;
    }

    gauge.setGaugeMode(BQ35100::ACCUMULATOR_MODE);
    gauge.useInternalTemp(true);
    gauge.setDesignCapacity(3800);
    gauge.setBatteryAlert(0); // as we are not using ALERT pin
    gauge.setSecurityMode(BQ35100::SECURITY_SEALED);
}
```

## Calibration

```cpp
#include "mbed.h"
#include "BQ35100.h"

BQ35100 gauge(I2C_SDA, I2C_SCL, D7);

int main() {
    // next four step are mandatory for calibration
    if (gauge.init(&i2c)) {
        debug("Init OK\n");

    } else {
        debug("Could not init the gauge\n");
        return 0;
    }

    if (gauge.setSecurityMode(BQ35100::SECURITY_UNSEALED)) {
        debug("Device unsealed\n");

    } else {
        debug("Unseal failed\n");
        return 0;
    }

    if (gauge.setGaugeMode(BQ35100::ACCUMULATOR_MODE)) {
        debug("Gauge mode set\n");

    } else {
        debug("Set gauge mode failed\n");
        return 0;
    }

    if (gauge.startGauge()) {
        debug("Gauge started\n");

    } else {
        debug("Could not start the gauge\n");
        return 0;
    }

    if (gauge.calibrateVoltage(3600)) { // mV
        debug("Voltage calibration successful\n");

    } else {
        debug("Voltage calibration failed\n");
        return 0;
    }

    if (gauge.performCCOffset()) {
        debug("CC offset successful\n");

    } else {
        debug("CC offset failed\n");
        return 0;
    }

    if (gauge.performBoardOffset()) {
        debug("Board offset successful\n");

    } else {
        debug("Board offset failed\n");
        return 0;
    }

    if (gauge.calibrateCurrent(100)) { // mA
        debug("Current calibration successful\n");

    } else {
        debug("Current calibration failed\n");
        return 0;
    }

    if (gauge.useInternalTemp(true) && gauge.calibrateTemperature(2962)) { // 23.05°C
        debug("Internal temperature calibration successful\n");

    } else {
        debug("Internal temperature calibration failed\n");
        return 0;
    }

    if (gauge.useInternalTemp(false) && gauge.calibrateTemperature(2962)) { // 23.05°C
        debug("External temperature calibration successful\n");

    } else {
        debug("External temperature calibration failed\n");
        return 0;
    }

    gauge.setSecurityMode(BQ35100::SECURITY_SEALED);
}
```
