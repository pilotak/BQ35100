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
        return;
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
    if (!gauge.init()) {
        return;
    }

    if (!gauge.setSecurityMode(BQ35100::SECURITY_UNSEALED)) {
        return;
    }

    if (!gauge.setGaugeMode(BQ35100::ACCUMULATOR_MODE)) {
        return;
    }

    if (!gauge.startGauge()) {
        return;
    }

    if (gauge.calibrateVoltage(3600)) { // mV
        debug("Voltage calibration successful\n");

    } else {
        debug("Voltage calibration failed\n");
    }

    if (gauge.performCCOffset()) {
        debug("CC offset successful\n");

    } else {
        debug("CC offset failed\n");
    }

    if (gauge.performBoardOffset()) {
        debug("Board offset successful\n");

    } else {
        debug("Board offset failed\n");
    }

    if (gauge.calibrateCurrent(100)) { // mA
        debug("Current calibration successful\n");

    } else {
        debug("Current calibration failed\n");
    }

    if (gauge.useInternalTemp(true) && gauge.calibrateCurrent(240)) { // 0.1°C
        debug("Internal temperature calibration successful\n");

    } else {
        debug("Internal temperature calibration failed\n");
    }

    if (gauge.useInternalTemp(false) && gauge.calibrateCurrent(240)) { // 0.1°C
        debug("External temperature calibration successful\n");

    } else {
        debug("External temperature calibration failed\n");
    }

    gauge.setSecurityMode(BQ35100::SECURITY_SEALED);
}
```
