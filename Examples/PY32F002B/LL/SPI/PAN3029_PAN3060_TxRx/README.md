# Notes

- If the NRST pin is connected to the MCU GPIO pin, you need to set `#define USE_RF_RST_GPIO 1`, if not using it, you can set it to `0`.
- If adapting to other MCU models, you can enable `USE_RF_REG_CHECK` initially by setting `#define USE_RF_REG_CHECK 1`, which can check if SPI interface is working correctly.
- If `USE_RF_REG_CHECK` is enabled, a check error will be reported when clearing IRQ, which can be ignored.
