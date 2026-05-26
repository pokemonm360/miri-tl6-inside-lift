# 24LC32AT EEPROM test

Minimal Zephyr application for testing a 24LC32AT-compatible I2C EEPROM.

- I2C bus: `i2c1`
- Pins: `PB6` SCL, `PB7` SDA
- 7-bit I2C address: `0x50` (`1010000`)
- EEPROM size: `4096` bytes
- Page size: `32` bytes

Build:

```powershell
.\.venv\Scripts\west.exe build -p always -b nucleo_l432kc `
  miri-tl6-inside-lift\eeprm_test `
  -d build\eeprm_test_nucleo_l432kc
```

Flash:

```powershell
.\.venv\Scripts\west.exe flash -d build\eeprm_test_nucleo_l432kc
```

The test writes and verifies two 32-byte patterns at offset `0x0100`, then
restores the original data from that block.
