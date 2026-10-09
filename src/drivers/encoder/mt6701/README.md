# MT6701 I2C absolute angle encoder

The driver reads the 14-bit absolute angle from MagnTek MT6701 registers
`0x03` and `0x04` in that order, then publishes `sensor_encoder` at 100 Hz.
`raw_angle` is 0–16383 and `angle` is in radians in `[0, 2π)`. This is a
single-turn angle; the driver does not unwrap revolutions or estimate speed.

## Hardware

- Connect the MT6701 SDA and SCL pins to an enabled PX4 I2C bus and use a
  common ground. Supply the sensor and set its MODE pin for I2C operation
  according to the board and sensor reference circuit. Use pull-ups suitable
  for the bus voltage. The [MagnTek data sheet](https://www.magntek.com.cn/upload/pdf/202407/MT6701_Rev.1.8.pdf)
  shows the pinout and reference circuit.
- The default 7-bit I2C address is `0x06`. A device programmed with the
  alternate address can use `0x46`; the driver only reads angle registers
  and does not program the EEPROM.

## Enable and run

Add `CONFIG_DRIVERS_ENCODER_MT6701=y` to the target board's `.px4board`
configuration, then build that target. Start it on the connected I2C bus,
for example:

```sh
mt6701 start -X -b 1
mt6701 status
mt6701 info
listener sensor_encoder
```

Use `-I` for an internal bus, and replace `1` with the actual bus number.
For a sensor whose I2C address was changed to `0x46`, add `-a 0x46` to the
`start` command. `mt6701 stop` stops the driver.

`mt6701 info` performs a one-time read on the driver's work queue and prints
the two angle-register bytes, decoded count, radians, degrees, age of the last
published sample, and I2C error counter. It is intended for manual debugging
and does not add continuous console output to the 100 Hz sampling loop.

The MT6701 has no documented fixed chip-ID register. Startup probes the
selected address by reading the angle registers, so a successful probe
confirms communication but cannot identify the chip if another device uses
that address. Failed reads are counted in `mt6701 status` and do not publish
new angle samples.
