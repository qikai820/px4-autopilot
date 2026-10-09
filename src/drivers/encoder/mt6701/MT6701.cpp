/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_encoder.h>

namespace
{
constexpr uint8_t MT6701_ADDRESS = 0x06;
constexpr uint8_t ANGLE_MSB_REG = 0x03;
constexpr uint8_t ANGLE_LSB_REG = 0x04;
constexpr uint32_t SAMPLE_INTERVAL_US = 10000; // 100 Hz
constexpr float RAD_PER_COUNT = 6.28318530717958647692f / 16384.f;
}

class MT6701 : public device::I2C, public I2CSPIDriver<MT6701>
{
public:
	explicit MT6701(const I2CSPIDriverConfig &config) : I2C(config), I2CSPIDriver(config) {}
	~MT6701() override
	{
		perf_free(_sample_perf);
		perf_free(_comms_errors);
	}

	int init() override
	{
		if (I2C::init() != PX4_OK) {
			return PX4_ERROR;
		}

		ScheduleOnInterval(SAMPLE_INTERVAL_US);
		return PX4_OK;
	}

	void RunImpl()
	{
		perf_begin(_sample_perf);
		const hrt_abstime timestamp_sample = hrt_absolute_time();
		uint16_t raw_angle = 0;

		if (read_angle(raw_angle) == PX4_OK) {
			sensor_encoder_s report{};
			report.timestamp_sample = timestamp_sample;
			report.timestamp = hrt_absolute_time();
			report.device_id = get_device_id();
			report.raw_angle = raw_angle;
			report.angle = static_cast<float>(raw_angle) * RAD_PER_COUNT;
			_sensor_encoder_pub.publish(report);
			_last_angle = report.angle;
			_last_sample = timestamp_sample;

		} else {
			perf_count(_comms_errors);
		}

		perf_end(_sample_perf);
	}

	void print_status() override
	{
		I2CSPIDriverBase::print_status();

		if (_last_sample != 0) {
			PX4_INFO("last angle: %.4f rad, age: %.3f s", (double)_last_angle,
				 (double)(hrt_absolute_time() - _last_sample) / 1e6);
		}

		perf_print_counter(_sample_perf);
		perf_print_counter(_comms_errors);
	}

	void custom_method(const BusCLIArguments &) override
	{
		// Run on the driver's work queue so this read cannot overlap RunImpl().
		I2CSPIDriverBase::print_status();
		uint16_t raw_angle = 0;
		uint8_t msb = 0;
		uint8_t lsb = 0;

		if (read_angle(raw_angle, &msb, &lsb) == PX4_OK) {
			PX4_INFO("registers: 0x03=0x%02x, 0x04=0x%02x", (unsigned)msb, (unsigned)lsb);
			PX4_INFO("angle: raw=%u, %.4f rad, %.2f deg", (unsigned)raw_angle,
				 (double)(raw_angle * RAD_PER_COUNT), (double)(raw_angle * 360.f / 16384.f));

		} else {
			perf_count(_comms_errors);
			PX4_ERR("failed to read angle registers");
		}

		if (_last_sample != 0) {
			PX4_INFO("last published sample: %.3f s ago", (double)(hrt_absolute_time() - _last_sample) / 1e6);

		} else {
			PX4_INFO("no sample published yet");
		}

		perf_print_counter(_comms_errors);
	}

	static void print_usage()
	{
		PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
MagnTek MT6701 I2C 14-bit absolute angle encoder. Publishes sensor_encoder at 100 Hz.

The sensor uses 7-bit I2C address 0x06 by default. A programmed device can use
0x46; select it with -a. The driver reads register 0x03 before 0x04 as required
by the data sheet. It does not program the sensor EEPROM.

### Example
$ mt6701 start -X -b 1
$ mt6701 info
$ listener sensor_encoder
)DESCR_STR");
		PRINT_MODULE_USAGE_NAME("mt6701", "driver");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
		PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(MT6701_ADDRESS);
		PRINT_MODULE_USAGE_COMMAND_DESCR("info", "Read and print angle registers for debugging");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	}

private:
	int probe() override
	{
		uint16_t raw_angle = 0;
		return read_angle(raw_angle);
	}

	int read_angle(uint16_t &raw_angle, uint8_t *raw_msb = nullptr, uint8_t *raw_lsb = nullptr)
	{
		uint8_t msb = 0;
		uint8_t lsb = 0;

		// MT6701 requires two single-byte reads, with 0x03 read first.
		if (transfer(&ANGLE_MSB_REG, 1, &msb, 1) != PX4_OK) {
			return PX4_ERROR;
		}

		if (transfer(&ANGLE_LSB_REG, 1, &lsb, 1) != PX4_OK) {
			return PX4_ERROR;
		}

		raw_angle = (static_cast<uint16_t>(msb) << 6) | (lsb >> 2);

		if (raw_msb != nullptr) {
			*raw_msb = msb;
		}

		if (raw_lsb != nullptr) {
			*raw_lsb = lsb;
		}

		return PX4_OK;
	}

	uORB::PublicationMulti<sensor_encoder_s> _sensor_encoder_pub{ORB_ID(sensor_encoder)};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME ": com_err")};
	hrt_abstime _last_sample{0};
	float _last_angle{0.f};
};

extern "C" __EXPORT int mt6701_main(int argc, char *argv[])
{
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = MT6701_ADDRESS;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		MT6701::print_usage();
		return PX4_ERROR;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ENC_DEVTYPE_MT6701);

	if (!strcmp(verb, "start")) {
		return MT6701::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return MT6701::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return MT6701::module_status(iterator);
	}

	if (!strcmp(verb, "info")) {
		return MT6701::module_custom_method(cli, iterator);
	}

	MT6701::print_usage();
	return PX4_ERROR;
}
