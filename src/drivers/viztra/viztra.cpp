/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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

#include "ViztraParser.hpp"

#include <cinttypes>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/viztra_observation.h>
#include <uORB/topics/viztra_status.h>

using namespace time_literals;

class Viztra final : public px4::ScheduledWorkItem
{
public:
	explicit Viztra(const char *port) :
		ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
	{
		strncpy(_port, port, sizeof(_port) - 1);
	}

	~Viztra() override
	{
		stop();
		perf_free(_read_perf);
		perf_free(_comms_errors);
	}

	int init()
	{
		const param_t timeout_param = param_find("VIZTRA_TOUT");

		if (timeout_param != PARAM_INVALID) {
			int32_t timeout_ms = 500;

			if (param_get(timeout_param, &timeout_ms) == PX4_OK) {
				_timeout_us = static_cast<hrt_abstime>(math::constrain(timeout_ms, int32_t{50}, int32_t{5000})) * 1000;
			}
		}

		if (open_serial_port() != PX4_OK) {
			return PX4_ERROR;
		}

		ScheduleOnInterval(10_ms);
		return PX4_OK;
	}

	void print_info()
	{
		PX4_INFO("port: %s", _port);
		PX4_INFO("frames: %" PRIu32 ", observations: %" PRIu32, _received_frames, _valid_observations);
		PX4_INFO("checksum errors: %" PRIu32 ", format errors: %" PRIu32, _checksum_errors, _format_errors);
		perf_print_counter(_read_perf);
		perf_print_counter(_comms_errors);
	}

private:
	int open_serial_port()
	{
		if (_fd >= 0) {
			return PX4_OK;
		}

		_fd = ::open(_port, O_RDONLY | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) {
			PX4_ERR("open %s failed (%d)", _port, errno);
			return PX4_ERROR;
		}

		termios uart_config {};

		if (tcgetattr(_fd, &uart_config) != 0) {
			PX4_ERR("tcgetattr failed (%d)", errno);
			close_serial_port();
			return PX4_ERROR;
		}

		cfsetispeed(&uart_config, B115200);
		cfsetospeed(&uart_config, B115200);
		uart_config.c_cflag |= CLOCAL | CREAD;
		uart_config.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
		uart_config.c_cflag |= CS8;
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~(OPOST | ONLCR);
		uart_config.c_cc[VMIN] = 0;
		uart_config.c_cc[VTIME] = 0;

		if (tcsetattr(_fd, TCSANOW, &uart_config) != 0) {
			PX4_ERR("tcsetattr failed (%d)", errno);
			close_serial_port();
			return PX4_ERROR;
		}

		return PX4_OK;
	}

	void close_serial_port()
	{
		if (_fd >= 0) {
			::close(_fd);
			_fd = -1;
		}
	}

	void stop()
	{
		ScheduleClear();
		close_serial_port();
	}

	static float read_le_float(const uint8_t data[4])
	{
		float value;
		memcpy(&value, data, sizeof(value));
		return value;
	}

	static float read_le_int32(const uint8_t data[4])
	{
		const uint32_t value = static_cast<uint32_t>(data[0]) |
				       (static_cast<uint32_t>(data[1]) << 8) |
				       (static_cast<uint32_t>(data[2]) << 16) |
				       (static_cast<uint32_t>(data[3]) << 24);
		return static_cast<float>(static_cast<int32_t>(value));
	}

	void handle_frame(const ViztraParser::Frame &frame, hrt_abstime timestamp)
	{
		_received_frames++;
		_last_frame_timestamp = timestamp;
		_last_protocol_status = frame.status;

		const bool offsets_are_angles = (frame.status & 0x04U) != 0U;
		const bool board_reports_valid = (frame.status & 0x01U) != 0U && (frame.status & 0x02U) == 0U;
		const float offset_x = offsets_are_angles ? read_le_float(frame.offset_x) : read_le_int32(frame.offset_x);
		const float offset_y = offsets_are_angles ? read_le_float(frame.offset_y) : read_le_int32(frame.offset_y);
		const bool observation_valid = board_reports_valid && std::isfinite(offset_x) && std::isfinite(offset_y);

		if (board_reports_valid && !observation_valid) {
			_format_errors++;
		}

		viztra_observation_s observation {};
		observation.timestamp = timestamp;
		observation.timestamp_sample = timestamp;
		observation.offset_x = offset_x;
		observation.offset_y = offset_y;
		observation.target_width_px = frame.width;
		observation.target_height_px = frame.height;
		observation.video_channel = frame.channel;
		observation.protocol_status = frame.status;
		observation.valid = observation_valid;
		observation.offsets_are_angles = offsets_are_angles;
		_observation_pub.publish(observation);

		if (observation_valid) {
			_valid_observations++;
			_last_valid_timestamp = timestamp;
		}

		publish_status(timestamp, true);
	}

	void publish_status(hrt_abstime timestamp, bool force)
	{
		if (!force && (timestamp - _last_status_timestamp < 100_ms)) {
			return;
		}

		_last_status_timestamp = timestamp;
		viztra_status_s status {};
		status.timestamp = timestamp;
		status.last_frame_timestamp = _last_frame_timestamp;
		status.last_valid_timestamp = _last_valid_timestamp;
		status.received_frames = _received_frames;
		status.valid_observations = _valid_observations;
		status.checksum_errors = _checksum_errors;
		status.format_errors = _format_errors;
		status.last_protocol_status = _last_protocol_status;
		status.connected = _last_frame_timestamp != 0 && timestamp - _last_frame_timestamp <= _timeout_us;
		status.observation_valid = _last_valid_timestamp != 0 && timestamp - _last_valid_timestamp <= _timeout_us;
		_status_pub.publish(status);
	}

	void Run() override
	{
		if (_fd < 0 && open_serial_port() != PX4_OK) {
			return;
		}

		perf_begin(_read_perf);
		uint8_t buffer[64];
		int bytes_read = 0;

		do {
			bytes_read = ::read(_fd, buffer, sizeof(buffer));

			if (bytes_read > 0) {
				for (int i = 0; i < bytes_read; i++) {
					ViztraParser::Frame frame {};
					const ViztraParser::Result result = _parser.parse(buffer[i], frame);

					if (result == ViztraParser::Result::Frame) {
						handle_frame(frame, hrt_absolute_time());

					} else if (result == ViztraParser::Result::ChecksumError) {
						_checksum_errors++;

					} else if (result == ViztraParser::Result::FormatError) {
						_format_errors++;
					}
				}
			}
		} while (bytes_read > 0);

		if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			perf_count(_comms_errors);
			close_serial_port();
		}

		publish_status(hrt_absolute_time(), false);
		perf_end(_read_perf);
	}

	char _port[20] {};
	int _fd{-1};
	hrt_abstime _timeout_us{500_ms};
	hrt_abstime _last_frame_timestamp{0};
	hrt_abstime _last_valid_timestamp{0};
	hrt_abstime _last_status_timestamp{0};
	uint32_t _received_frames{0};
	uint32_t _valid_observations{0};
	uint32_t _checksum_errors{0};
	uint32_t _format_errors{0};
	uint8_t _last_protocol_status{0};
	ViztraParser _parser;

	uORB::Publication<viztra_observation_s> _observation_pub{ORB_ID(viztra_observation)};
	uORB::Publication<viztra_status_s> _status_pub{ORB_ID(viztra_status)};

	perf_counter_t _read_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME ": comms error")};
};

namespace viztra
{
Viztra *g_dev{nullptr};

int start(const char *port)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	if (port == nullptr || port[0] == '\0') {
		PX4_ERR("serial device required");
		return PX4_ERROR;
	}

	g_dev = new Viztra(port);

	if (g_dev == nullptr || g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	delete g_dev;
	g_dev = nullptr;
	return PX4_OK;
}

int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	g_dev->print_info();
	return PX4_OK;
}

int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Read-only serial driver for Viztra visual observations. It publishes observation
and parser-health topics only; it does not send commands or control the vehicle.

### Examples

$ viztra start -d /dev/ttyS3
$ viztra status
$ viztra stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("viztra", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("sensors");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the read-only driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the driver");
	return PX4_OK;
}
} // namespace viztra

extern "C" __EXPORT int viztra_main(int argc, char *argv[])
{
	const char *device_path = nullptr;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int ch = 0;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		if (ch == 'd') {
			device_path = myoptarg;

		} else {
			return viztra::usage();
		}
	}

	if (myoptind >= argc) {
		return viztra::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		return viztra::start(device_path);
	}

	if (!strcmp(argv[myoptind], "stop")) {
		return viztra::stop();
	}

	if (!strcmp(argv[myoptind], "status")) {
		return viztra::status();
	}

	return viztra::usage();
}
