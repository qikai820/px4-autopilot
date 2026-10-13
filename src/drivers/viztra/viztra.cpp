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
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_work_queue/WorkItemSingleShot.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/viztra_detection.h>
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
		const param_t auto_param = param_find("VIZTRA_AUTO");

		if (auto_param != PARAM_INVALID) {
			int32_t auto_value = 1;

			if (param_get(auto_param, &auto_value) == PX4_OK) {
				_auto_on_start = auto_value != 0;
			}
		}

		_auto_setup_state = _auto_on_start ? AutoSetupState::WaitingForStream : AutoSetupState::Disabled;

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
		PX4_INFO("tracking data packets: %" PRIu32 ", valid observations: %" PRIu32,
			 _received_frames, _valid_observations);
		PX4_INFO("AI detection packets: %" PRIu32, _detection_packets);
		PX4_INFO("checksum errors: %" PRIu32 ", format errors: %" PRIu32, _checksum_errors, _format_errors);
		perf_print_counter(_read_perf);
		perf_print_counter(_comms_errors);
	}

	void print_diagnostics();
	int test_receive();
	int enable_auto_tracking();
	int enable_multi_detection();
	int track_id(int requested_id);

private:
	static constexpr uint8_t RAW_HISTORY_SIZE{64};
	static constexpr hrt_abstime DETECTION_MAX_AGE{1_s};

	enum class TrackError : uint8_t { None, NoRecentDetection, NoCurrentTarget, IdNotDetected, ModeNotConfirmed, PortUnavailable, WriteFailed };

	struct TrackRequest {
		Viztra *driver;
		int requested_id;
		uint32_t ack_sequence{0};
		uint8_t selected_id{0};
		uint8_t selected_confidence{0};
		uint8_t latest_frame_id{0};
		uint8_t latest_targets{0};
		int write_errno{0};
		TrackError error{TrackError::None};
	};

	struct ControlRequest {
		Viztra *driver;
		uint32_t ack_sequence{0};
		int write_errno{0};
		bool port_unavailable{false};
	};

	enum class AckKind : uint8_t { Track, Detection, AutoLock };
	enum class AutoSetupState : uint8_t { Disabled, WaitingForStream, WaitingForDetectionAck, WaitingForAutoLockAck, Confirmed };
	enum class AutoSetupFailure : uint8_t { None, DetectionWrite, DetectionTimeout, DetectionRejected, AutoLockWrite, AutoLockTimeout, AutoLockRejected };

	struct CommandAckSnapshot {
		Viztra *driver;
		AckKind kind{AckKind::Track};
		uint32_t sequence{0};
		uint8_t result{0xff};
		uint8_t state{0xff};
	};

	struct DiagnosticSnapshot {
		Viztra *driver{nullptr};
		uint64_t received_bytes{0};
		hrt_abstime last_byte_timestamp{0};
		hrt_abstime last_frame_timestamp{0};
		hrt_abstime last_valid_timestamp{0};
		uint32_t received_frames{0};
		uint32_t valid_observations{0};
		uint32_t detection_packets{0};
		bool multi_detection_confirmed{false};
		bool auto_lock_confirmed{false};
		bool auto_on_start{false};
		AutoSetupState auto_setup_state{AutoSetupState::Disabled};
		AutoSetupFailure auto_setup_failure{AutoSetupFailure::None};
		uint32_t auto_setup_attempts{0};
		uint32_t detection_ack_count{0};
		uint32_t auto_lock_ack_count{0};
		uint8_t detection_ack_result{0xff};
		uint8_t detection_ack_state{0xff};
		uint8_t auto_lock_ack_result{0xff};
		uint8_t auto_lock_ack_mode{0xff};
		int auto_setup_write_errno{0};
		uint32_t nonempty_detection_packets{0};
		uint32_t checksum_errors{0};
		uint32_t format_errors{0};
		hrt_abstime last_detection_timestamp{0};
		viztra_observation_s last_observation{};
		viztra_observation_s last_valid_observation{};
		viztra_detection_s last_detection{};
		viztra_detection_s last_nonempty_detection{};
		uint8_t recent_raw[RAW_HISTORY_SIZE]{};
		uint8_t raw_write_index{0};
		uint8_t raw_count{0};
	};

	static void capture_on_work_queue(void *argument);
	static void send_detection_mode_on_work_queue(void *argument);
	static void send_auto_lock_on_work_queue(void *argument);
	static void send_track_on_work_queue(void *argument);
	static void capture_command_ack_on_work_queue(void *argument);
	void capture(DiagnosticSnapshot &snapshot);
	void advance_auto_setup(hrt_abstime now);
	void retry_auto_setup(hrt_abstime now, AutoSetupFailure reason);
	void update_lock_notification(hrt_abstime now);
	int enable_auto_tracking_once();
	static void print_raw_bytes(const DiagnosticSnapshot &snapshot, uint8_t count);
	static void print_observation(const char *label, const viztra_observation_s &observation, hrt_abstime now);
	static void print_detection(const viztra_detection_s &detection, hrt_abstime now);

	int open_serial_port()
	{
		if (_fd >= 0) {
			return PX4_OK;
		}

		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

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

		if (_auto_on_start) {
			_auto_setup_state = AutoSetupState::WaitingForStream;
			_multi_detection_confirmed = false;
			_auto_lock_confirmed = false;
			_last_byte_timestamp = 0;
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
		const float relative_n_m = frame.has_relative_position ? read_le_float(frame.relative_n) : NAN;
		const float relative_e_m = frame.has_relative_position ? read_le_float(frame.relative_e) : NAN;
		const float relative_d_m = frame.has_relative_position ? read_le_float(frame.relative_d) : NAN;
		const bool relative_position_valid = frame.has_relative_position && std::isfinite(relative_n_m)
					     && std::isfinite(relative_e_m) && std::isfinite(relative_d_m);
		const bool observation_valid = board_reports_valid && std::isfinite(offset_x) && std::isfinite(offset_y);

		if (board_reports_valid && !observation_valid) {
			_format_errors++;
		}

		viztra_observation_s observation {};
		observation.timestamp = timestamp;
		observation.timestamp_sample = timestamp;
		observation.offset_x = offset_x;
		observation.offset_y = offset_y;
		observation.relative_n_m = relative_n_m;
		observation.relative_e_m = relative_e_m;
		observation.relative_d_m = relative_d_m;
		observation.target_width_px = frame.width;
		observation.target_height_px = frame.height;
		observation.video_channel = frame.channel;
		observation.protocol_status = frame.status;
		observation.valid = observation_valid;
		observation.offsets_are_angles = offsets_are_angles;
		observation.relative_position_valid = relative_position_valid;
		_last_observation = observation;
		_observation_pub.publish(observation);

		if (observation_valid) {
			_valid_observations++;
			_last_valid_timestamp = timestamp;
			_last_valid_observation = observation;
		}

		publish_status(timestamp, true);
	}

	void handle_detection(const ViztraParser::Detection &detection, hrt_abstime timestamp)
	{
		if (detection.total_targets == 0 || _recent_detection_frame_id != detection.frame_id ||
		    _last_detection_timestamp == 0 || timestamp - _last_detection_timestamp > DETECTION_MAX_AGE) {
			memset(_recent_detection_ids, 0, sizeof(_recent_detection_ids));
			memset(_recent_detection_confidence, 0, sizeof(_recent_detection_confidence));
			_recent_detection_frame_id = detection.frame_id;
		}

		viztra_detection_s report{};
		report.timestamp = timestamp;
		report.frame_id = detection.frame_id;
		report.total_targets = detection.total_targets;
		report.packet_targets = detection.count;

		for (uint8_t i = 0; i < detection.count; i++) {
			const uint8_t id = detection.targets[i].id;
			_recent_detection_ids[id] = true;
			_recent_detection_confidence[id] = detection.targets[i].confidence;

			report.target_id[i] = detection.targets[i].id;
			report.target_type[i] = detection.targets[i].type;
			report.confidence[i] = detection.targets[i].confidence;
			report.x_px[i] = detection.targets[i].x;
			report.y_px[i] = detection.targets[i].y;
			report.width_px[i] = detection.targets[i].width;
			report.height_px[i] = detection.targets[i].height;
		}

		_detection_packets++;
		_last_frame_timestamp = timestamp;
		_last_detection_timestamp = timestamp;
		_last_detection = report;

		if (detection.count > 0) {
			_nonempty_detection_packets++;
			_last_nonempty_detection = report;
		}

		_detection_pub.publish(report);
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
			update_lock_notification(hrt_absolute_time());
			return;
		}

		perf_begin(_read_perf);
		uint8_t buffer[64];
		int bytes_read = 0;
		ViztraParser::Frame frame{};
		ViztraParser::Detection detection{};

		do {
			bytes_read = ::read(_fd, buffer, sizeof(buffer));

			if (bytes_read > 0) {
				_last_byte_timestamp = hrt_absolute_time();

				for (int i = 0; i < bytes_read; i++) {
					_recent_raw[_raw_write_index] = buffer[i];
					_raw_write_index = (_raw_write_index + 1) % RAW_HISTORY_SIZE;

					if (_raw_count < RAW_HISTORY_SIZE) {
						_raw_count++;
					}

					_received_bytes++;
					const ViztraParser::Result result = _parser.parse(buffer[i], frame, detection);

					if (result == ViztraParser::Result::Frame) {
						handle_frame(frame, hrt_absolute_time());

					} else if (result == ViztraParser::Result::Detection) {
						handle_detection(detection, hrt_absolute_time());

					} else if (result == ViztraParser::Result::TrackAck) {
						_track_ack_sequence++;
						_track_ack_result = _parser.track_ack_result();
						_track_ack_state = _parser.track_ack_state();

					} else if (result == ViztraParser::Result::DetectionAck) {
						_detection_ack_sequence++;
						_detection_ack_result = _parser.detection_ack_result();
						_detection_ack_state = _parser.detection_ack_state();
						_multi_detection_confirmed = _detection_ack_result == 0 && _detection_ack_state == 2;

					} else if (result == ViztraParser::Result::AutoLockAck) {
						_auto_lock_ack_sequence++;
						_auto_lock_ack_result = _parser.auto_lock_ack_result();
						_auto_lock_ack_mode = _parser.auto_lock_ack_mode();
						_auto_lock_confirmed = _auto_lock_ack_result == 0 && _auto_lock_ack_mode == 2;

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

		advance_auto_setup(hrt_absolute_time());
		update_lock_notification(hrt_absolute_time());
		publish_status(hrt_absolute_time(), false);
		perf_end(_read_perf);
	}

	char _port[20] {};
	int _fd{-1};
	hrt_abstime _timeout_us{500_ms};
	hrt_abstime _last_frame_timestamp{0};
	hrt_abstime _last_valid_timestamp{0};
	hrt_abstime _last_status_timestamp{0};
	hrt_abstime _last_byte_timestamp{0};
	hrt_abstime _last_detection_timestamp{0};
	uint64_t _received_bytes{0};
	uint8_t _recent_raw[RAW_HISTORY_SIZE]{};
	uint8_t _raw_write_index{0};
	uint8_t _raw_count{0};
	uint32_t _received_frames{0};
	uint32_t _valid_observations{0};
	uint32_t _detection_packets{0};
	uint32_t _nonempty_detection_packets{0};
	uint32_t _checksum_errors{0};
	uint32_t _format_errors{0};
	uint32_t _track_ack_sequence{0};
	uint8_t _track_ack_result{0xff};
	uint8_t _track_ack_state{0xff};
	uint32_t _detection_ack_sequence{0};
	uint8_t _detection_ack_result{0xff};
	uint8_t _detection_ack_state{0xff};
	uint32_t _auto_lock_ack_sequence{0};
	uint8_t _auto_lock_ack_result{0xff};
	uint8_t _auto_lock_ack_mode{0xff};
	uint8_t _recent_detection_frame_id{0};
	bool _recent_detection_ids[256]{};
	uint8_t _recent_detection_confidence[256]{};
	bool _multi_detection_confirmed{false};
	bool _auto_lock_confirmed{false};
	bool _lock_notified{false};
	bool _auto_on_start{true};
	AutoSetupState _auto_setup_state{AutoSetupState::Disabled};
	AutoSetupFailure _auto_setup_failure{AutoSetupFailure::None};
	hrt_abstime _auto_setup_next_attempt{0};
	hrt_abstime _auto_setup_deadline{0};
	uint32_t _auto_setup_attempts{0};
	uint32_t _auto_detection_ack_start{0};
	uint32_t _auto_lock_ack_start{0};
	int _auto_setup_write_errno{0};
	uint8_t _last_protocol_status{0};
	viztra_observation_s _last_observation{};
	viztra_observation_s _last_valid_observation{};
	viztra_detection_s _last_detection{};
	viztra_detection_s _last_nonempty_detection{};
	ViztraParser _parser;

	uORB::Publication<viztra_detection_s> _detection_pub{ORB_ID(viztra_detection)};
	uORB::Publication<viztra_observation_s> _observation_pub{ORB_ID(viztra_observation)};
	uORB::Publication<viztra_status_s> _status_pub{ORB_ID(viztra_status)};
	orb_advert_t _mavlink_log_pub{nullptr};

	perf_counter_t _read_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME ": comms error")};
};

void Viztra::update_lock_notification(hrt_abstime now)
{
	// Do not turn a single missing or invalid frame into a QGC alert. The
	// observation timeout still applies, with a 500 ms minimum for messages.
	const hrt_abstime notification_timeout = _timeout_us < 500_ms ? 500_ms : _timeout_us;
	const bool locked = _last_valid_timestamp != 0 && now - _last_valid_timestamp <= notification_timeout;

	if (locked == _lock_notified) {
		return;
	}

	_lock_notified = locked;

	if (locked) {
		mavlink_log_info(&_mavlink_log_pub, "Viztra: target locked");

	} else {
		mavlink_log_warning(&_mavlink_log_pub, "Viztra: target lost");
	}
}

void Viztra::retry_auto_setup(hrt_abstime now, AutoSetupFailure reason)
{
	_auto_setup_failure = reason;
	_auto_setup_state = AutoSetupState::WaitingForStream;
	_auto_setup_next_attempt = now + 1_s;

	if (_auto_setup_attempts == 1) {
		PX4_WARN("automatic tracking setup has not completed; retrying after board data arrives");
	}
}

void Viztra::advance_auto_setup(hrt_abstime now)
{
	if (_fd < 0 || _auto_setup_state == AutoSetupState::Disabled || _auto_setup_state == AutoSetupState::Confirmed) {
		return;
	}

	if (_auto_setup_state == AutoSetupState::WaitingForStream) {
		if (_last_byte_timestamp == 0 || now - _last_byte_timestamp > 1_s || now < _auto_setup_next_attempt) {
			return;
		}

		// Protocol v3.1 section 4.4.1: detection plus multi-target tracking.
		const uint8_t command[] {0x58, 0x07, 0x03, 0x01, 0x02, 0x02, 0x00, 0x08, 0x59};
		_auto_setup_attempts++;
		_auto_setup_write_errno = 0;
		_multi_detection_confirmed = false;
		_auto_lock_confirmed = false;
		_auto_detection_ack_start = _detection_ack_sequence;
		const ssize_t written = ::write(_fd, command, sizeof(command));

		if (written != static_cast<ssize_t>(sizeof(command))) {
			_auto_setup_write_errno = written < 0 ? errno : EIO;
			retry_auto_setup(now, AutoSetupFailure::DetectionWrite);
			return;
		}

		_auto_setup_state = AutoSetupState::WaitingForDetectionAck;
		_auto_setup_deadline = now + 750_ms;
		return;
	}

	if (_auto_setup_state == AutoSetupState::WaitingForDetectionAck) {
		if (_detection_ack_sequence != _auto_detection_ack_start) {
			if (!_multi_detection_confirmed) {
				retry_auto_setup(now, AutoSetupFailure::DetectionRejected);
				return;
			}

			// Protocol v3.1 section 4.4.5: cyclic auto-lock, highest-confidence target.
			const uint8_t command[] {0x58, 0x07, 0x03, 0x05, 0x04, 0x02, 0x00, 0x00, 0x00, 0x0e, 0x59};
			_auto_lock_ack_start = _auto_lock_ack_sequence;
			const ssize_t written = ::write(_fd, command, sizeof(command));

			if (written != static_cast<ssize_t>(sizeof(command))) {
				_auto_setup_write_errno = written < 0 ? errno : EIO;
				retry_auto_setup(now, AutoSetupFailure::AutoLockWrite);
				return;
			}

			_auto_setup_state = AutoSetupState::WaitingForAutoLockAck;
			_auto_setup_deadline = now + 750_ms;
			return;
		}

		if (now >= _auto_setup_deadline) {
			retry_auto_setup(now, AutoSetupFailure::DetectionTimeout);
		}

		return;
	}

	if (_auto_lock_ack_sequence != _auto_lock_ack_start) {
		if (_auto_lock_confirmed) {
			_auto_setup_state = AutoSetupState::Confirmed;
			_auto_setup_failure = AutoSetupFailure::None;
			PX4_INFO("cyclic auto-lock enabled after %" PRIu32 " setup attempt(s)", _auto_setup_attempts);

		} else {
			retry_auto_setup(now, AutoSetupFailure::AutoLockRejected);
		}

	} else if (now >= _auto_setup_deadline) {
		retry_auto_setup(now, AutoSetupFailure::AutoLockTimeout);
	}
}

void Viztra::capture_on_work_queue(void *argument)
{
	auto &snapshot = *static_cast<DiagnosticSnapshot *>(argument);
	const Viztra &driver = *snapshot.driver;
	snapshot.received_bytes = driver._received_bytes;
	snapshot.last_byte_timestamp = driver._last_byte_timestamp;
	snapshot.last_frame_timestamp = driver._last_frame_timestamp;
	snapshot.last_valid_timestamp = driver._last_valid_timestamp;
	snapshot.received_frames = driver._received_frames;
	snapshot.valid_observations = driver._valid_observations;
	snapshot.detection_packets = driver._detection_packets;
	snapshot.multi_detection_confirmed = driver._multi_detection_confirmed;
	snapshot.auto_lock_confirmed = driver._auto_lock_confirmed;
	snapshot.auto_on_start = driver._auto_on_start;
	snapshot.auto_setup_state = driver._auto_setup_state;
	snapshot.auto_setup_failure = driver._auto_setup_failure;
	snapshot.auto_setup_attempts = driver._auto_setup_attempts;
	snapshot.detection_ack_count = driver._detection_ack_sequence;
	snapshot.auto_lock_ack_count = driver._auto_lock_ack_sequence;
	snapshot.detection_ack_result = driver._detection_ack_result;
	snapshot.detection_ack_state = driver._detection_ack_state;
	snapshot.auto_lock_ack_result = driver._auto_lock_ack_result;
	snapshot.auto_lock_ack_mode = driver._auto_lock_ack_mode;
	snapshot.auto_setup_write_errno = driver._auto_setup_write_errno;
	snapshot.nonempty_detection_packets = driver._nonempty_detection_packets;
	snapshot.checksum_errors = driver._checksum_errors;
	snapshot.format_errors = driver._format_errors;
	snapshot.last_detection_timestamp = driver._last_detection_timestamp;
	snapshot.last_observation = driver._last_observation;
	snapshot.last_valid_observation = driver._last_valid_observation;
	snapshot.last_detection = driver._last_detection;
	snapshot.last_nonempty_detection = driver._last_nonempty_detection;
	memcpy(snapshot.recent_raw, driver._recent_raw, sizeof(snapshot.recent_raw));
	snapshot.raw_write_index = driver._raw_write_index;
	snapshot.raw_count = driver._raw_count;
}

void Viztra::capture(DiagnosticSnapshot &snapshot)
{
	snapshot.driver = this;
	// Read driver-owned data on its work queue, then print from the NSH thread.
	px4::WorkItemSingleShot reader(*this, capture_on_work_queue, &snapshot);
	reader.ScheduleNow();
	reader.wait();
}

void Viztra::send_detection_mode_on_work_queue(void *argument)
{
	auto &request = *static_cast<ControlRequest *>(argument);
	Viztra &driver = *request.driver;

	if (driver._fd < 0) {
		request.port_unavailable = true;
		return;
	}

	// Protocol v3.1 section 4.4.1: enable AI detection with multi-target tracking.
	const uint8_t command[] {0x58, 0x07, 0x03, 0x01, 0x02, 0x02, 0x00, 0x08, 0x59};
	request.ack_sequence = driver._detection_ack_sequence;
	const ssize_t written = ::write(driver._fd, command, sizeof(command));

	if (written != static_cast<ssize_t>(sizeof(command))) {
		request.write_errno = written < 0 ? errno : EIO;
		return;
	}

	// Old IDs (including detection-only ID 0) do not identify targets in the new mode.
	memset(driver._recent_detection_ids, 0, sizeof(driver._recent_detection_ids));
	memset(driver._recent_detection_confidence, 0, sizeof(driver._recent_detection_confidence));
	driver._last_detection_timestamp = 0;
}

int Viztra::enable_multi_detection()
{
	ControlRequest request{this};
	px4::WorkItemSingleShot sender(*this, send_detection_mode_on_work_queue, &request);
	sender.ScheduleNow();
	sender.wait();

	if (request.port_unavailable) {
		PX4_ERR("serial port is unavailable");
		return PX4_ERROR;
	}

	if (request.write_errno != 0) {
		PX4_ERR("failed to enable multi-target detection (%d)", request.write_errno);
		return PX4_ERROR;
	}

	const hrt_abstime deadline = hrt_absolute_time() + 750_ms;
	CommandAckSnapshot ack{this, AckKind::Detection};

	while (hrt_absolute_time() < deadline) {
		px4::WorkItemSingleShot reader(*this, capture_command_ack_on_work_queue, &ack);
		reader.ScheduleNow();
		reader.wait();

		if (ack.sequence != request.ack_sequence) {
			if (ack.result == 0 && ack.state == 2) {
				PX4_INFO("AI detection + multi-target tracking enabled");
				return PX4_OK;
			}

			PX4_ERR("board rejected multi-target detection (result=%u, state=%u)",
				(unsigned)ack.result, (unsigned)ack.state);
			return PX4_ERROR;
		}

		px4_usleep(20'000);
	}

	PX4_ERR("no multi-target detection acknowledgement");
	return PX4_ERROR;
}

void Viztra::send_auto_lock_on_work_queue(void *argument)
{
	auto &request = *static_cast<ControlRequest *>(argument);
	Viztra &driver = *request.driver;

	if (driver._fd < 0) {
		request.port_unavailable = true;
		return;
	}

	// Protocol v3.1 section 4.4.5: cyclic auto-lock, highest-confidence policy.
	const uint8_t command[] {0x58, 0x07, 0x03, 0x05, 0x04, 0x02, 0x00, 0x00, 0x00, 0x0e, 0x59};
	request.ack_sequence = driver._auto_lock_ack_sequence;
	const ssize_t written = ::write(driver._fd, command, sizeof(command));

	if (written != static_cast<ssize_t>(sizeof(command))) {
		request.write_errno = written < 0 ? errno : EIO;
	}
}

int Viztra::enable_auto_tracking()
{
	px4::WorkItemSingleShot pauser(*this, [](void *argument) {
		static_cast<Viztra *>(argument)->_auto_setup_state = AutoSetupState::Disabled;
	}, this);
	pauser.ScheduleNow();
	pauser.wait();

	const int result = enable_auto_tracking_once();
	struct FinishRequest { Viztra *driver; bool success; } request{this, result == PX4_OK};
	px4::WorkItemSingleShot finisher(*this, [](void *argument) {
		auto &finish = *static_cast<FinishRequest *>(argument);
		Viztra &driver = *finish.driver;
		driver._auto_setup_state = finish.success ? AutoSetupState::Confirmed :
					   (driver._auto_on_start ? AutoSetupState::WaitingForStream : AutoSetupState::Disabled);
		driver._auto_setup_next_attempt = hrt_absolute_time() + 1_s;

		if (finish.success) {
			driver._auto_setup_failure = AutoSetupFailure::None;
		}
	}, &request);
	finisher.ScheduleNow();
	finisher.wait();
	return result;
}

int Viztra::enable_auto_tracking_once()
{
	if (enable_multi_detection() != PX4_OK) {
		return PX4_ERROR;
	}

	ControlRequest request{this};
	px4::WorkItemSingleShot sender(*this, send_auto_lock_on_work_queue, &request);
	sender.ScheduleNow();
	sender.wait();

	if (request.port_unavailable) {
		PX4_ERR("serial port is unavailable");
		return PX4_ERROR;
	}

	if (request.write_errno != 0) {
		PX4_ERR("failed to enable cyclic auto-lock (%d)", request.write_errno);
		return PX4_ERROR;
	}

	const hrt_abstime deadline = hrt_absolute_time() + 750_ms;
	CommandAckSnapshot ack{this, AckKind::AutoLock};

	while (hrt_absolute_time() < deadline) {
		px4::WorkItemSingleShot reader(*this, capture_command_ack_on_work_queue, &ack);
		reader.ScheduleNow();
		reader.wait();

		if (ack.sequence != request.ack_sequence) {
			if (ack.result == 0 && ack.state == 2) {
				PX4_INFO("cyclic auto-lock enabled (highest-confidence target, reacquire after loss)");
				return PX4_OK;
			}

			PX4_ERR("board rejected cyclic auto-lock (result=%u, mode=%u)",
				(unsigned)ack.result, (unsigned)ack.state);
			return PX4_ERROR;
		}

		px4_usleep(20'000);
	}

	PX4_ERR("no cyclic auto-lock acknowledgement");
	return PX4_ERROR;
}

void Viztra::send_track_on_work_queue(void *argument)
{
	auto &request = *static_cast<TrackRequest *>(argument);
	Viztra &driver = *request.driver;
	const hrt_abstime now = hrt_absolute_time();

	if (driver._last_detection_timestamp == 0 || now - driver._last_detection_timestamp > DETECTION_MAX_AGE) {
		request.error = TrackError::NoRecentDetection;
		return;
	}

	request.latest_frame_id = driver._last_detection.frame_id;
	request.latest_targets = driver._last_detection.total_targets;

	if (request.requested_id < 0) {
		// Omitted ID: choose the highest-confidence nonzero target in the current
		// image frame. Use ID 0 only if it is the sole candidate and mode is confirmed.
		bool found = false;

		for (unsigned id = 1; id < 256; id++) {
			if (driver._recent_detection_ids[id] &&
			    (!found || driver._recent_detection_confidence[id] > request.selected_confidence)) {
				request.selected_id = static_cast<uint8_t>(id);
				request.selected_confidence = driver._recent_detection_confidence[id];
				found = true;
			}
		}

		if (!found && driver._recent_detection_ids[0] && driver._multi_detection_confirmed) {
			request.selected_id = 0;
			request.selected_confidence = driver._recent_detection_confidence[0];
			found = true;
		}

		if (!found) {
			request.error = driver._recent_detection_ids[0] ? TrackError::ModeNotConfirmed : TrackError::NoCurrentTarget;
			return;
		}

	} else {
		request.selected_id = static_cast<uint8_t>(request.requested_id);

		if (!driver._recent_detection_ids[request.selected_id]) {
			request.error = TrackError::IdNotDetected;
			return;
		}

		request.selected_confidence = driver._recent_detection_confidence[request.selected_id];
	}

	if (request.selected_id == 0 && !driver._multi_detection_confirmed) {
		request.error = TrackError::ModeNotConfirmed;
		return;
	}

	if (driver._fd < 0) {
		request.error = TrackError::PortUnavailable;
		return;
	}

	// Protocol v3.1 section 4.4.6: ID tracking (03 11), ten payload bytes.
	// The document says IDs are 1..64, but a board reported IDs 84 and 0.
	// Carry the observed ID and check its acknowledgement.
	uint8_t command[] {0x58, 0x07, 0x03, 0x11, 0x0a, 0x02, request.selected_id,
			   0, 0, 0, 0, 0, 0, 0, 0, 0, 0x59};
	uint8_t checksum = 0;

	for (unsigned i = 2; i < sizeof(command) - 2; i++) {
		checksum += command[i];
	}

	command[sizeof(command) - 2] = checksum;
	request.ack_sequence = driver._track_ack_sequence;

	const ssize_t written = ::write(driver._fd, command, sizeof(command));

	if (written != static_cast<ssize_t>(sizeof(command))) {
		request.write_errno = written < 0 ? errno : EIO;
		request.error = TrackError::WriteFailed;
	}
}

void Viztra::capture_command_ack_on_work_queue(void *argument)
{
	auto &snapshot = *static_cast<CommandAckSnapshot *>(argument);

	if (snapshot.kind == AckKind::Detection) {
		snapshot.sequence = snapshot.driver->_detection_ack_sequence;
		snapshot.result = snapshot.driver->_detection_ack_result;
		snapshot.state = snapshot.driver->_detection_ack_state;

	} else if (snapshot.kind == AckKind::AutoLock) {
		snapshot.sequence = snapshot.driver->_auto_lock_ack_sequence;
		snapshot.result = snapshot.driver->_auto_lock_ack_result;
		snapshot.state = snapshot.driver->_auto_lock_ack_mode;

	} else {
		snapshot.sequence = snapshot.driver->_track_ack_sequence;
		snapshot.result = snapshot.driver->_track_ack_result;
		snapshot.state = snapshot.driver->_track_ack_state;
	}
}

int Viztra::track_id(int requested_id)
{
	TrackRequest request{this, requested_id};
	px4::WorkItemSingleShot sender(*this, send_track_on_work_queue, &request);
	sender.ScheduleNow();
	sender.wait();

	switch (request.error) {
	case TrackError::NoRecentDetection:
		PX4_ERR("no AI detection in the last 1 second");
		return PX4_ERROR;

	case TrackError::NoCurrentTarget:
		PX4_ERR("no target in current image frame %u", (unsigned)request.latest_frame_id);
		return PX4_ERROR;

	case TrackError::IdNotDetected:
		PX4_ERR("target ID %d absent from current image frame %u (targets=%u)", requested_id,
			(unsigned)request.latest_frame_id, (unsigned)request.latest_targets);
		return PX4_ERROR;

	case TrackError::ModeNotConfirmed:
		PX4_ERR("target ID 0 requires confirmed multi-target mode; run 'viztra detect' first");
		return PX4_ERROR;

	case TrackError::PortUnavailable:
		PX4_ERR("serial port is unavailable");
		return PX4_ERROR;

	case TrackError::WriteFailed:
		PX4_ERR("failed to write ID tracking command (%d)", request.write_errno);
		return PX4_ERROR;

	case TrackError::None:
		break;
	}

	const hrt_abstime deadline = hrt_absolute_time() + 750_ms;
	CommandAckSnapshot ack{this};

	while (hrt_absolute_time() < deadline) {
		px4::WorkItemSingleShot reader(*this, capture_command_ack_on_work_queue, &ack);
		reader.ScheduleNow();
		reader.wait();

		if (ack.sequence != request.ack_sequence) {
			if (ack.result == 0 && ack.state == 2) {
				PX4_INFO("ID tracking accepted for target %u%s (confidence=%u%%, image frame=%u)",
					 (unsigned)request.selected_id, requested_id < 0 ? " [auto]" : "",
					 (unsigned)request.selected_confidence, (unsigned)request.latest_frame_id);
				return PX4_OK;
			}

			PX4_ERR("board rejected ID %u tracking (result=%u, state=%u)",
				(unsigned)request.selected_id, (unsigned)ack.result, (unsigned)ack.state);
			return PX4_ERROR;
		}

		px4_usleep(20'000);
	}

	PX4_ERR("no ID tracking acknowledgement for target %u", (unsigned)request.selected_id);
	return PX4_ERROR;
}

void Viztra::print_raw_bytes(const DiagnosticSnapshot &snapshot, uint8_t count)
{
	uint8_t index = (snapshot.raw_write_index + RAW_HISTORY_SIZE - count) % RAW_HISTORY_SIZE;
	char line[16 * 3 + 1]{};

	for (uint8_t i = 0; i < count; i++) {
		const uint8_t column = i % 16;
		snprintf(&line[column * 3], sizeof(line) - column * 3, "%02x ", (unsigned)snapshot.recent_raw[index]);
		index = (index + 1) % RAW_HISTORY_SIZE;

		if (column == 15 || i == count - 1) {
			PX4_INFO("%s", line);
			line[0] = '\0';
		}
	}
}

void Viztra::print_observation(const char *label, const viztra_observation_s &observation, hrt_abstime now)
{
	PX4_INFO("%s: valid=%s, tracking=%s, channel=%u, status=0x%02x, age=%.3f s", label,
		 observation.valid ? "true" : "false", (observation.protocol_status & 0x02U) ? "stopped" : "running",
		 (unsigned)observation.video_channel,
		 (unsigned)observation.protocol_status, (double)(now - observation.timestamp) / 1e6);
	PX4_INFO("target offset: x=%.2f, y=%.2f %s; box: %u x %u px",
		 (double)observation.offset_x, (double)observation.offset_y,
		 observation.offsets_are_angles ? "deg" : "px",
		 (unsigned)observation.target_width_px, (unsigned)observation.target_height_px);

	if (observation.relative_position_valid) {
		PX4_INFO("relative NED: N=%.2f, E=%.2f, D=%.2f m",
			 (double)observation.relative_n_m, (double)observation.relative_e_m, (double)observation.relative_d_m);

	} else {
		PX4_INFO("relative NED: unavailable");
	}
}

void Viztra::print_detection(const viztra_detection_s &detection, hrt_abstime now)
{
	PX4_INFO("AI detection: image frame ID=%u, total targets=%u, targets in packet=%u, age=%.3f s",
		 (unsigned)detection.frame_id, (unsigned)detection.total_targets, (unsigned)detection.packet_targets,
		 (double)(now - detection.timestamp) / 1e6);

	for (uint8_t i = 0; i < detection.packet_targets; i++) {
		PX4_INFO("target[%u]: target ID=%u, type=%u, confidence=%u%%, box=(%u,%u) %ux%u px",
			 (unsigned)i, (unsigned)detection.target_id[i], (unsigned)detection.target_type[i],
			 (unsigned)detection.confidence[i], (unsigned)detection.x_px[i], (unsigned)detection.y_px[i],
			 (unsigned)detection.width_px[i], (unsigned)detection.height_px[i]);
	}
}

void Viztra::print_diagnostics()
{
	DiagnosticSnapshot *snapshot = new DiagnosticSnapshot{};

	if (!snapshot) {
		PX4_ERR("cannot allocate diagnostic snapshot");
		return;
	}

	capture(*snapshot);
	const hrt_abstime now = hrt_absolute_time();
	PX4_INFO("port: %s, baud: 115200 8N1", _port);
	PX4_INFO("rx bytes: %" PRIu64, snapshot->received_bytes);

	if (snapshot->last_byte_timestamp != 0) {
		PX4_INFO("last rx byte: %.3f s ago", (double)(now - snapshot->last_byte_timestamp) / 1e6);

	} else {
		PX4_INFO("last rx byte: never");
	}

	PX4_INFO("tracking data packets: %" PRIu32 ", valid observations: %" PRIu32,
		 snapshot->received_frames, snapshot->valid_observations);
	PX4_INFO("AI detection packets: %" PRIu32, snapshot->detection_packets);
	PX4_INFO("multi-target mode confirmed this driver session: %s",
		 snapshot->multi_detection_confirmed ? "yes" : "no");
	PX4_INFO("cyclic auto-lock confirmed this driver session: %s",
		 snapshot->auto_lock_confirmed ? "yes" : "no");
	const char *setup_state = "disabled";

	switch (snapshot->auto_setup_state) {
	case AutoSetupState::Disabled: break;
	case AutoSetupState::WaitingForStream: setup_state = "waiting for board data or retry"; break;
	case AutoSetupState::WaitingForDetectionAck: setup_state = "waiting for detection ACK"; break;
	case AutoSetupState::WaitingForAutoLockAck: setup_state = "waiting for auto-lock ACK"; break;
	case AutoSetupState::Confirmed: setup_state = "confirmed"; break;
	}

	const char *setup_failure = "none";

	switch (snapshot->auto_setup_failure) {
	case AutoSetupFailure::None: break;
	case AutoSetupFailure::DetectionWrite: setup_failure = "detection write"; break;
	case AutoSetupFailure::DetectionTimeout: setup_failure = "detection ACK timeout"; break;
	case AutoSetupFailure::DetectionRejected: setup_failure = "detection rejected"; break;
	case AutoSetupFailure::AutoLockWrite: setup_failure = "auto-lock write"; break;
	case AutoSetupFailure::AutoLockTimeout: setup_failure = "auto-lock ACK timeout"; break;
	case AutoSetupFailure::AutoLockRejected: setup_failure = "auto-lock rejected"; break;
	}

	PX4_INFO("automatic setup: parameter=%s, state=%s, attempts=%" PRIu32 ", last failure=%s, write errno=%d",
		 snapshot->auto_on_start ? "on" : "off", setup_state, snapshot->auto_setup_attempts,
		 setup_failure, snapshot->auto_setup_write_errno);
	PX4_INFO("detection ACKs: %" PRIu32 ", last result=%u, state=%u; auto-lock ACKs: %" PRIu32
		 ", last result=%u, mode=%u", snapshot->detection_ack_count,
		 (unsigned)snapshot->detection_ack_result, (unsigned)snapshot->detection_ack_state,
		 snapshot->auto_lock_ack_count, (unsigned)snapshot->auto_lock_ack_result,
		 (unsigned)snapshot->auto_lock_ack_mode);
	PX4_INFO("checksum errors: %" PRIu32 ", format errors: %" PRIu32, snapshot->checksum_errors, snapshot->format_errors);

	if (snapshot->last_frame_timestamp != 0) {
		PX4_INFO("last frame: %.3f s ago", (double)(now - snapshot->last_frame_timestamp) / 1e6);

	} else {
		PX4_INFO("last frame: never");
	}

	if (snapshot->last_valid_timestamp != 0) {
		PX4_INFO("last valid target: %.3f s ago", (double)(now - snapshot->last_valid_timestamp) / 1e6);

	} else {
		PX4_INFO("last valid target: never");
	}

	if (snapshot->last_detection_timestamp != 0) {
		PX4_INFO("last AI detection packet: %.3f s ago", (double)(now - snapshot->last_detection_timestamp) / 1e6);

	} else {
		PX4_INFO("last AI detection packet: never");
	}

	delete snapshot;
}

int Viztra::test_receive()
{
	DiagnosticSnapshot *snapshot = new DiagnosticSnapshot{};

	if (!snapshot) {
		PX4_ERR("cannot allocate diagnostic snapshot");
		return PX4_ERROR;
	}

	capture(*snapshot);
	const uint64_t before_bytes = snapshot->received_bytes;
	const uint32_t before_frames = snapshot->received_frames;
	const uint32_t before_valid = snapshot->valid_observations;
	const uint32_t before_detections = snapshot->detection_packets;
	const uint32_t before_nonempty_detections = snapshot->nonempty_detection_packets;
	const uint32_t before_checksum_errors = snapshot->checksum_errors;
	const uint32_t before_format_errors = snapshot->format_errors;
	PX4_INFO("capturing serial data for 3 seconds on %s", _port);
	px4_usleep(3'000'000);
	capture(*snapshot);

	const uint64_t new_bytes = snapshot->received_bytes - before_bytes;
	const uint32_t new_frames = snapshot->received_frames - before_frames;
	const uint32_t new_valid = snapshot->valid_observations - before_valid;
	const uint32_t new_detections = snapshot->detection_packets - before_detections;
	const uint32_t new_nonempty_detections = snapshot->nonempty_detection_packets - before_nonempty_detections;
	PX4_INFO("rx bytes: %" PRIu64 ", tracking data packets: %" PRIu32 ", valid observations: %" PRIu32,
		 new_bytes, new_frames, new_valid);
	PX4_INFO("AI detection packets: %" PRIu32, new_detections);
	PX4_INFO("checksum errors: %" PRIu32 ", format errors: %" PRIu32,
		 snapshot->checksum_errors - before_checksum_errors, snapshot->format_errors - before_format_errors);

	if (new_bytes == 0) {
		PX4_WARN("no serial bytes received");
		delete snapshot;
		return PX4_ERROR;
	}

	const uint8_t count = new_bytes < snapshot->raw_count ? static_cast<uint8_t>(new_bytes) : snapshot->raw_count;
	PX4_INFO("last %u received bytes (hex):", (unsigned)count);
	print_raw_bytes(*snapshot, count);

	if (new_frames == 0 && new_detections == 0) {
		PX4_WARN("serial bytes received, but no Viztra tracking or AI detection frame decoded");
		delete snapshot;
		return PX4_ERROR;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (new_detections > 0) {
		print_detection(snapshot->last_detection, now);

		if (snapshot->last_detection.total_targets == 0) {
			PX4_WARN("current image frame has no target; an older target ID cannot be locked now");
		}

		if (snapshot->last_detection.packet_targets > 0 && snapshot->last_detection.target_id[0] == 0) {
			if (snapshot->multi_detection_confirmed) {
				PX4_WARN("board reports target ID 0 outside documented range; 'viztra track 0' will check its acknowledgement");

			} else {
				PX4_WARN("target ID 0 reported; run 'viztra detect' before trying 'viztra track 0'");
			}
		}

		PX4_INFO("use 'target ID' for viztra track, not image frame ID or packet count");

		if (new_nonempty_detections > 0 && snapshot->last_detection.packet_targets == 0) {
			PX4_INFO("last packet with a recognized target in test window (historical):");
			print_detection(snapshot->last_nonempty_detection, now);
		}
	}

	if (new_valid > 0) {
		print_observation("last locked target", snapshot->last_valid_observation, now);

		if (!snapshot->last_observation.valid) {
			PX4_INFO("latest frame: target invalid, status=0x%02x",
				 (unsigned)snapshot->last_observation.protocol_status);
		}

	} else if (new_frames > 0) {
		PX4_INFO("no valid locked target in test window");
		print_observation("latest tracking frame", snapshot->last_observation, now);
	}

	delete snapshot;
	return PX4_OK;
}

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

int info()
{
	if (g_dev == nullptr) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	g_dev->print_diagnostics();
	return PX4_OK;
}

int test()
{
	if (g_dev == nullptr) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	return g_dev->test_receive();
}

int detect()
{
	if (g_dev == nullptr) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	return g_dev->enable_multi_detection();
}

int auto_track()
{
	if (g_dev == nullptr) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	return g_dev->enable_auto_tracking();
}

int track(const char *id_text)
{
	if (g_dev == nullptr) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	if (id_text == nullptr) {
		return g_dev->track_id(-1);
	}

	char *end = nullptr;
	errno = 0;
	const long id = strtol(id_text, &end, 10);

	if (errno != 0 || end == id_text || *end != '\0' || id < 0 || id > 255) {
		PX4_ERR("invalid target ID '%s' (expected 0..255)", id_text);
		return PX4_ERROR;
	}

	return g_dev->track_id(static_cast<int>(id));
}

int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial driver for Viztra tracking and AI detection packets. It publishes tracking
observations and detections. With VIZTRA_AUTO enabled, startup configures cyclic
auto-lock after board data arrives and retries until acknowledged. The board selects
the highest-confidence detection and reacquires after loss. The auto command applies this immediately. The track command selects
a current ID manually, or the best current detection when no ID is given.

### Examples

$ viztra start -d /dev/ttyS3
$ viztra status
$ viztra info
$ viztra auto
$ viztra test
$ viztra detect
$ viztra track
$ viztra test
$ viztra track 3
$ listener viztra_detection -n 1
$ listener viztra_observation -n 1
$ viztra stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("viztra", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("sensors");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("info", "Print serial receive and decoder diagnostics");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Capture 3 seconds of serial data and print tracking and AI detections");
	PRINT_MODULE_USAGE_COMMAND_DESCR("detect", "Enable AI detection with multi-target tracking");
	PRINT_MODULE_USAGE_COMMAND_DESCR("auto", "Enable cyclic auto-lock on the board");
	PRINT_MODULE_USAGE_COMMAND_DESCR("track", "Track the best current detection or a specified ID");
	PRINT_MODULE_USAGE_ARG("<id>", "Optional current target ID (0..255; ID 0 requires viztra detect)", true);
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

	if (!strcmp(argv[myoptind], "info")) {
		return viztra::info();
	}

	if (!strcmp(argv[myoptind], "test")) {
		return viztra::test();
	}

	if (!strcmp(argv[myoptind], "detect")) {
		return myoptind + 1 == argc ? viztra::detect() : viztra::usage();
	}

	if (!strcmp(argv[myoptind], "auto")) {
		return myoptind + 1 == argc ? viztra::auto_track() : viztra::usage();
	}

	if (!strcmp(argv[myoptind], "track")) {
		if (myoptind + 1 == argc) {
			return viztra::track(nullptr);
		}

		if (myoptind + 2 == argc) {
			return viztra::track(argv[myoptind + 1]);
		}

		return viztra::usage();
	}

	return viztra::usage();
}
