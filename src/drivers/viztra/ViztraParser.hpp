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

#pragma once

#include <cstdint>

class ViztraParser
{
public:
	static constexpr uint8_t LEGACY_FRAME_LENGTH{21};
	static constexpr uint8_t EXTENDED_FRAME_LENGTH{33};
	static constexpr uint8_t MAX_DETECTIONS_PER_PACKET{22};
	static constexpr uint8_t MAX_DETECTION_FRAME_LENGTH{252};

	enum class Result : uint8_t {
		None,
		Frame,
		Detection,
		Heartbeat,
		DetectionAck,
		AutoLockAck,
		TrackAck,
		ChecksumError,
		FormatError,
	};

	struct Frame {
		uint8_t status;
		uint8_t channel;
		uint8_t offset_x[4];
		uint8_t offset_y[4];
		uint16_t width;
		uint16_t height;
		uint8_t relative_n[4];
		uint8_t relative_e[4];
		uint8_t relative_d[4];
		bool has_relative_position;
	};

	struct DetectedTarget {
		uint8_t id;
		uint8_t type;
		uint8_t confidence;
		uint16_t x;
		uint16_t y;
		uint16_t width;
		uint16_t height;
	};

	struct Detection {
		uint8_t frame_id;
		uint8_t total_targets;
		uint8_t count;
		DetectedTarget targets[MAX_DETECTIONS_PER_PACKET];
	};

	Result parse(uint8_t byte, Frame &frame, Detection &detection)
	{
		switch (_index) {
		case 0:
			if (byte == kHeader0) {
				_buffer[_index++] = byte;
			}

			return Result::None;

		case 1:
			if (byte == kHeader1) {
				_buffer[_index++] = byte;

			} else {
				_index = (byte == kHeader0) ? 1 : 0;
				_buffer[0] = byte;
			}

			return Result::None;

		default:
			_buffer[_index++] = byte;
			break;
		}

		if (_index == 5) {
			const uint8_t command = _buffer[3];
			const uint8_t payload_length = _buffer[4];
			const bool tracking = command == kTrackingCommand
					      && (payload_length == kLegacyPayloadLength || payload_length == kExtendedPayloadLength);
			const bool detection_packet = command == kDetectionCommand && payload_length >= 3
						      && payload_length <= 3 + 11 * MAX_DETECTIONS_PER_PACKET
						      && (payload_length - 3) % 11 == 0;
			const bool heartbeat = command == kHeartbeatCommand && payload_length == 6;
			const bool detection_ack = _buffer[2] == 0x03 && command == 0x81 && payload_length == 2;
			const bool auto_lock_ack = _buffer[2] == 0x03 && command == 0x85 && payload_length == 4;
			const bool track_ack = _buffer[2] == 0x03 && command == 0x91 && payload_length == 4;

			if (!detection_ack && !auto_lock_ack && !track_ack &&
			    (_buffer[2] != kCommand0 || (!tracking && !detection_packet && !heartbeat))) {
				reset();
				return Result::FormatError;
			}

			_expected_length = static_cast<uint8_t>(payload_length + 7U);
		}

		if (_index < _expected_length) {
			return Result::None;
		}

		const uint8_t checksum_index = static_cast<uint8_t>(_expected_length - 2U);
		const uint8_t end_index = static_cast<uint8_t>(_expected_length - 1U);
		const bool valid_end = _buffer[end_index] == kEnd;
		uint8_t checksum = 0;

		for (uint8_t i = 2; i < checksum_index; i++) {
			checksum += _buffer[i];
		}

		const bool valid_checksum = checksum == _buffer[checksum_index];

		if (valid_end && valid_checksum && _buffer[2] == kCommand0 && _buffer[3] == kTrackingCommand) {
			frame.status = _buffer[5];
			frame.channel = _buffer[6];

			for (uint8_t i = 0; i < 4; i++) {
				frame.offset_x[i] = _buffer[7 + i];
				frame.offset_y[i] = _buffer[11 + i];
			}

			frame.width = read_le_u16(&_buffer[15]);
			frame.height = read_le_u16(&_buffer[17]);
			frame.has_relative_position = _buffer[4] == kExtendedPayloadLength;

			if (frame.has_relative_position) {
				for (uint8_t i = 0; i < 4; i++) {
					frame.relative_n[i] = _buffer[19 + i];
					frame.relative_e[i] = _buffer[23 + i];
					frame.relative_d[i] = _buffer[27 + i];
				}
			}
			reset();
			return Result::Frame;
		}

		if (valid_end && valid_checksum && _buffer[2] == kCommand0 && _buffer[3] == kDetectionCommand) {
			detection.frame_id = _buffer[5];
			detection.total_targets = _buffer[6];
			detection.count = _buffer[7];

			if (detection.total_targets > 64 || detection.count > detection.total_targets
			    || detection.count != (_buffer[4] - 3) / 11) {
				reset();
				return Result::FormatError;
			}

			for (uint8_t i = 0; i < detection.count; i++) {
				const uint8_t *target = &_buffer[8 + 11 * i];
				detection.targets[i] = {target[0], target[1], target[2], read_le_u16(&target[3]),
							read_le_u16(&target[5]), read_le_u16(&target[7]), read_le_u16(&target[9])};
			}

			reset();
			return Result::Detection;
		}

		if (valid_end && valid_checksum && _buffer[2] == kCommand0 && _buffer[3] == kHeartbeatCommand) {
			reset();
			return Result::Heartbeat;
		}

		if (valid_end && valid_checksum && _buffer[2] == 0x03 && _buffer[3] == 0x81) {
			_detection_ack_result = _buffer[5];
			_detection_ack_state = _buffer[6];
			reset();
			return Result::DetectionAck;
		}

		if (valid_end && valid_checksum && _buffer[2] == 0x03 && _buffer[3] == 0x85) {
			_auto_lock_ack_result = _buffer[5];
			_auto_lock_ack_mode = _buffer[6];
			reset();
			return Result::AutoLockAck;
		}

		if (valid_end && valid_checksum && _buffer[2] == 0x03 && _buffer[3] == 0x91) {
			_track_ack_result = _buffer[5];
			_track_ack_state = _buffer[6];
			reset();
			return Result::TrackAck;
		}

		reset();
		return valid_checksum ? Result::FormatError : Result::ChecksumError;
	}

	void reset()
	{
		_index = 0;
		_expected_length = EXTENDED_FRAME_LENGTH;
	}

	uint8_t track_ack_result() const { return _track_ack_result; }
	uint8_t track_ack_state() const { return _track_ack_state; }
	uint8_t detection_ack_result() const { return _detection_ack_result; }
	uint8_t detection_ack_state() const { return _detection_ack_state; }
	uint8_t auto_lock_ack_result() const { return _auto_lock_ack_result; }
	uint8_t auto_lock_ack_mode() const { return _auto_lock_ack_mode; }

private:
	static constexpr uint8_t kHeader0{0x78};
	static constexpr uint8_t kHeader1{0x07};
	static constexpr uint8_t kCommand0{0x00};
	static constexpr uint8_t kTrackingCommand{0x81};
	static constexpr uint8_t kDetectionCommand{0x82};
	static constexpr uint8_t kHeartbeatCommand{0x83};
	static constexpr uint8_t kLegacyPayloadLength{0x0e};
	static constexpr uint8_t kExtendedPayloadLength{0x1a};
	static constexpr uint8_t kEnd{0x79};

	static uint16_t read_le_u16(const uint8_t *data)
	{
		return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
	}

	uint8_t _buffer[MAX_DETECTION_FRAME_LENGTH] {};
	uint8_t _index{0};
	uint8_t _expected_length{EXTENDED_FRAME_LENGTH};
	uint8_t _track_ack_result{0xff};
	uint8_t _track_ack_state{0xff};
	uint8_t _detection_ack_result{0xff};
	uint8_t _detection_ack_state{0xff};
	uint8_t _auto_lock_ack_result{0xff};
	uint8_t _auto_lock_ack_mode{0xff};
};
