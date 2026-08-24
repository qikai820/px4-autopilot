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
	static constexpr uint8_t FRAME_LENGTH{21};

	enum class Result : uint8_t {
		None,
		Frame,
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
	};

	Result parse(uint8_t byte, Frame &frame)
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
			if ((_buffer[2] != kCommand0) || (_buffer[3] != kCommand1) || (_buffer[4] != kPayloadLength)) {
				reset();
				return Result::FormatError;
			}
		}

		if (_index < FRAME_LENGTH) {
			return Result::None;
		}

		const bool valid_end = _buffer[20] == kEnd;
		uint8_t checksum = 0;

		for (uint8_t i = 2; i <= 18; i++) {
			checksum += _buffer[i];
		}

		const bool valid_checksum = checksum == _buffer[19];

		if (valid_end && valid_checksum) {
			frame.status = _buffer[5];
			frame.channel = _buffer[6];

			for (uint8_t i = 0; i < 4; i++) {
				frame.offset_x[i] = _buffer[7 + i];
				frame.offset_y[i] = _buffer[11 + i];
			}

			frame.width = read_le_u16(&_buffer[15]);
			frame.height = read_le_u16(&_buffer[17]);
			reset();
			return Result::Frame;
		}

		reset();
		return valid_checksum ? Result::FormatError : Result::ChecksumError;
	}

	void reset()
	{
		_index = 0;
	}

private:
	static constexpr uint8_t kHeader0{0x78};
	static constexpr uint8_t kHeader1{0x07};
	static constexpr uint8_t kCommand0{0x00};
	static constexpr uint8_t kCommand1{0x81};
	static constexpr uint8_t kPayloadLength{0x0e};
	static constexpr uint8_t kEnd{0x79};

	static uint16_t read_le_u16(const uint8_t *data)
	{
		return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
	}

	uint8_t _buffer[FRAME_LENGTH] {};
	uint8_t _index{0};
};
