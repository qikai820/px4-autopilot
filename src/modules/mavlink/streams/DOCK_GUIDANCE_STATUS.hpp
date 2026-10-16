/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef DOCK_GUIDANCE_STATUS_HPP
#define DOCK_GUIDANCE_STATUS_HPP

#include <uORB/topics/dock_guidance_status.h>

class MavlinkStreamDockGuidanceStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamDockGuidanceStatus(mavlink); }
	static constexpr const char *get_name_static() { return "DOCK_GUIDANCE_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DOCK_GUIDANCE_STATUS; }
	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }
	unsigned get_size() override
	{
		return _status_sub.advertised() ? MAVLINK_MSG_ID_DOCK_GUIDANCE_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamDockGuidanceStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}
	uORB::Subscription _status_sub{ORB_ID(dock_guidance_status)};

	bool send() override
	{
		dock_guidance_status_s status{};
		if (_mavlink->get_free_tx_buf() < get_size() || !_status_sub.update(&status)) {
			return false;
		}

		mavlink_dock_guidance_status_t msg{};
		msg.time_boot_ms = static_cast<uint32_t>(status.timestamp / 1000);
		msg.relative_distance_m = status.relative_distance_m;
		msg.flags = status.flags;
		msg.stage = status.stage;
		memcpy(msg.entry_reason, status.entry_reason, sizeof(msg.entry_reason));
		mavlink_msg_dock_guidance_status_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // DOCK_GUIDANCE_STATUS_HPP
