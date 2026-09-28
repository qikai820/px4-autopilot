#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <lib/parameters/param.h>
#include <uORB/Subscription.hpp>

#include <uORB/uORBMessageFields.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

#include <matrix/matrix/math.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <errno.h>

using namespace time_literals;

class UorbUdpBridge final : public ModuleBase<UorbUdpBridge>
{
public:
	static constexpr unsigned GENERIC_TOPIC_COUNT{11};
	UorbUdpBridge() = default;
	~UorbUdpBridge() override
	{
		close_socket();
		for (unsigned i = 0; i < GENERIC_TOPIC_COUNT; ++i) {
			delete _generic_subs[i];
		}
	}

	static int task_spawn(int argc, char **argv)
	{
		_task_id = px4_task_spawn_cmd("uorb_udp_bridge",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_DEFAULT - 20,
					      PX4_STACK_ADJUSTED(4096),
					      (px4_main_t)&run_trampoline,
					      (char *const *)argv);

		if (_task_id < 0) {
			_task_id = -1;
			return -errno;
		}

		return wait_until_running();
	}

	static UorbUdpBridge *instantiate(int, char **) { return new UorbUdpBridge(); }

	static int custom_command(int argc, char *argv[])
	{
		if (argc > 0 && !strcmp(argv[0], "config")) {
			UorbUdpBridge *instance = get_instance();
			if (!instance || argc < 2) { return PX4_ERROR; }
			const int port = (argc > 2) ? atoi(argv[2]) : 9870;
			const float rate = (argc > 3) ? strtof(argv[3], nullptr) : 20.f;
			return instance->configure(argv[1], port, rate) ? PX4_OK : PX4_ERROR;
		}
		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }
		PX4_INFO("usage: uorb_udp_bridge {start|stop|status}");
		PX4_INFO("  uorb_udp_bridge config <ip> [port] [rate_hz]");
		PX4_INFO("  default: 192.168.3.109:9870, 20 Hz");
		return 0;
	}

	int print_status() override
	{
		PX4_INFO("UDP fd=%d %s:%u at %.1f Hz (requested %.1f, max %.1f), sent %lu datagrams (%lu bytes), splits %lu, drops %lu, errors %lu last_errno %d",
			 _socket, _address, _port, (double)_rate_hz, (double)_requested_rate_hz, (double)_max_rate_hz,
			 (unsigned long)_sent,
			 (unsigned long)_bytes_sent, (unsigned long)_packet_splits, (unsigned long)_oversize_drops,
			 (unsigned long)_send_errors, _last_errno);
		return 0;
	}

	bool init()
	{
		_max_rate_param = param_find("UORB_UDP_MAX_HZ");
		update_rate_limit(false);

		if (!configure(_address, _port, _requested_rate_hz)) { return false; }
		init_generic_topics();
		return true;
	}

	void run() override
	{
		if (!init()) {
			return;
		}

		hrt_abstime next_run = hrt_absolute_time();

		while (!should_exit()) {
			// Create and use the socket in this task. NuttX file descriptors
			// belong to the task group that created them.
			if (_socket < 0 && !open_socket()) {
				px4_usleep(1_s);
				continue;
			}

			parameter_update_s parameter_update;

			if (_parameter_update_sub.update(&parameter_update)) {
				update_rate_limit(true);
			}

			publish_generic_topics();

			const hrt_abstime interval_us = static_cast<hrt_abstime>(1'000'000.f / _rate_hz);
			next_run += interval_us;
			const hrt_abstime now = hrt_absolute_time();

			if (next_run > now) {
				px4_usleep(next_run - now);

			} else {
				// Do not run catch-up bursts after serialization or network stalls.
				next_run = now;
			}
		}

		close_socket();
	}

private:
	static constexpr unsigned MAX_LAYOUT_FIELDS{96};
	struct FieldLayout {
		const char *type{nullptr};
		uint16_t offset{0};
		uint8_t array_size{1};
		char name[48]{};
	};
	struct TopicLayout {
		FieldLayout fields[MAX_LAYOUT_FIELDS]{};
		uint8_t count{0};
		bool valid{false};
	};

	bool configure(const char *address, int port, float rate)
	{
		if (port < 1 || port > 65535 || rate <= 0.f || !std::isfinite(rate)) { return false; }
		char address_copy[INET_ADDRSTRLEN]{};
		strncpy(address_copy, address, sizeof(address_copy) - 1);
		struct sockaddr_in dest{};
		dest.sin_family = AF_INET;
		dest.sin_port = htons((uint16_t)port);
		if (inet_aton(address, &dest.sin_addr) == 0) { return false; }
		_dest = dest;
		strncpy(_address, address_copy, sizeof(_address) - 1);
		_port = (uint16_t)port;
		_requested_rate_hz = rate;
		_rate_hz = math::min(_requested_rate_hz, _max_rate_hz);
		return true;
	}

	void update_rate_limit(bool reschedule)
	{
		float max_rate_hz = _max_rate_hz;

		if (_max_rate_param != PARAM_INVALID && param_get(_max_rate_param, &max_rate_hz) == PX4_OK
		    && std::isfinite(max_rate_hz)) {
			_max_rate_hz = math::constrain(max_rate_hz, 1.f, 120.f);
		}

		const float limited_rate_hz = math::min(_requested_rate_hz, _max_rate_hz);

		if (reschedule && fabsf(limited_rate_hz - _rate_hz) > 0.01f) {
			_rate_hz = limited_rate_hz;
			PX4_INFO("rate updated: requested %.1f Hz, max %.1f Hz, effective %.1f Hz",
				 (double)_requested_rate_hz, (double)_max_rate_hz, (double)_rate_hz);
		}
	}

	bool open_socket()
	{
		const int sock = socket(AF_INET, SOCK_DGRAM, 0);

		if (sock < 0) {
			_last_errno = errno;
			const hrt_abstime now = hrt_absolute_time();

			if (now - _last_error_report > 1_s) {
				PX4_WARN("socket(AF_INET, SOCK_DGRAM) failed: errno=%d", _last_errno);
				_last_error_report = now;
			}

			return false;
		}

		_socket = sock;
		PX4_INFO("UDP socket opened in work queue: fd=%d -> %s:%u", _socket, _address, _port);
		return true;
	}

	void close_socket() { if (_socket >= 0) { ::close(_socket); _socket = -1; } }
	void start_packet(uint64_t timestamp)
	{
		_message_valid = true;
		const int length = snprintf(_buffer, sizeof(_buffer), "{\"time\":%.6f",
					(double)timestamp * 1e-6);

		if (length < 0 || (size_t)length >= sizeof(_buffer)) {
			_length = 0;
			_message_valid = false;

		} else {
			_length = (size_t)length;
		}
	}
	void field_uint64(const char *name, uint64_t value)
	{
		char text[192];
		const int length = snprintf(text, sizeof(text), ",\"%s/%s\":%" PRIu64, _topic, name, value);

		if (length < 0 || (size_t)length >= sizeof(text)) {
			_message_valid = false;

		} else {
			append(text, (size_t)length);
		}
	}
	void init_generic_topics()
	{
		const struct orb_metadata *const *topics = orb_get_topics();

		for (unsigned i = 0; i < GENERIC_TOPIC_COUNT; ++i) {
			for (size_t j = 0; j < orb_topics_count(); ++j) {
				if (strcmp(generic_topic_names()[i], topics[j]->o_name) == 0) {
					_generic_meta[i] = topics[j];
					_generic_subs[i] = new uORB::Subscription(topics[j]);
					init_topic_layout(i);
					break;
				}
			}

			if (_generic_meta[i] == nullptr) {
				PX4_WARN("uORB topic not found: %s", generic_topic_names()[i]);
			}
		}
	}

	void publish_generic_topics()
	{
		const hrt_abstime now = hrt_absolute_time();
		start_packet(now);
		bool any_updated = false;

		for (unsigned i = 0; i < GENERIC_TOPIC_COUNT; ++i) {
			if (_generic_meta[i] == nullptr || _generic_subs[i] == nullptr
			    || _generic_meta[i]->o_size > sizeof(_generic_data[i])) {
				continue;
			}
			if (now - _last_topic_send[i] < _topic_interval_us[i]) { continue; }
			_last_topic_send[i] = now;

			// Copy only when the subscription reports a new generation. The source
			// timestamp check filters duplicate samples after a missed publication.
			if (_generic_subs[i]->updated() && _generic_subs[i]->update(_generic_data[i])) {
				uint64_t sample_timestamp = 0;
				memcpy(&sample_timestamp, _generic_data[i], sizeof(sample_timestamp));

				if (sample_timestamp == 0 || sample_timestamp == _last_topic_timestamp[i]) {
					continue;
				}

				_last_topic_timestamp[i] = sample_timestamp;
				serialize_generic_topic(i, _generic_data[i]);
				any_updated = true;
			}
		}

		if (any_updated) {
			end();
		} else {
			_length = 0;
		}
	}

	void init_topic_layout(unsigned topic_index)
	{
		const orb_metadata &meta = *_generic_meta[topic_index];
		char format_buffer[128];
		uORB::MessageFormatReader format_reader(format_buffer, sizeof(format_buffer));

		if (!format_reader.readUntilFormat(meta.o_id)) {
			return;
		}

		int data_offset = 0;
		int field_length = 0;

		TopicLayout &layout = _topic_layout[topic_index];
		while (format_reader.readNextField(field_length) && layout.count < MAX_LAYOUT_FIELDS) {
			if (field_length <= 0 || field_length >= static_cast<int>(sizeof(format_buffer))) {
				return;
			}

			const char *c_type = orb_get_c_type(format_buffer[0]);
			const char *field_name = strchr(format_buffer, ' ');
			int array_size = 1;

			if (field_name == nullptr) {
				return;
			}

			++field_name;
			const char *array_start = strchr(format_buffer, '[');
			if (array_start != nullptr) {
				array_size = atoi(array_start + 1);
			}

			FieldLayout &field = layout.fields[layout.count++];
			field.type = c_type;
			field.offset = data_offset;
			field.array_size = array_size;
			strncpy(field.name, field_name, sizeof(field.name) - 1);
			field.name[sizeof(field.name) - 1] = '\0';

			if (c_type == nullptr) {
				// Nested message fields are not flattened yet, but their bytes must
				// still be skipped to keep the following fields aligned.
				data_offset += field_size_from_format(format_buffer[0], array_size);
				continue;
			}

			if (c_type != nullptr) {
				data_offset += primitive_size(c_type) * array_size;
			}
		}
		layout.valid = layout.count > 0;
	}

	void serialize_generic_topic(unsigned topic_index, const uint8_t *data)
	{
		const orb_metadata &meta = *_generic_meta[topic_index];
		strncpy(_topic, meta.o_name, sizeof(_topic) - 1);
		_topic[sizeof(_topic) - 1] = '\0';
		const TopicLayout &layout = _topic_layout[topic_index];
		if (!layout.valid) { return; }
		for (unsigned i = 0; i < layout.count; ++i) {
			const FieldLayout &field = layout.fields[i];
			if (field.type == nullptr || strncmp(field.name, "_padding", 8) == 0) { continue; }
			for (unsigned index = 0; index < field.array_size; ++index) {
				append_generic_value(field.type, field.name, index, field.array_size,
							 data + field.offset + index * primitive_size(field.type));
			}
		}

		if (!strcmp(meta.o_name, "vehicle_attitude")) {
			vehicle_attitude_s attitude{};
			memcpy(&attitude, data, sizeof(attitude));
			append_euler_angles(attitude.q);

		} else if (!strcmp(meta.o_name, "vehicle_attitude_setpoint")) {
			vehicle_attitude_setpoint_s attitude_setpoint{};
			memcpy(&attitude_setpoint, data, sizeof(attitude_setpoint));
			append_euler_angles(attitude_setpoint.q_d);

		} else if (!strcmp(meta.o_name, "vehicle_rates_setpoint")) {
			vehicle_rates_setpoint_s rates_setpoint{};
			memcpy(&rates_setpoint, data, sizeof(rates_setpoint));
			field_numeric("roll_rate", 0, 1, rates_setpoint.roll);
			field_numeric("pitch_rate", 0, 1, rates_setpoint.pitch);
			field_numeric("yaw_rate", 0, 1, rates_setpoint.yaw);
		}
	}

	void append_euler_angles(const float q[4])
	{
		const matrix::Eulerf euler{matrix::Quatf{q}};
		field_numeric("roll", 0, 1, euler.phi());
		field_numeric("pitch", 0, 1, euler.theta());
		field_numeric("yaw", 0, 1, euler.psi());
	}

	static size_t primitive_size(const char *c_type)
	{
		if (!strcmp(c_type, "int8_t") || !strcmp(c_type, "uint8_t") || !strcmp(c_type, "char") || !strcmp(c_type, "bool")) { return 1; }
		if (!strcmp(c_type, "int16_t") || !strcmp(c_type, "uint16_t")) { return 2; }
		if (!strcmp(c_type, "int32_t") || !strcmp(c_type, "uint32_t") || !strcmp(c_type, "float")) { return 4; }
		if (!strcmp(c_type, "int64_t") || !strcmp(c_type, "uint64_t") || !strcmp(c_type, "double")) { return 8; }
		return 0;
	}

	static size_t field_size_from_format(char type, int array_size)
	{
		const char *c_type = orb_get_c_type(type);
		return c_type ? primitive_size(c_type) * static_cast<size_t>(array_size) : 0;
	}

	void append_generic_value(const char *c_type, const char *name, int index, int array_size, const uint8_t *data)
	{
		const unsigned field_index_value = (array_size > 1) ? static_cast<unsigned>(index) : 0;
		if (!strcmp(c_type, "uint64_t")) {
			uint64_t value; memcpy(&value, data, sizeof(value));
			(array_size > 1) ? field_uint64_index(name, field_index_value, value) : field_uint64(name, value);
		} else if (!strcmp(c_type, "int64_t")) {
			int64_t value; memcpy(&value, data, sizeof(value));
			field_numeric(name, field_index_value, array_size, static_cast<double>(value));
		} else if (!strcmp(c_type, "double")) {
			double value; memcpy(&value, data, sizeof(value));
			field_numeric(name, field_index_value, array_size, value);
		} else if (!strcmp(c_type, "float")) {
			float value; memcpy(&value, data, sizeof(value));
			field_numeric(name, field_index_value, array_size, value);
		} else if (!strcmp(c_type, "int32_t")) {
			int32_t value; memcpy(&value, data, sizeof(value));
			field_numeric(name, field_index_value, array_size, value);
		} else if (!strcmp(c_type, "uint32_t")) {
			uint32_t value; memcpy(&value, data, sizeof(value));
			field_numeric(name, field_index_value, array_size, value);
		} else if (!strcmp(c_type, "int16_t")) {
			int16_t value; memcpy(&value, data, sizeof(value));
			field_numeric(name, field_index_value, array_size, value);
		} else if (!strcmp(c_type, "uint16_t")) {
			uint16_t value; memcpy(&value, data, sizeof(value));
			field_numeric(name, field_index_value, array_size, value);
		} else if (!strcmp(c_type, "int8_t")) {
			field_numeric(name, field_index_value, array_size, *reinterpret_cast<const int8_t *>(data));
		} else if (!strcmp(c_type, "uint8_t") || !strcmp(c_type, "char") || !strcmp(c_type, "bool")) {
			field_numeric(name, field_index_value, array_size, *data);
		}
	}

	void field_numeric(const char *name, unsigned index, int array_size, double value)
	{
		char text[192];
		const bool valid = std::isfinite(value) && fabs(value) <= VALUE_LIMIT;
		const int length = valid
				  ? ((array_size > 1)
				     ? snprintf(text, sizeof(text), ",\"%s/%s%u\":%.8g", _topic, name, index, value)
				     : snprintf(text, sizeof(text), ",\"%s/%s\":%.8g", _topic, name, value))
				  : ((array_size > 1)
				     ? snprintf(text, sizeof(text), ",\"%s/%s%u\":null", _topic, name, index)
				     : snprintf(text, sizeof(text), ",\"%s/%s\":null", _topic, name));

		if (length < 0 || (size_t)length >= sizeof(text)) {
			_message_valid = false;

		} else {
			append(text, (size_t)length);
		}
	}

	void field_uint64_index(const char *name, unsigned index, uint64_t value)
	{
		char text[192];
		const int length = snprintf(text, sizeof(text), ",\"%s/%s%u\":%" PRIu64, _topic, name, index, value);

		if (length < 0 || (size_t)length >= sizeof(text)) {
			_message_valid = false;

		} else {
			append(text, (size_t)length);
		}
	}

	static const char *const *generic_topic_names()
	{
		static const char *const names[] {
			"vehicle_attitude",
			"vehicle_local_position",
			"actuator_motors",
			"actuator_servos",
			"viztra_observation",
			"viztra_status",
			// Dock/inner-loop diagnosis: actual body rates and each controller
			// setpoint are exported as complete generic uORB JSON objects.
			"vehicle_angular_velocity",
			"vehicle_rates_setpoint",
			"vehicle_attitude_setpoint",
			"vehicle_local_position_setpoint",
			// Camera mount pitch is the commanded body-relative angle [deg];
			// camera pitch/yaw are target image errors, not servo feedback.
			"dock_guidance_setpoint"
		};
		static_assert(sizeof(names) / sizeof(names[0]) == GENERIC_TOPIC_COUNT,
			      "generic topic count does not match topic registry");
		return names;
	}
	void append(const char *text, size_t length)
	{
		if (!_message_valid || length + 1 > UDP_PAYLOAD_LIMIT) {
			_message_valid = false;
			++_oversize_drops;
			return;
		}

		if (_length + length + 1 > UDP_PAYLOAD_LIMIT) {
			// Keep every datagram below the Ethernet MTU. IP fragmentation
			// hides later JSON fields from UDP-port packet filters and is more
			// likely to be lost on embedded Ethernet paths.
			// Finish the current valid JSON object and continue the same topic
			// in a new packet. This avoids dropping a complete update batch when
			// several large uORB topics are updated in the same cycle.
			end();
			++_packet_splits;
			start_packet(hrt_absolute_time());
		}

		memcpy(_buffer + _length, text, length);
		_length += length;
	}
	void end()
	{
		if (!_message_valid || _length + 1 > UDP_PAYLOAD_LIMIT || _length + 1 >= sizeof(_buffer)) {
			_length = 0;
			return;
		}
		_buffer[_length++] = '}';

		if (_socket >= 0) {
			const ssize_t ret = ::sendto(_socket, _buffer, _length, 0,
						     (sockaddr *)&_dest, sizeof(_dest));

			if (ret >= 0) {
				++_sent;
				_bytes_sent += (uint32_t)ret;

			} else {
				++_send_errors;
				_last_errno = errno;
				const hrt_abstime now = hrt_absolute_time();

				// Do not flood the NuttX console if the host is unreachable.
				if (now - _last_error_report > 1_s) {
					PX4_WARN("sendto %s:%u failed: errno=%d len=%lu",
						 _address, _port, _last_errno, (unsigned long)_length);
					_last_error_report = now;
				}
			}
		}

		_length = 0;
	}

	int _socket{-1}; sockaddr_in _dest{}; char _address[INET_ADDRSTRLEN] {"192.168.3.100"};
	uint16_t _port{9870}; float _rate_hz{20.f}; float _requested_rate_hz{20.f}; float _max_rate_hz{20.f};
	param_t _max_rate_param{PARAM_INVALID};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uint32_t _sent{0}; uint32_t _bytes_sent{0};
	uint32_t _send_errors{0}; uint32_t _packet_splits{0}; uint32_t _oversize_drops{0};
	int _last_errno{0}; hrt_abstime _last_error_report{0};
	static constexpr double VALUE_LIMIT{1.e6};
	static constexpr size_t UDP_PAYLOAD_LIMIT{1400};
	char _buffer[UDP_PAYLOAD_LIMIT + 1]{}; char _topic[64]{}; size_t _length{0}; bool _message_valid{true};
	const orb_metadata *_generic_meta[GENERIC_TOPIC_COUNT]{};
	uORB::Subscription *_generic_subs[GENERIC_TOPIC_COUNT]{};
	uint8_t _generic_data[GENERIC_TOPIC_COUNT][2048]{};
	uint64_t _last_topic_timestamp[GENERIC_TOPIC_COUNT]{};
	TopicLayout _topic_layout[GENERIC_TOPIC_COUNT]{};
	const hrt_abstime _topic_interval_us[GENERIC_TOPIC_COUNT] {
		20_ms, 20_ms, 20_ms, 20_ms, 100_ms, 500_ms,
		20_ms, 20_ms, 20_ms, 20_ms, 20_ms
	};
	hrt_abstime _last_topic_send[GENERIC_TOPIC_COUNT]{};
};

extern "C" __EXPORT int uorb_udp_bridge_main(int argc, char *argv[])
{
	return UorbUdpBridge::main(argc, argv);
}
