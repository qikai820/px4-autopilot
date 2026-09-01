#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/uORBMessageFields.hpp>
#include <uORB/uORB.h>

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

class UorbUdpBridge final : public ModuleBase<UorbUdpBridge>, public px4::ScheduledWorkItem
{
public:
	UorbUdpBridge() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}
	~UorbUdpBridge() override
	{
		close_socket();
		for (unsigned i = 0; i < GENERIC_TOPIC_COUNT; ++i) {
			delete _generic_subs[i];
		}
	}

	static int task_spawn(int, char **)
	{
		UorbUdpBridge *instance = new UorbUdpBridge();
		if (!instance) { return PX4_ERROR; }
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		if (instance->init()) { return PX4_OK; }
		_object.store(nullptr);
		delete instance;
		return PX4_ERROR;
	}

	static UorbUdpBridge *instantiate(int, char **) { return new UorbUdpBridge(); }

	static int custom_command(int argc, char *argv[])
	{
		if (argc > 0 && !strcmp(argv[0], "config")) {
			UorbUdpBridge *instance = get_instance();
			if (!instance || argc < 2) { return PX4_ERROR; }
			const int port = (argc > 2) ? atoi(argv[2]) : 9870;
			const float rate = (argc > 3) ? strtof(argv[3], nullptr) : 50.f;
			return instance->configure(argv[1], port, rate) ? PX4_OK : PX4_ERROR;
		}
		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }
		PX4_INFO("usage: uorb_udp_bridge {start|stop|status}");
		PX4_INFO("  uorb_udp_bridge config <ip> [port] [rate_hz]");
		PX4_INFO("  default: 192.168.3.100:9870, 50 Hz");
		return 0;
	}

	int print_status() override
	{
		PX4_INFO("UDP fd=%d %s:%u at %.1f Hz, sent %lu datagrams (%lu bytes), errors %lu last_errno %d",
			 _socket, _address, _port, (double)_rate_hz, (unsigned long)_sent,
			 (unsigned long)_bytes_sent, (unsigned long)_send_errors, _last_errno);
		return 0;
	}

	bool init()
	{
		if (!configure(_address, _port, _rate_hz)) { return false; }
		init_generic_topics();
		return true;
	}

	void Run() override
	{
		if (should_exit()) {
			close_socket();
			ScheduleClear();
			return;
		}

		// NuttX file descriptors belong to the task group that created them.
		// Create the socket from the work-queue context where sendto() runs.
		if (_socket < 0 && !open_socket()) {
			return;
		}

		publish_generic_topics();
	}

private:
	bool configure(const char *address, int port, float rate)
	{
		if (port < 1 || port > 65535 || rate <= 0.f) { return false; }
		char address_copy[INET_ADDRSTRLEN]{};
		strncpy(address_copy, address, sizeof(address_copy) - 1);
		struct sockaddr_in dest{};
		dest.sin_family = AF_INET;
		dest.sin_port = htons((uint16_t)port);
		if (inet_aton(address, &dest.sin_addr) == 0) { return false; }
		_dest = dest;
		strncpy(_address, address_copy, sizeof(_address) - 1);
		_port = (uint16_t)port; _rate_hz = rate;
		ScheduleOnInterval((uint32_t)(1'000'000.f / rate));
		return true;
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
	void begin(const char *topic, uint64_t timestamp)
	{
		strncpy(_topic, topic, sizeof(_topic) - 1);
		_topic[sizeof(_topic) - 1] = '\0';
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
		for (unsigned i = 0; i < GENERIC_TOPIC_COUNT; ++i) {
			if (_generic_meta[i] == nullptr || _generic_subs[i] == nullptr
			    || _generic_meta[i]->o_size > sizeof(_generic_data[i])) {
				continue;
			}

			if (_generic_subs[i]->update(_generic_data[i])) {
				serialize_generic_topic(*_generic_meta[i], _generic_data[i]);
			}
		}
	}

	void serialize_generic_topic(const orb_metadata &meta, const uint8_t *data)
	{
		begin(meta.o_name, hrt_absolute_time());
		char format_buffer[128];
		uORB::MessageFormatReader format_reader(format_buffer, sizeof(format_buffer));

		if (!format_reader.readUntilFormat(meta.o_id)) {
			_length = 0;
			return;
		}

		int data_offset = 0;
		int field_length = 0;

		while (format_reader.readNextField(field_length)) {
			if (field_length <= 0 || field_length >= static_cast<int>(sizeof(format_buffer))) {
				_length = 0;
				return;
			}

			const char *c_type = orb_get_c_type(format_buffer[0]);
			const char *field_name = strchr(format_buffer, ' ');
			int array_size = 1;

			if (field_name == nullptr) {
				_length = 0;
				return;
			}

			++field_name;
			const char *array_start = strchr(format_buffer, '[');
			if (array_start != nullptr) {
				array_size = atoi(array_start + 1);
			}

			if (c_type == nullptr) {
				// Nested message fields are not flattened yet, but their bytes must
				// still be skipped to keep the following fields aligned.
				data_offset += field_size_from_format(format_buffer[0], array_size);
				continue;
			}

			for (int index = 0; index < array_size; ++index) {
				const uint8_t *value = data + data_offset;
				if (strncmp(field_name, "_padding", 8) != 0) {
					append_generic_value(c_type, field_name, index, array_size, value);
				}
				data_offset += primitive_size(c_type);
			}
		}

		end();
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
			"geo_control_attitude_setpoint",
			"vehicle_angular_velocity",
			"geo_control_position_adaptive",
			"vehicle_local_position",
			"vehicle_local_position_setpoint",
			"vehicle_angular_acceleration_setpoint",
			"vehicle_thrust_setpoint",
			"vehicle_torque_setpoint",
			"actuator_motors",
			"actuator_servos",
			"vehicle_control_mode",
			"viztra_observation",
			"viztra_status"
		};
		return names;
	}
	void append(const char *text, size_t length)
	{
		if (!_message_valid || _length > sizeof(_buffer) - 2 || length > sizeof(_buffer) - _length - 2) {
			// Never transmit a partially written JSON object. The next topic
			// starts a fresh message and can recover from this overflow.
			_message_valid = false;
			return;
		}

		memcpy(_buffer + _length, text, length);
		_length += length;
	}
	void end()
	{
		if (!_message_valid || _length + 1 >= sizeof(_buffer)) {
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
	uint16_t _port{9870}; float _rate_hz{50.f}; uint32_t _sent{0}; uint32_t _bytes_sent{0};
	uint32_t _send_errors{0}; int _last_errno{0}; hrt_abstime _last_error_report{0};
	static constexpr double VALUE_LIMIT{1.e6};
	static constexpr unsigned GENERIC_TOPIC_COUNT{14};
	char _buffer[4096]{}; char _topic[64]{}; size_t _length{0}; bool _message_valid{true};
	const orb_metadata *_generic_meta[GENERIC_TOPIC_COUNT]{};
	uORB::Subscription *_generic_subs[GENERIC_TOPIC_COUNT]{};
	uint8_t _generic_data[GENERIC_TOPIC_COUNT][2048]{};
};

extern "C" __EXPORT int uorb_udp_bridge_main(int argc, char *argv[])
{
	return UorbUdpBridge::main(argc, argv);
}
