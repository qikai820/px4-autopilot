#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_acceleration_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/geo_control_attitude_setpoint.h>
#include <uORB/topics/geo_control_position_adaptive.h>
#include <matrix/matrix/math.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <cmath>
#include <errno.h>

using namespace time_literals;

class UorbUdpBridge final : public ModuleBase<UorbUdpBridge>, public px4::ScheduledWorkItem
{
public:
	UorbUdpBridge() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}
	~UorbUdpBridge() override { close_socket(); }

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

		vehicle_attitude_s attitude{};
		if (_attitude_sub.update(&attitude)) {
			begin("vehicle_attitude", attitude.timestamp);
			field("q0", attitude.q[0]); field("q1", attitude.q[1]);
			field("q2", attitude.q[2]); field("q3", attitude.q[3]); end();
			const matrix::Eulerf euler(matrix::Quatf(attitude.q));
			begin("vehicle_attitude_euler", attitude.timestamp);
			field("roll", euler.phi()); field("pitch", euler.theta()); field("yaw", euler.psi()); end();
		}

		geo_control_attitude_setpoint_s attitude_sp{};
		if (_geo_attitude_sp_sub.update(&attitude_sp)) {
			begin("geo_control_attitude_setpoint", attitude_sp.timestamp);
			field("q0", attitude_sp.q[0]); field("q1", attitude_sp.q[1]);
			field("q2", attitude_sp.q[2]); field("q3", attitude_sp.q[3]);
			field("roll", attitude_sp.roll); field("pitch", attitude_sp.pitch); field("yaw", attitude_sp.yaw);
			field("attitude_integral_x", attitude_sp.attitude_integral[0]);
			field("attitude_integral_y", attitude_sp.attitude_integral[1]);
			field("attitude_integral_z", attitude_sp.attitude_integral[2]);
			field("rate_integral_x", attitude_sp.rate_integral[0]);
			field("rate_integral_y", attitude_sp.rate_integral[1]);
			field("rate_integral_z", attitude_sp.rate_integral[2]);
			field("disturbance_torque_x", attitude_sp.disturbance_torque[0]);
			field("disturbance_torque_y", attitude_sp.disturbance_torque[1]);
			field("disturbance_torque_z", attitude_sp.disturbance_torque[2]);
			field("adaptive_torque_x", attitude_sp.adaptive_torque[0]);
			field("adaptive_torque_y", attitude_sp.adaptive_torque[1]);
			field("adaptive_torque_z", attitude_sp.adaptive_torque[2]); end();
			begin("geo_control_rate_setpoint", attitude_sp.timestamp);
			field("roll", attitude_sp.rates[0]); field("pitch", attitude_sp.rates[1]);
			field("yaw", attitude_sp.rates[2]); end();
		}

		vehicle_angular_velocity_s rates{};
		if (_rates_sub.update(&rates)) {
			begin("vehicle_angular_velocity", rates.timestamp);
			field("x", rates.xyz[0]); field("y", rates.xyz[1]); field("z", rates.xyz[2]); end();
		}

		geo_control_position_adaptive_s adaptive{};
		if (_adaptive_sub.update(&adaptive)) {
			begin("geo_control_position_adaptive", adaptive.timestamp);
			field_index("position_bias_velocity", 0, adaptive.position_bias_velocity[0]);
			field_index("position_bias_velocity", 1, adaptive.position_bias_velocity[1]);
			field_index("position_bias_velocity", 2, adaptive.position_bias_velocity[2]);
			field_index("velocity_disturbance_acceleration", 0, adaptive.velocity_disturbance_acceleration[0]);
			field_index("velocity_disturbance_acceleration", 1, adaptive.velocity_disturbance_acceleration[1]);
			field_index("velocity_disturbance_acceleration", 2, adaptive.velocity_disturbance_acceleration[2]);
			field_index("acceleration_observation", 0, adaptive.acceleration_observation[0]);
			field_index("acceleration_observation", 1, adaptive.acceleration_observation[1]);
			field_index("acceleration_observation", 2, adaptive.acceleration_observation[2]);
			field_index("velocity_integral", 0, adaptive.velocity_integral[0]);
			field_index("velocity_integral", 1, adaptive.velocity_integral[1]);
			field_index("velocity_integral", 2, adaptive.velocity_integral[2]);
			field("hover_thrust_estimate", adaptive.hover_thrust_estimate);
			field("hover_thrust_observation", adaptive.hover_thrust_observation);
			field("hover_thrust_learning", adaptive.hover_thrust_learning ? 1.f : 0.f);
			field("vertical_capture_active", adaptive.vertical_capture_active ? 1.f : 0.f);
			field("vertical_capture_velocity", adaptive.vertical_capture_velocity);
			field("valid", adaptive.valid ? 1.f : 0.f);
			field("mode", (float)adaptive.mode);
			end();
		}

		vehicle_local_position_s position{};
		if (_position_sub.update(&position)) {
			begin("vehicle_local_position", position.timestamp);
			field("x", position.x); field("y", position.y); field("z", position.z);
			field("vx", position.vx); field("vy", position.vy); field("vz", position.vz);
			field("dist_bottom", position.dist_bottom); end();
		}

		vehicle_local_position_setpoint_s position_sp{};
		if (_position_sp_sub.update(&position_sp)) {
			begin("vehicle_local_position_setpoint", position_sp.timestamp);
			field("x", position_sp.x); field("y", position_sp.y); field("z", position_sp.z);
			field("vx", position_sp.vx); field("vy", position_sp.vy); field("vz", position_sp.vz);
			field_index("acceleration", 0, position_sp.acceleration[0]);
			field_index("acceleration", 1, position_sp.acceleration[1]);
			field_index("acceleration", 2, position_sp.acceleration[2]);
			field_index("thrust", 0, position_sp.thrust[0]);
			field_index("thrust", 1, position_sp.thrust[1]);
			field_index("thrust", 2, position_sp.thrust[2]);
			field("yaw", position_sp.yaw); field("yawspeed", position_sp.yawspeed); end();
		}

		vehicle_angular_acceleration_setpoint_s angular_accel_sp{};
		if (_angular_accel_sp_sub.update(&angular_accel_sp)) {
			begin("vehicle_angular_acceleration_setpoint", angular_accel_sp.timestamp);
			field("x", angular_accel_sp.xyz[0]); field("y", angular_accel_sp.xyz[1]);
			field("z", angular_accel_sp.xyz[2]); end();
		}

		vehicle_thrust_setpoint_s thrust{};
		if (_thrust_sub.update(&thrust)) {
			begin("vehicle_thrust_setpoint", thrust.timestamp);
			field("x", thrust.xyz[0]); field("y", thrust.xyz[1]); field("z", thrust.xyz[2]); end();
		}

		vehicle_torque_setpoint_s torque{};
		if (_torque_sub.update(&torque)) {
			begin("vehicle_torque_setpoint", torque.timestamp);
			field("x", torque.xyz[0]); field("y", torque.xyz[1]); field("z", torque.xyz[2]); end();
		}

		actuator_motors_s motors{};
		if (_motors_sub.update(&motors)) {
			begin("actuator_motors", motors.timestamp);
			for (unsigned i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) { field_index("control", i, motors.control[i]); }
			end();
		}

		actuator_servos_s servos{};
		if (_servos_sub.update(&servos)) {
			begin("actuator_servos", servos.timestamp);
			for (unsigned i = 0; i < actuator_servos_s::NUM_CONTROLS; ++i) { field_index("control", i, servos.control[i]); }
			end();
		}

		vehicle_control_mode_s mode{};
		if (_mode_sub.update(&mode)) {
			begin("vehicle_control_mode", mode.timestamp);
			field("armed", (float)mode.flag_armed);
			field("offboard", (float)mode.flag_control_offboard_enabled);
			field("manual", (float)mode.flag_control_manual_enabled);
			field("allocation", (float)mode.flag_control_allocation_enabled); end();
		}
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
	void field(const char *name, float value)
	{
		char text[192];
		const bool valid = std::isfinite(value) && fabsf(value) <= VALUE_LIMIT;
		const int length = valid
				  ? snprintf(text, sizeof(text), ",\"%s/%s\":%.8g", _topic, name, (double)value)
				  : snprintf(text, sizeof(text), ",\"%s/%s\":null", _topic, name);

		if (length < 0 || (size_t)length >= sizeof(text)) {
			_message_valid = false;

		} else {
			append(text, (size_t)length);
		}
	}
	void field_index(const char *name, unsigned index, float value)
	{
		char text[192];
		const bool valid = std::isfinite(value) && fabsf(value) <= VALUE_LIMIT;
		const int length = valid
				  ? snprintf(text, sizeof(text), ",\"%s/%s%u\":%.8g", _topic, name, index, (double)value)
				  : snprintf(text, sizeof(text), ",\"%s/%s%u\":null", _topic, name, index);

		if (length < 0 || (size_t)length >= sizeof(text)) {
			_message_valid = false;

		} else {
			append(text, (size_t)length);
		}
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
	static constexpr float VALUE_LIMIT{1.e6f};
	char _buffer[4096]{}; char _topic[64]{}; size_t _length{0}; bool _message_valid{true};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _geo_attitude_sp_sub{ORB_ID(geo_control_attitude_setpoint)};
	uORB::Subscription _adaptive_sub{ORB_ID(geo_control_position_adaptive)};
	uORB::Subscription _rates_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _position_sp_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _angular_accel_sp_sub{ORB_ID(vehicle_angular_acceleration_setpoint)};
	uORB::Subscription _thrust_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _torque_sub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Subscription _motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _servos_sub{ORB_ID(actuator_servos)};
	uORB::Subscription _mode_sub{ORB_ID(vehicle_control_mode)};
};

extern "C" __EXPORT int uorb_udp_bridge_main(int argc, char *argv[])
{
	return UorbUdpBridge::main(argc, argv);
}
