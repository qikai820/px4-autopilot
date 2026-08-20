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
#include <matrix/matrix/math.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <cmath>

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
		PX4_INFO("UDP %s:%u at %.1f Hz, sent %u datagrams", _address, _port, (double)_rate_hz, _sent);
		return 0;
	}

	bool init()
	{
		if (!configure(_address, _port, _rate_hz)) { return false; }
		return true;
	}

	void Run() override
	{
		if (should_exit()) { ScheduleClear(); return; }

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
			field("roll", attitude_sp.roll); field("pitch", attitude_sp.pitch); field("yaw", attitude_sp.yaw); end();
		}

		vehicle_angular_velocity_s rates{};
		if (_rates_sub.update(&rates)) {
			begin("vehicle_angular_velocity", rates.timestamp);
			field("x", rates.xyz[0]); field("y", rates.xyz[1]); field("z", rates.xyz[2]); end();
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
		const int sock = socket(AF_INET, SOCK_DGRAM, 0);
		if (sock < 0) { PX4_ERR("UDP socket failed"); return false; }
		struct sockaddr_in dest{};
		dest.sin_family = AF_INET;
		dest.sin_port = htons((uint16_t)port);
		if (inet_aton(address, &dest.sin_addr) == 0) { ::close(sock); return false; }
		close_socket(); _socket = sock; _dest = dest;
		strncpy(_address, address_copy, sizeof(_address) - 1);
		_port = (uint16_t)port; _rate_hz = rate;
		ScheduleOnInterval((uint32_t)(1'000'000.f / rate));
		return true;
	}

	void close_socket() { if (_socket >= 0) { ::close(_socket); _socket = -1; } }
	void begin(const char *topic, uint64_t timestamp)
	{
		strncpy(_topic, topic, sizeof(_topic) - 1);
		_length = (size_t)snprintf(_buffer, sizeof(_buffer), "{\"time\":%.6f",
					(double)timestamp * 1e-6);
	}
	void field(const char *name, float value)
	{
		char text[64];
		if (std::isfinite(value)) { snprintf(text, sizeof(text), ",\"%s/%s\":%.8g", _topic, name, (double)value); }
		else { snprintf(text, sizeof(text), ",\"%s/%s\":null", _topic, name); }
		append(text);
	}
	void field_index(const char *name, unsigned index, float value)
	{
		char text[72];
		if (std::isfinite(value)) { snprintf(text, sizeof(text), ",\"%s/%s%u\":%.8g", _topic, name, index, (double)value); }
		else { snprintf(text, sizeof(text), ",\"%s/%s%u\":null", _topic, name, index); }
		append(text);
	}
	void append(const char *text)
	{
		const size_t n = strlen(text);
		if (_length + n + 2 < sizeof(_buffer)) { memcpy(_buffer + _length, text, n); _length += n; }
	}
	void end()
	{
		if (_length + 1 >= sizeof(_buffer)) { return; }
		_buffer[_length++] = '}';
		if (_socket >= 0) { ::sendto(_socket, _buffer, _length, MSG_DONTWAIT, (sockaddr *)&_dest, sizeof(_dest)); ++_sent; }
		_length = 0;
	}

	int _socket{-1}; sockaddr_in _dest{}; char _address[INET_ADDRSTRLEN] {"192.168.3.100"};
	uint16_t _port{9870}; float _rate_hz{50.f}; uint32_t _sent{0};
	char _buffer[1400]{}; char _topic[64]{}; size_t _length{0};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _geo_attitude_sp_sub{ORB_ID(geo_control_attitude_setpoint)};
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
