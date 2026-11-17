#include <lib/vector_control/VoliroTiltMonitor.hpp>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using Monitor = vector_control::VoliroTiltMonitor;

int main(int argc, char **argv)
{
	assert(argc == 2);
	std::ifstream file(argv[1]);
	assert(file.good());
	// The logged hover normalization and physical parameters, not the current
	// example airframe defaults. This is a sensor replay, not flight validation.
	Monitor::Settings settings;
	settings.x = .02f; settings.tail = .74f;
	settings.mass = .2f * (4.f * 39.5f + 10.f) / 9.80665f;
	settings.inertia = {.18f, .12f, .38f};
	settings.servo_rate = math::radians(200.f);
	settings.max_speed = 5.f;
	auto legacy_settings = settings;
	legacy_settings.max_speed = 0.f;
	Monitor legacy, guarded;
	std::string line;
	uint64_t start = 0, first_fault = 0;
	unsigned count = 0, inhibited = 0;

	while (std::getline(file, line)) {
		if (line.empty() || line.front() == '#') { continue; }

		std::replace(line.begin(), line.end(), ',', ' ');
		std::istringstream row(line);
		Monitor::Sample sample;
		uint64_t now;
		row >> now >> sample.acceleration_timestamp >> sample.angular_timestamp
		    >> sample.command_timestamp >> sample.velocity_timestamp;

		for (int i = 0; i < 3; ++i) { row >> sample.specific_force(i); }

		for (int i = 0; i < 3; ++i) { row >> sample.rates(i); }

		for (int i = 0; i < 3; ++i) { row >> sample.angular_acceleration(i); }

		row >> sample.command >> sample.speed;
		assert(!row.fail());

		if (!start) { start = now; }

		legacy.update(legacy_settings, sample, now, true, true, true);
		guarded.update(settings, sample, now, true, true, true);

		if (legacy.fault() && !first_fault) { first_fault = now; }

		if (!Monitor::motionObservable(settings, sample, now)) { ++inhibited; assert(!guarded.valid()); }

		assert(!guarded.fault());
		++count;
	}

	assert(count == 250 && legacy.fault() && inhibited > 200);
	printf("PASS: log17 aligned sensor replay (%u rows): legacy fault after %.3fs, speed-gated fault=%d, inhibited=%u\n",
	       count, double(first_fault - start) * 1e-6, guarded.fault(), inhibited);
}
