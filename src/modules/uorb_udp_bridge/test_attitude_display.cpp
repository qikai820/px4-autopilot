#include "AttitudeDisplay.hpp"
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace matrix;

int main()
{
	for (float bank : {-.35f, 0.f, .35f}) {
		for (float heading : {-3.13f, -.3f, .8f, 3.13f}) {
			float previous_pitch = -M_PI_F;

			for (int degrees = -160; degrees <= 160; ++degrees) {
				const float pitch = math::radians(float(degrees));
				const Quatf q = Quatf(Eulerf(0.f, 0.f, heading)) * Quatf(Eulerf(bank, 0.f, 0.f)) * Quatf(Eulerf(0.f, pitch, 0.f));
				const Vector3f displayed = uorb_udp_bridge::attitudeDisplayAngles(q, 17);
				assert(fabsf(displayed(0) - bank) < 1e-5f);
				assert(fabsf(displayed(1) - pitch) < 1e-5f);
				assert(fabsf(wrap_pi(displayed(2) - heading)) < 1e-5f);
				assert(displayed(1) >= previous_pitch);
				previous_pitch = displayed(1);
				assert((displayed - uorb_udp_bridge::attitudeDisplayAngles(-q, 17)).norm() < 1e-6f);
				assert((displayed - uorb_udp_bridge::attitudeDisplayAngles(q * 1.002f, 17)).norm() < 1e-5f);
				const Quatf reconstructed = Quatf(Eulerf(0.f, 0.f, displayed(2))) * Quatf(Eulerf(displayed(0), 0.f, 0.f)) * Quatf(Eulerf(0.f, displayed(1),
							    0.f));
				assert((q.inversed() * reconstructed).canonical().imag().norm() < 1e-5f);

				for (int airframe : {0, 16}) {
					assert((uorb_udp_bridge::attitudeDisplayAngles(q, airframe) - Vector3f(Eulerf(q))).norm() < 1e-6f);
				}
			}
		}
	}
	assert(!uorb_udp_bridge::attitudeDisplayAngles(Quatf(0.f, 0.f, 0.f, 0.f), 17).isAllFinite());
	assert(!uorb_udp_bridge::attitudeDisplayAngles(Quatf(NAN, 0.f, 0.f, 0.f), 17).isAllFinite());
	const Vector3f singular = uorb_udp_bridge::attitudeDisplayAngles(Quatf(Eulerf(M_PI_2_F, 0.f, 0.f)), 17);
	assert(PX4_ISFINITE(singular(0)) && !PX4_ISFINITE(singular(1)) && !PX4_ISFINITE(singular(2)));
	const auto example = uorb_udp_bridge::attitudeDisplayAngles(Quatf(Eulerf(0.f, math::radians(160.f), 0.f)), 17);
	printf("PASS: Voliro +/-160 continuous pitch, bank/heading, quaternion round-trip/sign/normalization, legacy airframes and invalid inputs\n");
	printf("pitch at +160 deg: %.7f rad\n", (double)example(1));
}
