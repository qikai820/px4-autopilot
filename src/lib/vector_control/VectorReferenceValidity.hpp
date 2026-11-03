#pragma once

#include <stdint.h>

namespace vector_control
{

/** Reject a stale, future, out-of-order or wrong-estimator-frame sidecar.
 * Both publishers stamp the standard setpoint and its vector extension with
 * the same timestamp; the sample timestamp follows the originating sensor. */
inline bool referenceMatches(uint64_t now, uint64_t setpoint, uint64_t reference, uint64_t sample,
			     uint8_t reference_epoch, uint8_t current_epoch)
{
	return reference != 0 && reference == setpoint && reference <= now && now - reference < 100000
	       && sample != 0 && sample <= reference && now - sample < 100000 && reference_epoch == current_epoch;
}

} // namespace vector_control
