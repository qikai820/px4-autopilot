// SPDX-License-Identifier: BSD-3-Clause
#include "mock_hardware.h"
#include DSHOT_BACKEND

static void finish_frame(unsigned timer, uint8_t status)
{
	mock_dma_t *dma = &mock_dma[timer];
	dma->count = 0;
	if (dma->callback) {
		dma->callback(dma, status, dma->arg);
	}
}

static void check_packet(unsigned output, uint16_t data)
{
	unsigned timer = timer_io_channels[output].timer_index;
	const io_timers_channel_mapping_element_t *map = &io_timers_channel_mapping.element[timer];
	unsigned column = timer_io_channels[output].timer_channel - map->lowest_timer_channel;
	uint32_t *buffer = (uint32_t *)mock_dma[timer].memory;
	uint16_t packet = 0;
	for (unsigned bit = 0; bit < 16; ++bit) {
		uint32_t pulse = buffer[bit * map->channel_count_including_gaps + column];
		assert(pulse == 7 || pulse == 14);
		packet = (packet << 1) | (pulse == 14);
	}
	assert((packet >> 5) == data);
	unsigned checksum = ((packet >> 4) ^ (packet >> 8) ^ (packet >> 12)) & 15;
	assert((packet & 15) == (_bidirectional ? (~checksum & 15) : checksum));
	assert(buffer[16 * map->channel_count_including_gaps + column] == 0);
}

int main(int argc, char **argv)
{
	if (strcmp(argv[1], "bidirectional") == 0) {
		assert(up_dshot_init(0x0f, 150000, true) == 0x0f);
		dshot_motor_data_set(0, 196, false);
		up_dshot_trigger();
		check_packet(0, 196);
		assert(mock_dma[0].callback == dma_burst_finished_callback);
		puts("PASS: bidirectional transmit setup and inverted checksum");
		return 0;
	}

	bool sparse = strcmp(argv[1], "sparse") == 0;
	mock_mode[2] = IOTimerChanMode_PWMOut;
	if (sparse) { mock_mode[0] = IOTimerChanMode_PWMOut; }
	unsigned mask = sparse ? 0x30 : 0x3f;
	assert(up_dshot_init(mask, 150000, false) == mask);
	const uint16_t payloads[] = {0, 1, 196, 508, 2047};
	for (unsigned frame = 0; frame < sizeof(payloads) / sizeof(payloads[0]); ++frame) {
		for (unsigned output = 0; output < 4; ++output) {
			dshot_motor_data_set(output, 100 + frame + output, false);
		}
		dshot_motor_data_set(6, 100, false); // A PWM servo output must be ignored.
		dshot_motor_data_set(4, payloads[frame], false);
		dshot_motor_data_set(5, 0, false);
		up_dshot_trigger();
		// NuttX may dispatch a pending IRQ after its old flags were cleared.
		mock_dma[1].callback(&mock_dma[1], 0, mock_dma[1].arg);
		assert(mock_dma_request[1]);
		check_packet(4, payloads[frame]);
		assert(mock_mode[2] == IOTimerChanMode_PWMOut);
		if (sparse) {
			assert(mock_mode[0] == IOTimerChanMode_PWMOut);
			assert(mock_dma[0].starts == 0);
		} else {
			for (unsigned output = 0; output < 4; ++output) {
				check_packet(output, 100 + frame + output);
			}
			finish_frame(0, DMA_STATUS_TCIF);
		}
		finish_frame(1, frame == 1 ? DMA_STATUS_ERROR : DMA_STATUS_TCIF);
	}
	assert(timer_configs[1].output_complete == 4);
	assert(timer_configs[1].output_errors == 1);
	puts("PASS: sparse/all DShot timers, M5 packet/CRC, PWM ownership and DMA completion/error counters");
	return 0;
}
