// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_IO_TIMERS 3
#define MAX_TIMER_IO_CHANNELS 8
#define IO_TIMER_ALL_MODES_CHANNELS UINT16_MAX
#define DSHOT_MOTOR_PWM_BIT_WIDTH 20
#define STM32_GTIM_DMAR_OFFSET 0x4c
#define STM32_GTIM_CCR1_OFFSET 0x34
#define STM32_GTIM_CR1_OFFSET 0
#define STM32_GTIM_CCER_OFFSET 0x20
#define STM32_GTIM_DIER_OFFSET 0x0c
#define STM32_GTIM_PSC_OFFSET 0x28
#define STM32_GTIM_ARR_OFFSET 0x2c
#define STM32_GTIM_DCR_OFFSET 0x48
#define DMA_SCR_PRIHI 0
#define DMA_SCR_MSIZE_32BITS 0
#define DMA_SCR_PSIZE_32BITS 0
#define DMA_SCR_MINC 0
#define DMA_SCR_DIR_M2P 0
#define DMA_SCR_TCIE 0
#define DMA_SCR_TEIE 0
#define DMA_SCR_DMEIE 0
#define DMA_SCR_MSIZE_16BITS 0
#define DMA_SCR_PSIZE_16BITS 0
#define DMA_SCR_DIR_P2M 0
#define DMA_STATUS_TCIF 0x20
#define DMA_STATUS_ERROR 0x0c
#define PC_ELAPSED 0
#define PX4_OK 0
#define OK 0
#define PX4_ERROR -1
#define px4_cache_aligned_data() __attribute__((aligned(32)))
#define PX4_INFO(...) do {} while (0)
#define PX4_WARN(...) do {} while (0)
#define PX4_DEBUG(...) do {} while (0)

typedef uint64_t hrt_abstime;
typedef unsigned irqstate_t;
static irqstate_t px4_enter_critical_section(void) { return 0; }
static void px4_leave_critical_section(irqstate_t flags) {}
static uint32_t getreg32(uintptr_t address) { return 0; }
typedef void *perf_counter_t;
struct hrt_call { int unused; };
typedef struct mock_dma_s mock_dma_t;
typedef mock_dma_t *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE, uint8_t, void *);
struct mock_dma_s {
	unsigned timer;
	uintptr_t memory;
	unsigned count;
	dma_callback_t callback;
	void *arg;
	unsigned starts;
};
typedef enum {
	IOTimerChanMode_NotUsed,
	IOTimerChanMode_PWMOut,
	IOTimerChanMode_Dshot,
	IOTimerChanMode_DshotInverted,
	IOTimerChanMode_CaptureDMA,
} io_timer_channel_mode_t;
typedef struct {
	uint32_t base;
	uint32_t clock_freq;
	struct { unsigned dma_base, dma_map_up, dma_map_ch[4]; } dshot;
} io_timers_t;
typedef struct { uint8_t timer_index, timer_channel; } timer_io_channels_t;
typedef struct {
	unsigned first_channel_index, channel_count, lowest_timer_channel, channel_count_including_gaps;
} io_timers_channel_mapping_element_t;
typedef struct { io_timers_channel_mapping_element_t element[MAX_IO_TIMERS]; } io_timers_channel_mapping_t;

// CUAV 7 Nano: MAIN1-4=TIM5, MAIN5-6=TIM4, MAIN7-8=TIM1.
static const io_timers_t io_timers[] = {
	{0x40000c00, 240000000, {1, 1, {1, 1, 1, 1}}},
	{0x40000800, 240000000, {1, 2, {2, 2, 2, 2}}},
	{0x40010000, 240000000, {0, 0, {0}}},
};
static const timer_io_channels_t timer_io_channels[] = {
	{0, 4}, {0, 3}, {0, 2}, {0, 1}, {1, 2}, {1, 3}, {2, 2}, {2, 1},
};
static const io_timers_channel_mapping_t io_timers_channel_mapping = {
	{{0, 4, 1, 4}, {4, 2, 2, 2}, {6, 2, 1, 2}}
};
static mock_dma_t mock_dma[MAX_IO_TIMERS];
static bool mock_dma_request[MAX_IO_TIMERS];
static io_timer_channel_mode_t mock_mode[MAX_IO_TIMERS];

static DMA_HANDLE stm32_dmachannel(unsigned map)
{
	mock_dma_t *dma = &mock_dma[map - 1];
	dma->timer = map - 1;
	return dma;
}
static void stm32_dmastop(DMA_HANDLE dma) { (void)dma; }
static void stm32_dmafree(DMA_HANDLE dma) { (void)dma; }
static size_t stm32_dmaresidual(DMA_HANDLE dma) { return dma->count; }
static void px4_stm32_dmasetup(DMA_HANDLE dma, uint32_t peripheral, uint32_t memory,
		unsigned count, unsigned config)
{
	assert(peripheral == io_timers[dma->timer].base + STM32_GTIM_DMAR_OFFSET);
	dma->memory = memory;
	dma->count = count;
}
static void stm32_dmastart(DMA_HANDLE dma, dma_callback_t callback, void *arg, bool half)
{
	assert(!mock_dma_request[dma->timer]);
	dma->callback = callback;
	dma->arg = arg;
	++dma->starts;
}
static int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode, void *callback, void *context)
{
	unsigned timer = timer_io_channels[channel].timer_index;
	assert(mock_mode[timer] == IOTimerChanMode_NotUsed || mock_mode[timer] == mode);
	mock_mode[timer] = mode;
	return 0;
}
static int io_timer_unallocate_timer(unsigned timer) { mock_mode[timer] = 0; return 0; }
static int io_timer_unallocate_channel(unsigned channel) { return 0; }
static unsigned io_timer_get_group(unsigned timer) { return timer == 0 ? 0x0f : timer == 1 ? 0x30 : 0xc0; }
static int io_timer_set_enable(bool state, io_timer_channel_mode_t mode, unsigned mask)
{
	return 0;
}
static int io_timer_set_dshot_burst_mode(unsigned timer, unsigned frequency, unsigned channels)
{
	assert(channels == io_timers_channel_mapping.element[timer].channel_count_including_gaps);
	return 0;
}
static void io_timer_update_dma_req(unsigned timer, bool enable) { mock_dma_request[timer] = enable; }
static void io_timer_set_dshot_capture_mode(unsigned timer, unsigned channel, unsigned frequency) {}
static void io_timer_capture_dma_req(unsigned timer, unsigned channel, bool enable) {}
static void up_clean_dcache(uintptr_t start, uintptr_t end) {}
static void up_invalidate_dcache(uintptr_t start, uintptr_t end) {}
static void hrt_call_after(struct hrt_call *call, hrt_abstime delay, void (*callback)(void *), void *arg) {}
static perf_counter_t perf_alloc(int type, const char *name) { return NULL; }
static void perf_begin(perf_counter_t counter) {}
static void perf_end(perf_counter_t counter) {}
