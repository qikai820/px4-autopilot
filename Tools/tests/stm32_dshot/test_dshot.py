#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Exercise the actual STM32 DShot backend with mocked timer/DMA hardware."""

from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
BACKEND = ROOT / 'platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c'
HEADERS = [
    'px4_platform_common/px4_config.h', 'px4_platform_common/micro_hal.h',
    'stm32_dma.h', 'stm32_tim.h', 'px4_arch/dshot.h', 'px4_arch/io_timer.h',
    'drivers/drv_dshot.h', 'px4_platform_common/log.h',
    'drivers/drv_input_capture.h', 'lib/perf/perf_counter.h',
    'nuttx/irq.h',
]


class DShotTest(unittest.TestCase):
    def test_mixed_pwm_and_dshot_packet_routing(self):
        with tempfile.TemporaryDirectory(prefix='stm32-dshot-') as temp:
            temp = Path(temp)
            for name in HEADERS:
                header = temp / name
                header.parent.mkdir(parents=True, exist_ok=True)
                header.write_text('#include "mock_hardware.h"\n')
            executable = temp / 'test_dshot'
            subprocess.run([
                'cc', '-std=gnu11', '-O1', '-fno-pie', '-no-pie',
                '-Wno-pointer-to-int-cast', '-I', str(temp),
                '-I', str(Path(__file__).parent),
                '-DDSHOT_BACKEND="' + str(BACKEND) + '"',
                str(Path(__file__).with_name('test_dshot.c')),
                '-o', str(executable),
            ], check=True)
            for mode in ('sparse', 'all', 'bidirectional'):
                with self.subTest(mode=mode):
                    subprocess.run([str(executable), mode], check=True)


if __name__ == '__main__':
    unittest.main()
