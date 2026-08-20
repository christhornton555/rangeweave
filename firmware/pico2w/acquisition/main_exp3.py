"""Temporary exp3 hardware-validation harness for Rangeweave Pico acquisition.

This deliberately reuses the exp2 acquisition implementation and changes only scheduler
priority so we can A/B the lower-than-baseline ToF frame rate observed in exp2.

Run manually from Thonny after interrupting the auto-started exp2 main.py. Do not rename
this file to main.py: it imports the canonical exp2 module as its implementation base.
"""

import time

import main as exp2


FIRMWARE_LABEL = b"rangeweave-pico2w-acq-0.1-exp3"


class Acquisition(exp2.Acquisition):
    def __init__(self):
        # STREAM_INFO reads FIRMWARE_LABEL from the imported canonical module.
        exp2.FIRMWARE_LABEL = FIRMWARE_LABEL
        super().__init__()

    def run(self):
        """Use the scheduler order already validated by the v0.5 diagnostic.

        Priority is:
            MAG -> ToF -> MAG catch-up -> CLOCK_SYNC -> FIFO -> metadata -> USB

        The LSM hardware FIFO is intentionally serviced after ToF. Its purpose is to
        absorb host-side scheduling jitter, so draining it before checking the 15 Hz
        ToF device only adds latency to the time-sensitive ToF polling path.
        """

        now_us = self.stack.clock.now_us()
        next_mag_us = now_us + exp2.MAG_INITIAL_OFFSET_US
        next_fifo_us = now_us
        next_tof_poll_us = now_us
        next_sync_us = now_us + exp2.CLOCK_SYNC_INITIAL_US
        next_status_us = now_us + exp2.STATUS_PERIOD_US
        next_info_us = now_us + exp2.STREAM_INFO_PERIOD_US

        self.emit_stream_info()

        while True:
            # 1. Magnetometer when due.
            now_us = self.stack.clock.now_us()
            if now_us >= next_mag_us:
                self.service_mag()
                next_mag_us += exp2.MAG_PERIOD_US
                now_after = self.stack.clock.now_us()
                if now_after - next_mag_us > exp2.MAG_PERIOD_US:
                    next_mag_us = now_after + exp2.MAG_PERIOD_US

            # 2. ToF before FIFO service, matching the validated diagnostic order.
            now_us = self.stack.clock.now_us()
            if now_us >= next_tof_poll_us:
                next_tof_poll_us = now_us + exp2.TOF_POLL_US
                if next_mag_us - now_us > exp2.TOF_MAG_GUARD_US:
                    self.service_tof()

            # 3. If the long ToF transfer crossed a magnetometer deadline, catch up.
            now_us = self.stack.clock.now_us()
            if now_us >= next_mag_us:
                self.service_mag()
                next_mag_us += exp2.MAG_PERIOD_US

            # 4. Raw clock correlation.
            now_us = self.stack.clock.now_us()
            if now_us >= next_sync_us:
                time_to_mag = next_mag_us - now_us
                time_since_mag = now_us - self.stack.last_mag_bus_activity_us
                if (
                    time_to_mag > exp2.CLOCK_SYNC_MAG_GUARD_US
                    and time_since_mag > exp2.CLOCK_SYNC_MAG_GUARD_US
                ):
                    self.service_clock_sync()
                    next_sync_us += exp2.CLOCK_SYNC_PERIOD_US

            # 5. Drain the LSM hardware FIFO after the time-sensitive ToF poll.
            now_us = self.stack.clock.now_us()
            if now_us >= next_fifo_us:
                time_to_mag = next_mag_us - now_us
                time_since_mag = now_us - self.stack.last_mag_bus_activity_us
                if (
                    time_to_mag > exp2.FIFO_MAG_GUARD_US
                    and time_since_mag > exp2.FIFO_MAG_GUARD_US
                ):
                    self.service_fifo()
                    next_fifo_us += exp2.FIFO_SERVICE_PERIOD_US
                    now_after = self.stack.clock.now_us()
                    if now_after - next_fifo_us > exp2.FIFO_SERVICE_PERIOD_US:
                        next_fifo_us = now_after + exp2.FIFO_SERVICE_PERIOD_US

            now_us = self.stack.clock.now_us()
            if self.imu_batch and self.imu_batch_started_us is not None:
                if now_us - self.imu_batch_started_us >= exp2.IMU_BATCH_MAX_LATENCY_US:
                    self.flush_imu_batch()

            if now_us >= next_status_us:
                self.emit_status(now_us)
                next_status_us += exp2.STATUS_PERIOD_US

            if now_us >= next_info_us:
                self.emit_stream_info()
                next_info_us += exp2.STREAM_INFO_PERIOD_US

            # Keep the exp2 bounded multi-chunk USB transport unchanged.
            self.transport.service(self.queue)

            time.sleep_ms(1)


def main():
    acquisition = Acquisition()
    acquisition.run()


if __name__ == "__main__":
    main()
