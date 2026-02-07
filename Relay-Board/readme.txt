2/7/2026

Code was tested. WIP.

Initial commit:
Reset only if all faults are cleared, check for all faults immediately.

Now, reset only if no BMS fault. Check for IMD fault after 2 seconds
On reset, clear latch, set state to SAFE, and wait another 2 seconds before counting IMD faults
During these 2 seconds, the stm32 still looks for BMS errors.
