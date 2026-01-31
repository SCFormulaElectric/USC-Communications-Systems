There are two options for f_postAIR
    PA0 is on timer 2, channel 1,
    PA6 is on timer 3, channel 1,
These pins are connected to each other on the PCB.

Selecting 1 over the other is a software decision chosen when configuring the TIMERS.
We decide to use 2 separate timers, 1 for postAIR and 1 for preAIR. 

TIMER1 has differnt interrupts than timer 3. Need to check what exactly this means. It probably triggers different callbacks.

Input voltage is max 400. After 4x210k R, == 12V. (Absolute voltage is not actually necessary).

We just need to find the ratio of pre and post frequencies. and ensure its 90% around 0.5s.

<0.1s, shut down. >1s, shutdown

if charged, set flag

We are using SDCIN PA2 as frequency input. This means pa0 which is postAIR is not enabled. Testing logic right now.

There is no UART...

Debugging this is gonna be involved