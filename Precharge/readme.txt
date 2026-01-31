There are two options for f_postAIR
    PA0 is on timer 2, channel 1,
    PA6 is on timer 3, channel 1,
These pins are connected to each other on the PCB.

Selecting 1 over the other is a software decision chosen when configuring the TIMERS.
We decide to use 2 separate timers, 1 for postAIR and 1 for preAIR. 

TIMER1 has differnt interrupts than timer 3. Need to check what exactly this means. It probably triggers different callbacks.