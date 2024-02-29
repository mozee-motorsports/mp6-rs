- ADC1_IN2: On Pin PF11
- PWM generation is going to be on TIM4, which is a general purpose timer, described on p1684. CH1 PWM output goes to PD12.
- [CAN Video](https://www.youtube.com/watch?v=sY1ie-CnOR0)
- [FDCAN Bitrate Calculator](https://www.kvaser.com/support/calculators/can-fd-bit-timing-calculator/)
    - FDCAN Clock is 137.5MHz
    - Guess on tolerance: 4687.5 ppm
    - Guess on maximum propogation delay for CAN tranciever: 180ns (Note: this is assuming MCP2561 Tranciever. We need to check this when we find the schematic). 
    - Nominal Bitrate (arbitration bitrate): 500kbps
    - Data Bitrate: 500kbps
    


