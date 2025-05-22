# SVPWM-STM32-Motor-Project
Purpose: Drive a 3 phase motor using SVPWM with the STM32L476RGT6 development board.

### Authors
Ryan Cramer, Avery White

### Project Description & Background 
This project implements space vector pulse width modulation (SVPWM) control of a 3-phase brushless DC (BLDC) motor using an STM32 Nucleo board. The STM32 generates six complementary PWM signals—two per motor phase—corresponding to the high-side and low-side gate drive signals for each inverter leg. Due to the STM32's limited GPIO output voltage (3.3V) and current (<100 mA), it cannot directly drive the power MOSFETs. Therefore, Infineon's IR2110 gate drivers are used to level-shift and amplify the PWM signals. Each IR2110 drives one high-side and one low-side N-channel MOSFET, forming a half-bridge. The IR2110's floating high-side architecture, powered via a bootstrap capacitor and diode, enables reliable gate drive even as the source node (motor phase) swings rapidly due to switching transients and motor back EMF. This architecture allows the STM32 to control high-voltage motor drive circuits safely and efficiently using standard low-voltage logic.

### SVPWM

### Schematic
