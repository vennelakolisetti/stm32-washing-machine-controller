# STM32 Washing Machine Controller

EG7014 Assignment 1 – reliable washing-machine controller for the **NUCLEO-F401RE** (STM32F401RE).

Implemented as a GRAFCET with set/reset equations, EXTI button inputs, a one-shot timer (TIM3),
a PWM motor LED (TIM2), SLEEP mode between events, a 2 s independent watchdog and status
messages on the ST-LINK virtual COM port.

## Files

| File | Description |
|---|---|
| `main.c` | Controller source. Replace only `main.c` in your STM32CubeIDE project (keep `stm32f4xx_it.c` and the MSP files). |
| `README.md` | This file. |

## Wiring

| Signal | Pin | Type |
|---|---|---|
| B1 blue (arm / clear fault) | PC13 | EXTI13 falling |
| Start | PA0 | EXTI0 rising, pull-down |
| WLH (drum full) | PA1 | EXTI1 rising, pull-down |
| TempR | PA4 | EXTI4 rising, pull-down |
| WLLow (drum empty) | PC7 | EXTI7 rising, pull-down |
| Stop | PB6 | EXTI6 rising, pull-down |
| Valve | PA8 | LED |
| Heater | PA10 | LED |
| Door lock | PB4 | LED |
| Pump | PB5 | LED |
| Motor | PB10 | TIM2_CH3 PWM LED |
| LD2 (on board) | PA5 | steady = armed, fast blink = fault |

## How to operate

1. Optional: open a serial terminal on the ST-LINK COM port, 115200 8N1.
2. Press **B1** – LD2 turns on (armed). Button presses before this are ignored.
3. **Start** – valve and door lock on.
4. **WLH** – valve off, heater on.
5. **TempR** – heater off, wash starts.
6. The wash runs by itself: left 2 s, off 1 s, right 2 s, off 1 s, repeated 3 times, then high speed for 5 s.
   **Stop** ends the wash at any point.
7. Pump on, door stays locked. **WLLow** – everything off, LD2 off. Press B1 to arm again.
8. **Fault:** if WLH, TempR or WLLow is not pressed within 30 s of its step, valve, heater, pump and motor
   switch off, only the door lock stays on, and LD2 blinks fast. Press B1 to return to idle.

## Changing parameters

| Change | Edit in `main.c` |
|---|---|
| Wash / pause / rinse time (max 6.5 s each) | `LEFT_WASH_TIME_MS`, `RIGHT_WASH_TIME_MS`, `WASH_PAUSE_TIME_MS`, `RINSE_TIME_MS` |
| Number of iterations | `WASH_CYCLE_COUNT` |
| PWM period / duty | `WASH_PWM_PERIOD_US`, `*_DUTY_PERCENT`, `RINSE_*` |
| Fault timeout (multiple of 5000) | `FAULT_TIMEOUT_MS` |
| Debounce | `BUTTON_DEBOUNCE_MS` |
| Watchdog / terminal / LD2 | `USE_WATCHDOG`, `USE_LOG`, `USE_LD2_STATUS` = 0 |
