/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : EG7014 Assignment 1 - Reliable washing-machine controller
  *                   NUCLEO-F401RE (STM32F401RE), GRAFCET with set/reset
  *                   equations, EXTI inputs, one-shot TIM3, PWM motor LED (TIM2),
  *                   SLEEP mode between events.
  *
  * Operation:  press blue B1 (arms the controller, LD2 turns on), then
  *             Start -> WLH -> TempR -> (wash runs by itself, or Stop) -> WLLow.
  *             If a sensor button is not pressed within FAULT_TIMEOUT_MS in
  *             Fill, Heat or Drain, the controller enters the FAULT step:
  *             valve, heater, pump and motor off, door lock stays on, LD2
  *             blinks fast. Press B1 to return to idle (everything off).
  ******************************************************************************
  */
/* USER CODE END Header */

/*
 * HOW THE PROGRAM WORKS (read this first)
 * ---------------------------------------
 * 1. Every push button is wired to its own EXTI (external interrupt) pin.
 *    A press triggers HAL_GPIO_EXTI_Callback(): it only RECORDS the event
 *    (one bit in input_events) and sets grafcet_request. No decisions there.
 * 2. TIM3 is a one-shot timer. When its time is up, HAL_TIM_PeriodElapsedCallback()
 *    sets timer_event and grafcet_request (the "Timer" transition condition).
 * 3. main() loops: if grafcet_request is set, it runs GRAFCET() ONCE, then clears
 *    the flag and puts the CPU to SLEEP (WFI) until the next interrupt.
 * 4. GRAFCET() reads the events, evaluates the set/reset equations of the main
 *    GRAFCET (x0..x5) and of the wash sub-GRAFCET (W0..W5), then writes the outputs.
 * 5. Outputs are a pure function of the active step (see UpdateMainOutputs()).
 *
 * Main steps: x0 Idle, x1 Fill, x2 Heat, x3 Wash (macro-step), x4 Drain, x5 Fault.
 */

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Build options                                                             */
/* ------------------------------------------------------------------------- */
/* Switches: set to 0 to remove a feature without touching the rest of the code */
#define USE_WATCHDOG   1   /* 1 = independent watchdog (2 s) refreshed in main loop */
#define USE_LOG        1   /* 1 = status text on ST-LINK virtual COM port (115200)  */
#define USE_LD2_STATUS 1   /* 1 = LD2 steady = armed, fast blink = fault             */

/* ------------------------------------------------------------------------- */
/* Pin map (unchanged wiring)                                                */
/* ------------------------------------------------------------------------- */
/* NUCLEO-F401RE blue user button and green LED */
#ifndef B1_Pin
#define B1_Pin            GPIO_PIN_13
#endif
#ifndef B1_GPIO_Port
#define B1_GPIO_Port      GPIOC
#endif
#ifndef LD2_Pin
#define LD2_Pin           GPIO_PIN_5
#endif
#ifndef LD2_GPIO_Port
#define LD2_GPIO_Port     GPIOA
#endif

/* Inputs (push buttons to 3.3 V, internal pull-down, rising-edge EXTI) */
#define START_PIN         GPIO_PIN_0    /* PA0  EXTI0  */
#define START_PORT        GPIOA
#define WLH_PIN           GPIO_PIN_1    /* PA1  EXTI1  : water level high  */
#define WLH_PORT          GPIOA
#define TEMP_PIN          GPIO_PIN_4    /* PA4  EXTI4  : temperature reached */
#define TEMP_PORT         GPIOA
#define WLLOW_PIN         GPIO_PIN_7    /* PC7  EXTI7  : water level low   */
#define WLLOW_PORT        GPIOC
#define STOP_PIN          GPIO_PIN_6    /* PB6  EXTI6  : Stop / end of wash */
#define STOP_PORT         GPIOB

/* Outputs (LEDs with series resistors) */
#define VALVE_PIN         GPIO_PIN_8    /* PA8  */
#define VALVE_PORT        GPIOA
#define HEATER_PIN        GPIO_PIN_10   /* PA10 */
#define HEATER_PORT       GPIOA
#define DOOR_PIN          GPIO_PIN_4    /* PB4  */
#define DOOR_PORT         GPIOB
#define PUMP_PIN          GPIO_PIN_5    /* PB5  */
#define PUMP_PORT         GPIOB
/* Motor LED: PB10 = TIM2_CH3 (PWM), configured in HAL_TIM_MspPostInit() */

/* ------------------------------------------------------------------------- */
/* Parameters (change here only)                                             */
/* A request such as "make the pause 1.5 s" is a one-line change below.      */
/* ------------------------------------------------------------------------- */
#define WASH_CYCLE_COUNT       3U       /* left/right iterations            */
#define LEFT_WASH_TIME_MS      2000U
#define RIGHT_WASH_TIME_MS     2000U
#define WASH_PAUSE_TIME_MS     1000U
#define RINSE_TIME_MS          5000U

/* Motor LED coding (brief): left = 90 % duty, right = 10 % duty, both with a
   0.5 s period; high speed = 0.1 s period (faster blinking) with 50 % duty.  */
#define WASH_PWM_PERIOD_US     500000U  /* 0.5 s period for washing         */
#define LEFT_DUTY_PERCENT      90U
#define RIGHT_DUTY_PERCENT     10U
#define RINSE_PWM_PERIOD_US    100000U  /* 0.1 s period for high speed      */
#define RINSE_DUTY_PERCENT     50U

#define BUTTON_DEBOUNCE_MS     50U

/* Supervision of Fill, Heat and Drain: a sensor must answer within this time */
#define FAULT_TIMEOUT_MS       30000U
#define TIMEOUT_SLICE_MS       5000U    /* TIM3 counts the timeout in slices  */
#define FAULT_BLINK_MS         100U     /* LD2 half-period in the fault step  */

/* TIM3: 84 MHz / (8399 + 1) = 10 kHz -> 0.1 ms per tick, 16-bit counter
   => longest interval = 65536 x 0.1 ms = 6.55 s. The 30 s timeout is therefore
   counted as 6 slices of 5 s (see GRAFCET(), timeout_slices).               */
#define TIM3_TICKS_PER_MS      10U
#define TIM3_MAX_TICKS         65536U

/* Compile-time checks: the build fails if a time is too long for the 16-bit TIM3. */
_Static_assert((LEFT_WASH_TIME_MS  * TIM3_TICKS_PER_MS) <= TIM3_MAX_TICKS, "left time too long for TIM3");
_Static_assert((RIGHT_WASH_TIME_MS * TIM3_TICKS_PER_MS) <= TIM3_MAX_TICKS, "right time too long for TIM3");
_Static_assert((WASH_PAUSE_TIME_MS * TIM3_TICKS_PER_MS) <= TIM3_MAX_TICKS, "pause time too long for TIM3");
_Static_assert((RINSE_TIME_MS      * TIM3_TICKS_PER_MS) <= TIM3_MAX_TICKS, "rinse time too long for TIM3");
_Static_assert((TIMEOUT_SLICE_MS   * TIM3_TICKS_PER_MS) <= TIM3_MAX_TICKS, "timeout slice too long for TIM3");
_Static_assert((FAULT_TIMEOUT_MS % TIMEOUT_SLICE_MS) == 0U, "timeout must be a whole number of slices");

/* Event bits recorded by the EXTI callback: one bit per button, so several
   events can be stored in the single variable input_events.                 */
#define EVENT_START            (1UL << 0)
#define EVENT_WATER_HIGH       (1UL << 1)
#define EVENT_TEMP_REACHED     (1UL << 2)
#define EVENT_WATER_LOW        (1UL << 3)
#define EVENT_WASH_END         (1UL << 4)
#define EVENT_RESET            (1UL << 5)   /* B1 pressed while armed (clears a fault) */

/* ------------------------------------------------------------------------- */
/* Peripheral handles                                                        */
/* ------------------------------------------------------------------------- */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* ------------------------------------------------------------------------- */
/* GRAFCET state variables                                                   */
/* ------------------------------------------------------------------------- */
/* Main GRAFCET steps: exactly one is true at any time (x0 starts true).    */
static bool x0 = true;   /* Idle  */
static bool x1 = false;  /* Fill  */
static bool x2 = false;  /* Heat  */
static bool x3 = false;  /* Wash (macro-step) */
static bool x4 = false;  /* Drain */
static bool x5 = false;  /* Fault (timeout in x1, x2 or x4) */

/* Wash sub-GRAFCET steps */
static bool W0 = false;  /* Left wash  */
static bool W1 = false;  /* Pause      */
static bool W2 = false;  /* Right wash */
static bool W3 = false;  /* Pause      */
static bool W4 = false;  /* Rinse / high speed */
static bool W5 = false;  /* Wash sub-GRAFCET complete */

/* Set / reset variables: sxN = "step N becomes active", rxN = "step N ends".
   Step update rule used everywhere:   xN = sxN || (xN && !rxN)             */
static bool sx0, sx1, sx2, sx3, sx4, sx5;
static bool rx0, rx1, rx2, rx3, rx4, rx5;
static bool sW0, sW1, sW2, sW3, sW4, sW5;
static bool rW0, rW1, rW2, rW3, rW4, rW5;

/* Wash sub-GRAFCET tracking: wash_started = macro-step entered,
   wash_cycles = completed left/right cycles, wash_complete = W5 reached.    */
static bool    wash_started  = false;
static bool    wash_complete = false;
static uint8_t wash_cycles   = 0U;

/* Timeout supervision of x1, x2 and x4: counts down 5 s slices from 6 to 0 */
static uint8_t timeout_slices = 0U;

/* ------------------------------------------------------------------------- */
/* Variables shared with interrupts                                          */
/* ------------------------------------------------------------------------- */
/* 'volatile' because these are changed in interrupts and read in main():
   it stops the compiler from keeping an old copy in a register.             */
static volatile bool     controller_active = false; /* set by B1                  */
static volatile uint8_t  grafcet_request   = 0U;    /* "run GRAFCET once" flag    */
static volatile uint32_t input_events      = 0U;    /* one bit per process button */
static volatile uint8_t  timer_event       = 0U;    /* TIM3 interval elapsed      */

/* Table that maps each EXTI pin to its event bit (and debounce time stamp).
   The callback searches this table, so adding a button = adding one line.   */
typedef struct
{
  uint16_t pin;
  uint32_t event;
  uint32_t last_tick;
} InputMap_t;

/* Start value that makes the very first press pass the 50 ms debounce test */
#define DEBOUNCE_INIT  ((uint32_t)(0U - BUTTON_DEBOUNCE_MS))

static InputMap_t input_map[] =
{
  { START_PIN, EVENT_START,        DEBOUNCE_INIT },
  { WLH_PIN,   EVENT_WATER_HIGH,   DEBOUNCE_INIT },
  { TEMP_PIN,  EVENT_TEMP_REACHED, DEBOUNCE_INIT },
  { WLLOW_PIN, EVENT_WATER_LOW,    DEBOUNCE_INIT },
  { STOP_PIN,  EVENT_WASH_END,     DEBOUNCE_INIT }
};
#define INPUT_COUNT  (sizeof(input_map) / sizeof(input_map[0]))

/* ------------------------------------------------------------------------- */
/* Prototypes                                                                */
/* ------------------------------------------------------------------------- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
#if USE_WATCHDOG
static void MX_IWDG_Init(void);
#endif

static void GRAFCET(void);
static void WashSubGRAFCET(bool timer_elapsed);
static void ResetWashSubGRAFCET(void);
static void ArmWashTimer(uint32_t duration_ms);
static void StopTimer(void);
static void SetMotorPwm(uint32_t period_us, uint32_t duty_percent);
static void SetMotorLeft(void);
static void SetMotorRight(void);
static void SetMotorRinse(void);
static void SetMotorOff(void);
static void UpdateMainOutputs(void);
static void UpdateStatusLed(void);
static void WriteOutput(GPIO_TypeDef *port, uint16_t pin, bool on);
static void Log(const char *text);

/* ------------------------------------------------------------------------- */
/* Small helpers                                                             */
/* ------------------------------------------------------------------------- */
/* Status text for the PC terminal (ST-LINK virtual COM port). This is the only
   blocking call (<= 10 ms, only when a step changes). Never call it from an ISR. */
static void Log(const char *text)
{
#if USE_LOG
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 10U);
#else
  (void)text;
#endif
}

static void WriteOutput(GPIO_TypeDef *port, uint16_t pin, bool on)
{
  HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ------------------------------------------------------------------------- */
/* Motor indicator: one PWM channel (TIM2_CH3, 1 MHz tick -> 1 us resolution) */
/* ------------------------------------------------------------------------- */
/* Period = period_us, duty = duty_percent of the period.
   TIM2 counts in microseconds, so:  ARR = period - 1,  CCR3 = period x duty / 100.
   Example (left): ARR = 499999, CCR3 = 450000 -> LED on for 90 % of 0.5 s.
   The counter is restarted so each mode begins with a clean waveform.       */
static void SetMotorPwm(uint32_t period_us, uint32_t duty_percent)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim2, period_us - 1U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (period_us * duty_percent) / 100U);
}

static void SetMotorLeft(void)  { SetMotorPwm(WASH_PWM_PERIOD_US,  LEFT_DUTY_PERCENT);  }
static void SetMotorRight(void) { SetMotorPwm(WASH_PWM_PERIOD_US,  RIGHT_DUTY_PERCENT); }
static void SetMotorRinse(void) { SetMotorPwm(RINSE_PWM_PERIOD_US, RINSE_DUTY_PERCENT); }

/* Compare = 0 means the output is never high, so the LED is off (motor stopped) */
static void SetMotorOff(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
}

/* ------------------------------------------------------------------------- */
/* One-shot interval timer (TIM3)                                            */
/* ------------------------------------------------------------------------- */
/* Start TIM3 for duration_ms:  ARR = duration_ms x 10 - 1  (0.1 ms per tick).
   The timer interrupt later calls HAL_TIM_PeriodElapsedCallback().          */
static void ArmWashTimer(uint32_t duration_ms)
{
  uint32_t timer_ticks = duration_ms * TIM3_TICKS_PER_MS;

  if (timer_ticks < 1U)
  {
    timer_ticks = 1U;
  }
  if (timer_ticks > TIM3_MAX_TICKS)
  {
    timer_ticks = TIM3_MAX_TICKS;
  }

  (void)HAL_TIM_Base_Stop_IT(&htim3);

  /* An old TIM3 event must never be mistaken for the new interval */
  __disable_irq();
  timer_event = 0U;
  __enable_irq();

  __HAL_TIM_SET_AUTORELOAD(&htim3, timer_ticks - 1U);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);   /* no false first event */

  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Stop TIM3 (saves power when no time is being counted) and discard any
   event it raised just before stopping.                                     */
static void StopTimer(void)
{
  (void)HAL_TIM_Base_Stop_IT(&htim3);

  __disable_irq();
  timer_event = 0U;
  __enable_irq();
}

/* ------------------------------------------------------------------------- */
/* Wash macro-step (sub-GRAFCET)                                             */
/* Left, pause, right, pause - repeated WASH_CYCLE_COUNT times - then rinse.  */
/* ------------------------------------------------------------------------- */
static void WashSubGRAFCET(bool timer_elapsed)
{
  bool more_cycles;

  /* Entering the macro-step: W0 (left) becomes active and its timer starts */
  if (!wash_started)
  {
    wash_started  = true;
    wash_complete = false;
    wash_cycles   = 0U;

    W0 = true;
    W1 = false;
    W2 = false;
    W3 = false;
    W4 = false;
    W5 = false;

    SetMotorLeft();
    ArmWashTimer(LEFT_WASH_TIME_MS);
    Log("  W0 LEFT  (0.5 s, 90 %)\r\n");
    return;
  }

  /* Every sub-GRAFCET transition is "step active AND Timer": if TIM3 has not
     elapsed, nothing can change, so leave at once. */
  if (!timer_elapsed)
  {
    return;
  }

  /* End of W3 completes one left/right cycle: count it, then decide whether
     another cycle follows (W3 -> W0) or the high-speed step (W3 -> W4). */
  if (W3)
  {
    wash_cycles++;
  }
  more_cycles = (wash_cycles < WASH_CYCLE_COUNT);

  /* Set equations */
  sW0 = W3 && more_cycles;    /* W3 . Timer . more cycles  */
  sW1 = W0;                   /* W0 . Timer                */
  sW2 = W1;                   /* W1 . Timer                */
  sW3 = W2;                   /* W2 . Timer                */
  sW4 = W3 && !more_cycles;   /* W3 . Timer . last cycle   */
  sW5 = W4;                   /* W4 . Timer                */

  /* Reset equations: the successor resets its predecessor */
  rW0 = sW1;
  rW1 = sW2;
  rW2 = sW3;
  rW3 = sW0 || sW4;
  rW4 = sW5;
  rW5 = false;                /* W5 is cleared by ResetWashSubGRAFCET() */

  W0 = sW0 || (W0 && !rW0);
  W1 = sW1 || (W1 && !rW1);
  W2 = sW2 || (W2 && !rW2);
  W3 = sW3 || (W3 && !rW3);
  W4 = sW4 || (W4 && !rW4);
  W5 = sW5 || (W5 && !rW5);

  /* Sub-step actions: set the motor mode and arm the timer for the new step */
  if (sW0)
  {
    SetMotorLeft();
    ArmWashTimer(LEFT_WASH_TIME_MS);
    Log("  W0 LEFT  (0.5 s, 90 %)\r\n");
  }
  else if (sW1)
  {
    SetMotorOff();
    ArmWashTimer(WASH_PAUSE_TIME_MS);
    Log("  W1 PAUSE\r\n");
  }
  else if (sW2)
  {
    SetMotorRight();
    ArmWashTimer(RIGHT_WASH_TIME_MS);
    Log("  W2 RIGHT (0.5 s, 10 %)\r\n");
  }
  else if (sW3)
  {
    SetMotorOff();
    ArmWashTimer(WASH_PAUSE_TIME_MS);
    Log("  W3 PAUSE\r\n");
  }
  else if (sW4)
  {
    SetMotorRinse();
    ArmWashTimer(RINSE_TIME_MS);
    Log("  W4 HIGH SPEED (0.1 s, 50 %)\r\n");
  }
  else if (sW5)
  {
    SetMotorOff();
    wash_complete = true;
    Log("  W5 wash complete\r\n");
  }
}

/* Leaving x3 (normally or by Stop): stop the timer, clear the sub-GRAFCET, motor off.
   This is why Stop can end the wash at ANY point without leaving the motor on. */
static void ResetWashSubGRAFCET(void)
{
  (void)HAL_TIM_Base_Stop_IT(&htim3);

  W0 = false;
  W1 = false;
  W2 = false;
  W3 = false;
  W4 = false;
  W5 = false;

  wash_started  = false;
  wash_complete = false;
  wash_cycles   = 0U;

  SetMotorOff();

  /* Drop a TIM3 event that may have fired just before the timer was stopped */
  __disable_irq();
  timer_event = 0U;
  __enable_irq();
}

/* ------------------------------------------------------------------------- */
/* Main GRAFCET outputs: pure function of the active step.                   */
/* Door lock is on in every step except Idle, so the door cannot be opened    */
/* while water, heat, motion or a fault is present.                           */
/* ------------------------------------------------------------------------- */
static void UpdateMainOutputs(void)
{
  bool V   = x1;                      /* water valve         */
  bool HTR = x2;                      /* heater              */
  bool LD  = x1 || x2 || x3 || x4 || x5;  /* door lock (also held in the fault step) */
  bool P   = x4;                      /* pump                */

  WriteOutput(VALVE_PORT,  VALVE_PIN,  V);
  WriteOutput(HEATER_PORT, HEATER_PIN, HTR);
  WriteOutput(DOOR_PORT,   DOOR_PIN,   LD);
  WriteOutput(PUMP_PORT,   PUMP_PIN,   P);

  UpdateStatusLed();
}

/* On-board LD2: steady = armed with B1, fast blink = fault step */
static void UpdateStatusLed(void)
{
#if USE_LD2_STATUS
  bool on = controller_active;

  if (x5)
  {
    on = (((HAL_GetTick() / FAULT_BLINK_MS) & 1U) == 0U);
  }
  WriteOutput(LD2_GPIO_Port, LD2_Pin, on);
#endif
}

/* ------------------------------------------------------------------------- */
/* Main GRAFCET - executed once per request                                  */
/* Order inside: 1 read events  2 wash sub-GRAFCET  3 timeout supervision     */
/* 4 set/reset equations  5 step update  6 timers  7 outputs  8 log           */
/* ------------------------------------------------------------------------- */
static void GRAFCET(void)
{
  uint32_t events;
  bool elapsed;
  bool was_washing;
  bool aborted;
  bool Timeout = false;
  bool Start, WLH, TempR, WLLow, WshEnd, Reset;
  const char *fault_step = "";

  /* 1. Atomic read-and-clear (interrupts masked): an interrupt cannot change
        the variables halfway through. Events are valid for this execution only,
        so a button pressed in the wrong step is discarded, never stored. */
  __disable_irq();
  events      = input_events;
  input_events = 0U;
  elapsed     = (timer_event != 0U);
  timer_event = 0U;
  __enable_irq();

  was_washing = x3;

  /* 2. The wash sub-GRAFCET runs only while the macro-step x3 is active */
  if (was_washing)
  {
    WashSubGRAFCET(elapsed);
  }

  Start  = ((events & EVENT_START)        != 0U);
  WLH    = ((events & EVENT_WATER_HIGH)   != 0U);
  TempR  = ((events & EVENT_TEMP_REACHED) != 0U);
  WLLow  = ((events & EVENT_WATER_LOW)    != 0U);
  WshEnd = ((events & EVENT_WASH_END)     != 0U);
  Reset  = ((events & EVENT_RESET)        != 0U);

  /* 3. Supervision of Fill, Heat and Drain: each TIM3 event is one 5 s slice.
        After 6 slices (30 s) without the sensor, Timeout becomes true. */
  if (elapsed && (x1 || x2 || x4))
  {
    if (timeout_slices > 0U)
    {
      timeout_slices--;
    }

    if (timeout_slices == 0U)
    {
      Timeout = true;
    }
    else
    {
      ArmWashTimer(TIMEOUT_SLICE_MS);
    }
  }

  /* 4. Set equations (transition condition AND predecessor step) */
  sx0 = (x4 && WLLow) || (x5 && Reset);    /* x4 . WLLow  +  x5 . Reset        */
  sx1 = x0 && Start;                       /* x0 . Start                       */
  sx2 = x1 && WLH;                         /* x1 . WLH                         */
  sx3 = x2 && TempR;                       /* x2 . TempR                       */
  sx4 = x3 && (wash_complete || WshEnd);   /* x3 . (wash complete + Stop)      */
  /* Fault: timeout, unless the expected sensor answered in the same pass */
  sx5 = Timeout && ((x1 && !WLH) || (x2 && !TempR) || (x4 && !WLLow));

  /* Reset equations: the set signal of the successor resets the predecessor,
     so the same signal that starts a step ends the previous one (one-hot). */
  rx0 = sx1;
  rx1 = sx2 || sx5;
  rx2 = sx3 || sx5;
  rx3 = sx4;
  rx4 = sx0 || sx5;
  rx5 = sx0;

  aborted = sx4 && !wash_complete;         /* Stop pressed before the wash finished */
  if (sx5)
  {
    fault_step = x1 ? "FILL" : (x2 ? "HEAT" : "DRAIN");
  }

  /* 5. Step update: all steps change together, after all set/reset signals
        have been computed (the synchronous behaviour GRAFCET requires). */
  x0 = sx0 || (x0 && !rx0);
  x1 = sx1 || (x1 && !rx1);
  x2 = sx2 || (x2 && !rx2);
  x3 = sx3 || (x3 && !rx3);
  x4 = sx4 || (x4 && !rx4);
  x5 = sx5 || (x5 && !rx5);

  /* Macro-step enter / leave: start W0 when x3 becomes active, clean up when it ends */
  if (!was_washing && x3)
  {
    WashSubGRAFCET(false);
  }
  else if (was_washing && !x3)
  {
    ResetWashSubGRAFCET();
  }

  /* 6. Timers: a fresh 30 s timeout starts in every supervised step;
        in Fault and Idle the timer is stopped. */
  if (sx1 || sx2 || sx4)
  {
    timeout_slices = (uint8_t)(FAULT_TIMEOUT_MS / TIMEOUT_SLICE_MS);
    ArmWashTimer(TIMEOUT_SLICE_MS);
  }
  else if (sx5 || sx0)
  {
    StopTimer();
  }

  /* Back to idle: disarm, so B1 must be pressed again (stops accidental starts) */
  if (sx0)
  {
    __disable_irq();
    controller_active = false;
    input_events      = 0U;
    timer_event       = 0U;
    grafcet_request   = 0U;
    __enable_irq();
  }

  /* 7. Write all outputs from the new active step */
  UpdateMainOutputs();

  /* 8. Optional status text (only when a step has just changed) */
  if (sx1) { Log("x1 FILL : valve + door lock ON\r\n"); }
  if (sx2) { Log("x2 HEAT : valve OFF, heater ON\r\n"); }
  if (sx3) { Log("x3 WASH : heater OFF, wash started\r\n"); }
  if (sx4) { Log(aborted ? "x4 DRAIN: STOP pressed, wash aborted, pump ON\r\n"
                         : "x4 DRAIN: wash finished, pump ON\r\n"); }
  if (sx5) { Log("x5 FAULT: timeout in "); Log(fault_step);
             Log(" - valve, heater, pump OFF, door stays locked. Press B1.\r\n"); }
  if (sx0) { Log("x0 IDLE : all OFF, door released. Press B1 to arm.\r\n"); }
}

/* ------------------------------------------------------------------------- */
/* main                                                                      */
/* ------------------------------------------------------------------------- */
int main(void)
{
  HAL_Init();               /* HAL, SysTick (1 ms tick) */
  SystemClock_Config();     /* 84 MHz */

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* TIM2 channel 3 PWM on PB10 (motor LED) */
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  /* Known safe state at power-up: motor off, all LEDs off */
  SetMotorOff();
  UpdateMainOutputs();
  Log("\r\nWashing machine controller ready. Press blue B1 to arm.\r\n");

#if USE_WATCHDOG
  MX_IWDG_Init();   /* start last, so start-up time cannot trip it */
#endif

  while (1)
  {
#if USE_WATCHDOG
    /* If the program ever hangs, this line stops running and the watchdog
       resets the system into the all-off state. */
    IWDG->KR = 0xAAAAU;   /* refresh: SysTick wakes this loop every 1 ms */
#endif

    /* Run the GRAFCET once per request, then clear the flag */
    if (grafcet_request != 0U)
    {
      grafcet_request = 0U;
      GRAFCET();
    }

    if (x5)
    {
      UpdateStatusLed();   /* fast LD2 blink while in the fault step */
    }

    /*
     * Sleep until the next interrupt. The request flag is checked with
     * interrupts masked: an interrupt that arrives between the check and WFI
     * keeps WFI from sleeping, so no event can be missed.
     */
    __disable_irq();
    if (grafcet_request == 0U)
    {
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    }
    __enable_irq();
  }
}

/* ------------------------------------------------------------------------- */
/* Clock: HSI 16 MHz -> PLL -> 84 MHz SYSCLK, APB1 42 MHz (timers 84 MHz)    */
/* ------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = 16;
  RCC_OscInitStruct.PLL.PLLN            = 336;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ            = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ------------------------------------------------------------------------- */
/* TIM2: PWM motor LED. 84 MHz / 84 = 1 MHz -> 1 us per tick (32-bit)        */
/* ------------------------------------------------------------------------- */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};
  TIM_OC_InitTypeDef      sConfigOC          = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 83;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = WASH_PWM_PERIOD_US - 1U;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);   /* PB10 -> TIM2_CH3 */
}

/* ------------------------------------------------------------------------- */
/* TIM3: one-shot interval timer. 84 MHz / 8400 = 10 kHz -> 0.1 ms per tick  */
/* ------------------------------------------------------------------------- */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};

  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance               = TIM3;
  htim3.Init.Prescaler         = 8399;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 9;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* TIM3 update interrupt (priority below the buttons) */
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* ------------------------------------------------------------------------- */
/* USART2: ST-LINK virtual COM port (status messages)                        */
/* ------------------------------------------------------------------------- */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = 115200;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ------------------------------------------------------------------------- */
/* GPIO and EXTI                                                             */
/* ------------------------------------------------------------------------- */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Outputs start in the safe state (all off) */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin | VALVE_PIN | HEATER_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, DOOR_PIN | PUMP_PIN, GPIO_PIN_RESET);

  /* Blue user button B1: PC13, falling edge (external pull-up on the board) */
  GPIO_InitStruct.Pin  = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* Start PA0 (A0), WLH PA1 (A1), TempR PA4 (A2): button to 3.3 V, internal
     pull-down keeps the pin low when released, so a press = RISING edge */
  GPIO_InitStruct.Pin  = START_PIN | WLH_PIN | TEMP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* LD2, valve PA8, heater PA10 */
  GPIO_InitStruct.Pin   = LD2_Pin | VALVE_PIN | HEATER_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* WLLow PC7 */
  GPIO_InitStruct.Pin  = WLLOW_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(WLLOW_PORT, &GPIO_InitStruct);

  /* Door lock PB4, pump PB5 */
  GPIO_InitStruct.Pin   = DOOR_PIN | PUMP_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Stop / end-of-wash PB6 */
  GPIO_InitStruct.Pin  = STOP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(STOP_PORT, &GPIO_InitStruct);

  /* Enable the interrupt channels. Lines 5-9 share one vector (PC7 = line 7,
     PB6 = line 6) and lines 10-15 share another (PC13 = line 13), but the HAL
     tells the callback the exact pin. Priority 0 = highest (buttons). */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

#if USE_WATCHDOG
/* Independent watchdog (runs on its own clock, so it works even if the
 * main clock or the program fails): LSI 32 kHz / 64 = 500 Hz, reload 1000 -> 2 s timeout.
 * Register-level so that the HAL IWDG module does not need to be enabled.     */
static void MX_IWDG_Init(void)
{
  uint32_t t0;

  __HAL_DBGMCU_FREEZE_IWDG();       /* do not reset while stopped at a breakpoint */

  IWDG->KR  = 0xCCCCU;              /* start (also starts the LSI)  */
  IWDG->KR  = 0x5555U;              /* unlock PR and RLR            */
  IWDG->PR  = 4U;                   /* divider /64                  */
  IWDG->RLR = 1000U;

  t0 = HAL_GetTick();
  while (IWDG->SR != 0U)
  {
    if ((HAL_GetTick() - t0) > 100U)
    {
      Error_Handler();
    }
  }

  IWDG->KR = 0xAAAAU;               /* first refresh */
}
#endif

/* ------------------------------------------------------------------------- */
/* Interrupt callbacks - record events only, no decision logic               */
/* ------------------------------------------------------------------------- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t last_blue = DEBOUNCE_INIT;
  uint32_t now = HAL_GetTick();
  uint32_t i;

  /* Blue button: arms the controller while it is disarmed; once armed, a
     press is a RESET event (only the Fault step uses it). */
  if (GPIO_Pin == B1_Pin)
  {
    if ((now - last_blue) >= BUTTON_DEBOUNCE_MS)
    {
      last_blue = now;

      if (!controller_active)
      {
        input_events      = 0U;
        controller_active = true;
        grafcet_request   = 1U;   /* lets the main loop switch LD2 on */
      }
      else
      {
        input_events    |= EVENT_RESET;   /* only the fault step reacts to it */
        grafcet_request  = 1U;
      }
    }
    return;
  }

  /* Process buttons are ignored until B1 has armed the controller */
  if (!controller_active)
  {
    return;
  }

  /* Find the button in the table; accept it only if 50 ms have passed since
     its last accepted edge (software debounce), then record the event. */
  for (i = 0U; i < INPUT_COUNT; i++)
  {
    if (GPIO_Pin == input_map[i].pin)
    {
      if ((now - input_map[i].last_tick) >= BUTTON_DEBOUNCE_MS)
      {
        input_map[i].last_tick = now;
        input_events |= input_map[i].event;
        grafcet_request = 1U;
      }
      break;
    }
  }
}

/* TIM3 interval elapsed: stop the timer (one-shot) and raise the Timer condition */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    (void)HAL_TIM_Base_Stop_IT(&htim3);
    timer_event     = 1U;
    grafcet_request = 1U;
  }
}

/* Unrecoverable error: force every output off; the watchdog then restarts the system */
void Error_Handler(void)
{
  __disable_irq();

  HAL_GPIO_WritePin(VALVE_PORT,  VALVE_PIN,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HEATER_PORT, HEATER_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DOOR_PORT,   DOOR_PIN,   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PUMP_PORT,   PUMP_PIN,   GPIO_PIN_RESET);

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
