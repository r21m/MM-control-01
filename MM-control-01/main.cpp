//! @file


#include "main.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include "shr16.h"
#include "adc.h"
#include "uart.h"
#include "spi.h"
#include "tmc2130.h"
#include "abtn3.h"
#include "mmctl.h"
#include "stepper.h"
#include "Buttons.h"
#include <avr/wdt.h>
#include "permanent_storage.h"
#include "version.h"
#include "config.h"
#include "motion.h"


uint8_t tmc2130_mode = NORMAL_MODE;

#if (UART_COM == 0)
FILE* uart_com = uart0io;
#elif (UART_COM == 1)
FILE* uart_com = uart1io;
#endif //(UART_COM == 0)


#ifdef PRUSA_FILAMENT_SENSOR
#include "pat9125.h"
PAT9125 PAT(PRUSA_FILAMENT_SENSOR_ADDRES);

int pat_x = 0;
int pat_y = 0;
int pat_x2 = 0;
int pat_y2 = 0;
int8_t pat_b = -1;
int8_t pat_s = -1;
bool pid_ok = false;
//mm output
float pat_x_mm = 0;
float pat_y_mm = 0;
//corr
#endif

namespace
{
//! @brief State
enum class S
{
  Idle,
  Setup,
  Printing,
  SignalFilament,
  Wait,
  WaitOk,
};
}

//! @brief Main MMU state
//!
//! @startuml
//!
//! title MMU Main State Diagram
//!
//! state Any {
//!   state Idle : Manual extruder selector
//!   state Setup
//!   state Printing
//!   state SignalFilament
//!
//!   [*] --> Idle : !MiddleButton
//!   [*] --> Setup : MiddleButton
//!   Any --> Printing : T<nr> || Eject
//!   Any --> Idle : Unload || RecoverEject
//!   Any --> SignalFilament : Load && filamentLoaded
//!   Any --> Wait : W0
//!   Setup --> Idle
//!   Wait --> Idle : RightButton
//!   WaitOk --> Idle : RightButton
//!   Wait --> WaitOk : MiddleButton && mmctl_IsOk
//!   WaitOk --> Wait : MiddleButton && !mmctl_IsOk
//! }
//! @enduml
static S state;

static void process_commands(FILE* inout);

static void led_blink(int _no)
{
  shr16_set_led(1 << 2 * _no);
  delay(40);
  shr16_set_led(0x000);
  delay(20);
  shr16_set_led(1 << 2 * _no);
  delay(40);

  shr16_set_led(0x000);
  delay(10);
}

//! @brief signal filament presence
//!
//! non-blocking
//! LED indication of states
//!
//! RG | RG | RG | RG | RG | meaning
//! -- | -- | -- | -- | -- | ------------------------
//! b0 | b0 | b0 | b0 | b0 | Error, filament detected, still present
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking
static void signal_filament_present()
{
  shr16_set_led(0x2aa);
  delay(300);
  shr16_set_led(0x000);
  delay(300);
}

void signal_load_failure()
{
  shr16_set_led(0x000);
  delay(800);
  shr16_set_led(2 << 2 * (4 - active_extruder));
  delay(800);
}

void signal_ok_after_load_failure()
{
  shr16_set_led(0x000);
  delay(800);
  shr16_set_led(1 << 2 * (4 - active_extruder));
  delay(100);
  shr16_set_led(2 << 2 * (4 - active_extruder));
  delay(100);
  delay(800);
}

//! @brief Signal filament presence
//!
//! @retval true still present
//! @retval false not present any more
bool filament_presence_signaler()
{
  if (digitalRead(A1) != 1)
  {
    signal_filament_present();
    return true;
  }
  else
  {
    isFilamentLoaded = false;
    return false;
  }
}



//! @brief Check, if filament is not present in FINDA
//!
//! blocks, until filament is not removed and button pushed
//!
//! button | action
//! ------ | ------
//! right  | continue after error
//!
//! LED indication of states
//!
//! RG | RG | RG | RG | RG | meaning
//! -- | -- | -- | -- | -- | ------------------------
//! b0 | b0 | b0 | b0 | b0 | Error, filament detected, still present
//! 0b | 0b | 0b | 0b | 0b | Error, filament detected, no longer present, continue by right button click
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking

void check_filament_not_present()
{
  while (digitalRead(A1) != 1)
  {
    while (Btn::right != buttonClicked())
    {
      if (digitalRead(A1) != 1)
      {
        signal_filament_present();
      }
      else
      {
        shr16_set_led(0x155);
        delay(300);
        shr16_set_led(0x000);
        delay(300);
      }
    }
  }
}

static void signal_drive_error()
{
  shr16_set_led(0x3ff);
  delay(300);
  shr16_set_led(0x000);
  delay(300);
}

void drive_error()
{
  for (uint8_t i = 0; i < 3; ++i)
  {
    signal_drive_error();
  }
  DriveError::increment();
}

//! @brief Unrecoverable hardware fault
//!
//! Stay in infinite loop and blink.
//!
//! LED indication of states
//!
//! RG | RG | RG | RG | RG
//! -- | -- | -- | -- | --
//! bb | bb | bb | bb | bb
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking
void unrecoverable_error()
{
  while (1)
  {
    signal_drive_error();
  }
}

//! @brief Initialization after reset
//!
//! button | action
//! ------ | ------
//! middle | enter setup
//! right  | continue after error
//!
//! LED indication of states
//!
//! RG | RG | RG | RG | RG | meaning
//! -- | -- | -- | -- | -- | ------------------------
//! 00 | 00 | 00 | 00 | 0b | Shift register initialized
//! 00 | 00 | 00 | 0b | 00 | uart initialized
//! 00 | 00 | 0b | 00 | 00 | spi initialized
//! 00 | 0b | 00 | 00 | 00 | tmc2130 initialized
//! 0b | 00 | 00 | 00 | 00 | A/D converter initialized
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking
void setup()
{
  permanentStorageInit();
  shr16_init(); // shift register
  led_blink(0);

  uart0_init(); //uart0
  uart1_init(); //uart1
  led_blink(1);


#if (UART_STD == 0)
  stdin = uart0io; // stdin = uart0
  stdout = uart0io; // stdout = uart0
#elif (UART_STD == 1)
  stdin = uart1io; // stdin = uart1
  stdout = uart1io; // stdout = uart1
#endif //(UART_STD == 1)

  fprintf_P(uart_com, PSTR("start\n")); //startup message

#ifdef PRUSA_FILAMENT_SENSOR
  PAT.pat9125_init();
  PAT.pat9125_set_res(PRUSA_FILAMENT_SENSOR_XRES, PRUSA_FILAMENT_SENSOR_YRES);
  PAT.pat9125_update();
#endif

  fprintf_P(uart_com, PSTR("start\n")); //startup message
  
    spi_init();
    led_blink(2);
    led_blink(3);

    adc_init(); // ADC
    led_blink(4);

    shr16_set_ena(7);
    shr16_set_led(0x000);

    // check if to goto the settings menu
    if (buttonClicked() == Btn::middle)
    {
      state = S::Setup;
    }

    tmc2130_init(HOMING_MODE);
    tmc2130_read_gstat(); //consume reset after power up
    uint8_t filament;
    if (FilamentLoaded::get(filament))
    {
      motion_set_idler(filament);
    }

    if (digitalRead(A1) != 1) isFilamentLoaded = true;

}

//! @brief Select filament menu
//!
//! Select filament by pushing left and right button, park position can be also selected.
//!
//! button | action
//! ------ | ------
//! left   | select previous filament
//! right  | select next filament
//!
//! LED indication of states
//!
//! RG | RG | RG | RG | RG | meaning
//! -- | -- | -- | -- | -- | ------------------------
//! 01 | 00 | 00 | 00 | 00 | filament 1
//! 00 | 01 | 00 | 00 | 00 | filament 2
//! 00 | 00 | 01 | 00 | 00 | filament 3
//! 00 | 00 | 00 | 01 | 00 | filament 4
//! 00 | 00 | 00 | 00 | 01 | filament 5
//! 00 | 00 | 00 | 00 | bb | park position
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking
void manual_extruder_selector()
{
  shr16_set_led(1 << 2 * (4 - active_extruder));

  if ((Btn::left | Btn::right) & buttonClicked())
  {
    delay(500);

    switch (buttonClicked())
    {
      case Btn::right:
        if (active_extruder < 5)
        {
          select_extruder(active_extruder + 1);
        }
        break;
      case Btn::left:
        if (active_extruder > 0) select_extruder(active_extruder - 1);
        break;

      default:
        break;
    }
    delay(500);
  }

  if (active_extruder == 5)
  {
    shr16_set_led(2 << 2 * 0);
    delay(50);
    shr16_set_led(1 << 2 * 0);
    delay(50);
  }
}

#ifdef PRUSA_FILAMENT_SENSOR
void update_sensor_data() {
  PAT.pat9125_update();
  delay(2);
#ifdef PRUSA_FILAMENT_SENSOR_INVERSE
  pat_x = -PAT.pat9125_x;
  pat_y = -PAT.pat9125_y;
#else
  pat_x = PAT.pat9125_x;
  pat_y = PAT.pat9125_y;
#endif
  pat_b = PAT.pat9125_b;
  pat_s = PAT.pat9125_s;
}

void update_sensor_data_mm() {
  PAT.pat9125_update();
  delay(2);
#ifdef PRUSA_FILAMENT_SENSOR_INVERSE
  pat_x_mm = -(PAT.pat9125_x * PRUSA_FILAMENT_SENSOR_MM_KX);
  pat_y_mm = -(PAT.pat9125_y * PRUSA_FILAMENT_SENSOR_MM_KY);
#else
  pat_x_mm = (PAT.pat9125_x * PRUSA_FILAMENT_SENSOR_MM_KX);
  pat_y_mm = (PAT.pat9125_y * PRUSA_FILAMENT_SENSOR_MM_KY);
#endif
  pat_b = PAT.pat9125_b;
  pat_s = PAT.pat9125_s;

}

void sensor_reset() {
  PAT.pat9125_reset();
  pat_x = 0;
  pat_y = 0;
  pat_b = 0;
  pat_s = 0;
  pat_x_mm = 0;
  pat_y_mm = 0;
}


#endif
//! @brief main loop
//!
//! It is possible to manually select filament and feed it when S::Idle.
//!
//! button | action
//! ------ | ------
//! middle | feed filament
//!
//! @copydoc manual_extruder_selector()
void loop()
{
  process_commands(uart_com);

  switch (state)
  {
    case S::Setup:
      if (!setupMenu()) state = S::Idle;
      break;
    case S::Printing:
      break;
    case S::SignalFilament:
      if (!filament_presence_signaler()) state = S::Idle;
      break;
    case S::Idle:
      manual_extruder_selector();
      if (Btn::middle == buttonClicked() && active_extruder < 5)
      {
        shr16_set_led(2 << 2 * (4 - active_extruder));
        delay(500);
        if (Btn::middle == buttonClicked())
        {
          motion_set_idler_selector(active_extruder);
          feed_filament();
        }
      }
      break;
    case S::Wait:
      signal_load_failure();
      switch (buttonClicked())
      {
        case Btn::middle:
          if (mmctl_IsOk()) state = S::WaitOk;
          break;
        case Btn::right:
          state = S::Idle;
          fprintf_P(uart_com, PSTR("ok\n"));
          break;
        default:
          break;
      }
      break;
    case S::WaitOk:
      signal_ok_after_load_failure();
      switch (buttonClicked())
      {
        case Btn::middle:
          if (!mmctl_IsOk()) state = S::Wait;
          break;
        case Btn::right:
          state = S::Idle;
          fprintf_P(uart_com, PSTR("ok\n"));
          break;
        default:
          break;
      }
      break;
  }
}

//! @brief receive and process commands from serial line
//! @par inout FILE* struct connected to serial line to be used
//!
//! All commands have syntax in form of one letter integer number.
void process_commands(FILE* inout)
{
  static char line[32];
  static int count = 0;
  int c = -1;
  if (count < 32)
  {
    if ((c = getc(inout)) >= 0)
    {
      if (c == '\r') c = 0;
      if (c == '\n') c = 0;
      line[count++] = c;
    }
  }
  else
  {
    count = 0;
    //overflow
  }
  int value = 0;
  int value0 = 0;

  if ((count > 0) && (c == 0))
  {
    //line received
    //printf_P(PSTR("line received: '%s' %d\n"), line, count);
    count = 0;
    //! T<nr.> change to filament <nr.>
    if (sscanf_P(line, PSTR("T%d"), &value) > 0)
    {
      if ((value >= 0) && (value < EXTRUDERS))
      {
        state = S::Printing;
        switch_extruder_withSensor(value);
        fprintf_P(inout, PSTR("ok\n"));
      }
    }
    //! L<nr.> Load filament <nr.>
    else if (sscanf_P(line, PSTR("L%d"), &value) > 0)
    {
      if ((value >= 0) && (value < EXTRUDERS))
      {
        if (isFilamentLoaded) state = S::SignalFilament;
        else
        {
          select_extruder(value);
          feed_filament();
        }
        fprintf_P(inout, PSTR("ok\n"));
      }
    }
    else if (sscanf_P(line, PSTR("M%d"), &value) > 0)
    {
      //! M0 set to normal mode
      //!@n M1 set to stealth mode
      switch (value) {
        case 0: tmc2130_mode = NORMAL_MODE; break;
        case 1: tmc2130_mode = STEALTH_MODE; break;
        default: return;
      }

      //init all axes
      tmc2130_init(tmc2130_mode);
      fprintf_P(inout, PSTR("ok\n"));
    }
    //! U<nr.> Unload filament. <nr.> is ignored but mandatory.
    else if (sscanf_P(line, PSTR("U%d"), &value) > 0)
    {
      unload_filament_withSensor();
      fprintf_P(inout, PSTR("ok\n"));
      state = S::Idle;
    }
    else if (sscanf_P(line, PSTR("X%d"), &value) > 0)
    {
      if (value == 0) //! X0 MMU reset
        wdt_enable(WDTO_15MS);
    }
    else if (sscanf_P(line, PSTR("P%d"), &value) > 0)
    {
      if (value == 0) //! P0 Read finda
        fprintf_P(inout, PSTR("%dok\n"), !digitalRead(A1));
    }
#ifdef PRUSA_FILAMENT_SENSOR
    else if (sscanf_P(line, PSTR("K%d"), &value) > 0)
    {
      if (value == 0) { //! K0 Read prusa filament sensor

        pid_ok = PAT.pat9125_read_pid();
        fprintf_P(inout, PSTR("%dok\n"), pid_ok);
      }
      if (value == 1) { //! K1 update and read all data, X,Y,B,S,F
        update_sensor_data();
        fprintf_P(inout, PSTR("%i,"), pat_x);
        fprintf_P(inout, PSTR("%i,"), pat_y);
        fprintf_P(inout, PSTR("%i,"), pat_b);
        fprintf_P(inout, PSTR("%i,"), pat_s);
        fprintf_P(inout, PSTR("%d,"), !digitalRead(A1));
        fprintf_P(inout, PSTR("ok\n"));
      }
      if (value == 2) { //! K1 update and read all data, XY mm output
        update_sensor_data_mm();
        char pat_x_mm_st[16];
        char pat_y_mm_st[16];

        dtostrf(pat_x_mm, 12, 1, pat_x_mm_st);
        dtostrf(pat_y_mm, 12, 1, pat_y_mm_st);

        fprintf_P(inout, PSTR("%s,"), pat_x_mm_st);
        fprintf_P(inout, PSTR("%s,"), pat_y_mm_st);
        fprintf_P(inout, PSTR("%i,"), pat_b);
        fprintf_P(inout, PSTR("%i,"), pat_s);
        fprintf_P(inout, PSTR("%d,"), !digitalRead(A1));
        fprintf_P(inout, PSTR("ok\n"));
      }
           if (value == 3) { //K 3 zero
             sensor_reset();
             delay(2);
           fprintf_P(inout, PSTR("ok\n"));
      }

    }

#endif
    else if (sscanf_P(line, PSTR("S%d"), &value) > 0)
    {
      if (value == 0) //! S0 return ok
        fprintf_P(inout, PSTR("ok\n"));
      else if (value == 1) //! S1 Read version
        fprintf_P(inout, PSTR("%dok\n"), fw_version);
      else if (value == 2) //! S2 Read build nr.
        fprintf_P(inout, PSTR("%dok\n"), fw_buildnr);
      else if (value == 3) //! S3 Read drive errors
        fprintf_P(inout, PSTR("%dok\n"), DriveError::get());
    }
    //! F<nr.> <type> filament type. <nr.> filament number, <type> 0, 1 or 2. Does nothing.
    else if (sscanf_P(line, PSTR("F%d %d"), &value, &value0) > 0)
    {
      if (((value >= 0) && (value < EXTRUDERS)) &&
          ((value0 >= 0) && (value0 <= 2)))
      {
        filament_type[value] = value0;
        fprintf_P(inout, PSTR("ok\n"));
      }
    }
    else if (sscanf_P(line, PSTR("C%d"), &value) > 0)
    {
      if (value == 0) //! C0 continue loading current filament (used after T-code).
      {
        load_filament_inPrinter();
        fprintf_P(inout, PSTR("ok\n"));
      }
    }
    else if (sscanf_P(line, PSTR("E%d"), &value) > 0)
    {
      if ((value >= 0) && (value < EXTRUDERS)) //! E<nr.> eject filament
      {
        eject_filament(value);
        fprintf_P(inout, PSTR("ok\n"));
        state = S::Printing;
      }
    }
    else if (sscanf_P(line, PSTR("R%d"), &value) > 0)
    {
      if (value == 0) //! R0 recover after eject filament
      {
        recover_after_eject();
        fprintf_P(inout, PSTR("ok\n"));
        state = S::Idle;
      }
    }
    else if (sscanf_P(line, PSTR("W%d"), &value) > 0)
    {
      if (value == 0) //! W0 Wait for user click
      {
        state = S::Wait;
      }
    }
  }
  else
  { //nothing received
  }
}

