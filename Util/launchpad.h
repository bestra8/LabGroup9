/**
 * launchpad.h
 *
 * This file declares a set of functions for programming the TI Tiva C LaunchPad.
 *
 * Author: Zhao Zhang
 * First created: Summer 2014
 * Last revised: Spring 2022
 *
 *
 * Copyright (c) 2016, Zhao Zhang
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
 */

#ifndef LAUNCHPAD_H_
#define LAUNCHPAD_H_

#include <stdint.h>
#include <stdbool.h>
#include <driverlib/gpio.h>


// Initialize the most fundamental part of the LaunchPad
void LpInit();

/*****************************************************************************
 * Time functions
 ****************************************************************************/

// Get sysTime, which is the system time in microseconds
uint32_t SysTimeGet();

// Check if sysTime has reached a give time
bool SysTimeReached(uint32_t time);

// Wait for sysTime to advance for a given number of microseconds
void SysTimeWait(uint32_t delay);

// Wait for a given number of milliseconds, using loop waiting
void WaitMs(uint32_t timeMs);

// Wait for a given number of microseconds, using loop waiting
void WaitUs(uint32_t timeUs);

/*****************************************************************************
 * Schedule functions
 ****************************************************************************/

// Schedule a callback event at a given time
void ScheduleCallback(void (*callback)(uint32_t time), uint32_t time);

// Execute all callback events; return true if at least one event is ececuted.
// Note: The function may not return if callback event chain never ends.
bool ScheduleExecute();

// Execute callback events for a certain duration in milliseconds
void ScheduleExecuteAndWait(uint32_t delay);

/*****************************************************************************
 * LEDs functions
 *
 * There are three LEDs: red, green, and blue.
 ****************************************************************************/

// Initialize the LEDs
void LedInit();

// Turn on/off the LEDs individually. There are three LEDs of color red, blue, or green.
void LedTurnOnOff(bool red, bool blue, bool green);

/****************************************************************************
 * Push button functions
 ****************************************************************************/

// Initialize the push button
void PbInit();

// Read either push button SW1 or SW2, return 1 if SW1 is pushed, 2 if SW2 is pushed,
// and 0 otherwise. This function is non-blocking, and there is no software debouncing.
int PbRead();

/****************************************************************************
 * UART functions
 ****************************************************************************/

// Enable UART0 with baud rate of 115,200 bps, and 8-N-1 frame
void UartInit();

// Send a character
void UartPutChar(char ch);

// Receive a character
char UartGetChar();

// Send a string
int UartPutString(char *buffer);

// Print a string through UART0 using an internal buffer of 80 characters.
// Return the number of characters printed.
int uprintf(char* fmt, ...);

#endif  // LAUNCHPAD_H_
