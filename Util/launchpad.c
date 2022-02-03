/**
 * launchpad.c
 *
 * This file includes miscellaneous functions for the TI LaunchPad.
 *
 * Zhao Zhang
 *
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <inc/hw_ints.h>
#include <inc/hw_types.h>
#include <driverlib/interrupt.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>

#include "launchpad.h"

// System tick clock frequency, which is the PIOSC in this library
#define SYSTICK_CLOCK_RATE 		16000000L

// maximum size for callback queue, see below
#define MAX_CALLBACK		    32

// The system clock rate in Hz. It will be configured to 50MHz
static uint32_t clockRate;

/***************************************************************************************
 * System time wait functions
 **************************************************************************************/

// System time in milliseconds
static volatile uint32_t sysTime = 0;

// System tick interrupt handler, for maintaining system time in microseconds
static void
SysTickIntHandler()
{
	sysTime++;
}

// Get sysTime
uint32_t											// return the current value of sysTime
SysTimeGet()
{
	return sysTime;
}

// Compare a given time with a system time, return 1 if the given time is reached, and 0 otherwise.
// It's assumed a program compares a desired time with sysTime frequently enough such that
// a signed 32-bit overflow won't happen.
bool												// return whether the given time is reached
SysTimeReached(uint32_t time)						// the time to be compared with sysTime
{
	int32_t diff = (int32_t) (sysTime - time);

	return (diff >= 0) ? true : false;
}

// Compare two times using sysTime as the reference point. This function handles
// 32-bit wrap-around. Return true if time1 <= time2
static inline bool
SysTimeLE(uint32_t time1, uint32_t time2)
{
	uint32_t t = sysTime;							// Get a local copy of sysTime

	int32_t t1 = (int32_t) (time1 - t);
	int32_t t2 = (int32_t) (time2 - t);

	return (t1 <= t2);
}

// Wait for sysTime for a delay.
// FIXME This function uses busy waiting which is not energy-efficient. It can utilize a CPU sleep
// mode to save energy.
void
SysTimeWait(uint32_t delay)							// delay in milliseconds
{
	uint32_t time = sysTime + delay;
	int32_t diff;

    do {
    	diff = (int32_t) (sysTime - time);
    } while (diff < 0);
}

// Wait for system time to reach a time point
void
SysTimeWaitAbsolute(uint32_t time)					// time point in milliseconds
{
	int32_t diff;

    do {
    	diff = (int32_t) (sysTime - time);
    } while (diff < 0);
}

/*************************************************************************************
 * An event-driven scheduling system using sysTime
 ************************************************************************************/

// Heap for an event-driven lightweight scheduling system
static unsigned CallbackHeapSize = 0;
static struct {
	void (*callback)(uint32_t time);
	uint32_t time;
} CallbackHeap[MAX_CALLBACK];

// Initialize the callback heap
static void
ScheduleInit()
{
	int i;

	for (i = 0; i < MAX_CALLBACK; i++) {
		CallbackHeap[i].callback = NULL;
		CallbackHeap[i].time = 0;
	}
}

// Swap two items in the callback heap
static inline
HeapSwap(int m, int n)
{
	uint32_t time;
	void (*callback)(uint32_t time);

	time = CallbackHeap[m].time;
	CallbackHeap[m].time = CallbackHeap[n].time;
	CallbackHeap[n].time = time;
	callback = CallbackHeap[m].callback;
	CallbackHeap[m].callback = CallbackHeap[n].callback;
	CallbackHeap[n].callback = callback;
}

// Schedule a callback event
void
ScheduleCallback(void (*callback)(uint32_t time), 	// the callback function
                 uint32_t time)						// when to call back
{
	int parent, child;

    // Check for callback queue overflow
	if (CallbackHeapSize >= MAX_CALLBACK-1) {
		uprintf("%s\n\r", "Error: callback queue overflows; some event is lost.");
		return;
	}

	// Add the callback event to the end of the heap
	child = CallbackHeapSize++;
	CallbackHeap[child].callback = callback;
	CallbackHeap[child].time = time;

	// Re-build the heap
	while (child > 0) {
		parent = (child - 1) >> 1;

		// Stop if the parent is at least as old as the child
		if (SysTimeLE(CallbackHeap[parent].time, CallbackHeap[child].time))
			break;

		// Otherwise, swap parent and child, and continue
		HeapSwap(parent, child);
		child = parent;
	}
}


// Execute all callbacks whose time is the same as or prior to sysTime. This function is non-blocking.
// FIXME: We can add a blocking version that utilizes CPU power saving features.
bool															// return true if any callback is executed
ScheduleExecute()
{
	int parent, leftChild, rightChild, elder;
	uint32_t time;
	void (*callback)(uint32_t time);
	bool executed = false;

	// Return immediately if the callback heap is empty
	if (CallbackHeapSize == 0)
		return executed;

	// Execute all callbacks prior to sysTime. If there is none, no callback will be executed
	while (SysTimeLE(CallbackHeap[0].time, sysTime)) {
		// Get the time and callback of the root and reduce the heap size
		time = CallbackHeap[0].time;
		callback = CallbackHeap[0].callback;
		CallbackHeapSize--;

		// If there is no other callback, clear the root
		if (CallbackHeapSize == 0) {
			CallbackHeap[0].time = 0;
			CallbackHeap[0].callback = NULL;
		}
		// Otherwise, rebuild the heap: Move the last node to the root, travel down
		// the heap until the parent at least as old as the child
		else {
			CallbackHeap[0].time = CallbackHeap[CallbackHeapSize].time;
			CallbackHeap[0].callback = CallbackHeap[CallbackHeapSize].callback;
			parent = 0;

			while (true) {
				// Check if left child is valid. If not, stop
				leftChild = (parent << 1) + 1;
				if (leftChild >= CallbackHeapSize)
					break;

				// Check the right child, if valid then decide the elder of
				// the left child and the right child
				rightChild = leftChild + 1;
				if (rightChild < CallbackHeapSize
						&& SysTimeLE(CallbackHeap[rightChild].time, CallbackHeap[leftChild].time))
					elder = rightChild;
				else
					elder = leftChild;

				// If the parent is as old as the elder, stop the swapping process;
				// otherwise, make the elder as the new parent, and travel down
				// the heap
				if (SysTimeLE(CallbackHeap[parent].time, CallbackHeap[elder].time))
					break;
				else {
					HeapSwap(parent, elder);
					parent = elder;
				}
			}
		}

		// Execute its callback, check first if it is NULL or not.
		// IMPORTANT: This should be done after heap-rebuilding, because the
	    // callback to be executed may insert one or more callbacks to the heap.
		// Double check the root of the heap is valid
		if (callback == NULL) {
			uprintf("%s\n\r", "Error: NULL callback at the heap's root.");
			continue;
		}
		else {
			callback(time);
			executed = true;
		}
	}

	return executed;
}

/*
 * Wait for a given number of milliseconds, using the SysCtlDelay() function.
 * Note: sysTimeWait() is more accurate because it uses the system tick counter,
 * which is not affect by CPU execution delay.
 */
void
WaitMs(uint32_t timeMs)
{
	// Return immediately if timeMs is zero; otherwise, SysCtlDelay() would wait for a long time
    if (timeMs == 0)
    	return;

    // Note: delayCount for SysCtlDelay() is three clock cycles
    uint32_t delayCount = (uint64_t) timeMs * clockRate / 1000;
    delayCount /= 3;
    SysCtlDelay(delayCount);
}

/*
 * Wait for a given number of microseconds
 */
void
WaitUs(uint32_t timeUs)
{
	// Return immediately if timeMs is zero; otherwise, SysCtlDelay() would wait for a long time
    if (timeUs == 0)
    	return;

    // Note: delayCount for SysCtlDelay() is three clock cycles
    uint32_t delayCount = (uint64_t) timeUs * clockRate / 1000000;
    delayCount /= 3;
    SysCtlDelay(delayCount);
}

/*
 * Initialize the clock of the LaunchPad
 */
void
LpInit()
{
    /// Specifies a 50-MHz CPU clock with a PLL divider of 4, using the
    /// main OSC.
    // Note: The code is from the workshop workbook.
    // Guess: The base clock is 25MHz (default value), boosted by PLL to
    // 200MHz, and then divided by 4 to 50MHz (I checked the datasheet).
    // I am not sure about the last flag.
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
                    | SYSCTL_XTAL_16MHZ);

    /// Get the system running clock
    clockRate = SysCtlClockGet();

    // Configure and enable System Tick for 1 millisecond period,
    // and register an interrupt handler to maintain system time.
    // FIXME The overhead of 1ms interrupt can be high for some application.
    // It is desirable to make the period configurable.
    SysTickPeriodSet(clockRate/1000-1);
    SysTickIntRegister(SysTickIntHandler);
    SysTickIntEnable();
    SysTickEnable();

    // Initialize the event-based scheduler
    ScheduleInit();

    // Initialize push button
    PbInit();

    // Initialize LEDs
    LedInit();

    // Initialize UART0
    UartInit();
}
