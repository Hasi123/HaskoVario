/* varioPower -- Turns the vario off by disabling LDO and putting the MCU to sleep 
 *
 * Copyright 2016-2020 David HASKO
 * 
 * This file is part of GNUVario.
 *
 * GNUVario is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNUVario is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include "varioPower.h"
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <marioSounds.h>

//can´t be part of the class
void wakeUp(){
  //do nothing just trigger a wakeup
}

void VarioPower::sleep(){

  //play shutdown sound also eliminates needing to debounce
  marioSounds.shutDown();
  
  //TODO reconfigure pins to sleep for minimal power consumption
/*  DDRB = 0;
  DDRC = 0;
  DDRD = 0;
  PORTB = 0;
  PORTC = 0;
  PORTD = 0;*/
  
  //turn off LDO
  PORTD &= ~(1<<PD5);
  
  //disable ADC
  ADCSRA = 0;
  
  noInterrupts();
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  
  //enable external interrupt to wake CPU
  //EIFR = (1 << INTF1);  // clear flag for interrupt 1 (shouldn't be needed)
  attachInterrupt (digitalPinToInterrupt(INTPIN), wakeUp, LOW);  //only LOW level interrupt is allowed to wake from sleep
 
  //turn off brown-out enable in software, will be automatically reenabled after wake
  sleep_bod_disable();
  
  // We are guaranteed that the sleep_cpu call will be done
  // as the processor executes the next instruction after
  // interrupts are turned on.
  interrupts ();  // one cycle
  sleep_cpu ();  // one cycle
  
  //sleep here//
  
  //ISR routine runs
  sleep_disable();
  
  //to stop ISR routine from re-running (do we need this?)
  noInterrupts();
  detachInterrupt(digitalPinToInterrupt(INTPIN));

  //reset atmega
  //asm volatile (" jmp 0");  //restart code from beginning
  //reset by watchdog is probably better
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}

void VarioPower::init(){
  //setup pins
  DDRD |= (1<<PD5); //LDO output
  PORTD |= (1<<PD5); //LDO on
  analogReference(INTERNAL);
  pinMode(INTPIN, INPUT_PULLUP);
  
  //check voltage and if ok continue
  //R1: 10M, R2: 3M
/*  if (analogRead(A1) < 730) { //smaller 3.4V
    this->sleep();
  }*/
}

void VarioPower::update(){
  if (digitalRead(INTPIN) == LOW){
	  this->sleep();
  }
/*  int volts = analogRead(A1);
  if (volts < 740) { marioSounds.lowVoltage(); }  //3.45V needs revision to continue running loop and only beep every minute
  else if (volts < 708) { this->sleep(); }  //3.3V
  */
}
