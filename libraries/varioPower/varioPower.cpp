/* varioPower -- Turns the vario off by disabling LDO and putting the MCU to sleep

   Copyright 2016-2020 David HASKO

   This file is part of GNUVario.

   GNUVario is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   GNUVario is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include "varioPower.h"
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <marioSounds.h>

//ISRs can´t be part of the class
void wakeUp() {
  //disable interrupts to avoid interrupt bootloop
  noInterrupts();
}
void reset() {
  wdt_enable(WDTO_1S);
}

void VarioPower::sleep() {
  //disable reset watchdog
  detachInterrupt(digitalPinToInterrupt(INTPIN));
  wdt_disable();

  //disable interfaces to be able to reconfigure pins
  //disable TWI
  TWCR &= ~bit(TWEN);
  //disable USART
  UCSR0B = 0;
  //disable SPI
  SPCR &= ~bit(SPE);

  //turn off LDO
  PORTD &= ~bit(PD5);

  //write all used pins to output low to completely eliminate any power drain
  DDRB |= bit(PB4) | bit(PB5) | bit(PB3) | bit(PB0);
  PORTB &= ~(bit(PB4) | bit(PB5) | bit(PB3) | bit(PB0));
  DDRC |= bit(PC4) | bit(PC5);
  PORTC &= ~(bit(PC4) | bit(PC5));
  DDRD |= bit(PD0) | bit(PD1) | bit(PD2);
  PORTD &= ~(bit(PD0) | bit(PD1) | bit(PD2));

  //play shutdown sound also eliminates needing to debounce
  marioSounds.shutDown();

  //disable ADC
  ADCSRA = 0;

  noInterrupts();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  //enable external interrupt to wake CPU
  EIFR = 3;  //clear flag for interrupts
  attachInterrupt(digitalPinToInterrupt(INTPIN), wakeUp, LOW);  //only LOW level interrupt is allowed to wake from sleep

  //turn off brown-out enable in software, will be automatically reenabled after wake
  sleep_bod_disable();

  // We are guaranteed that the sleep_cpu call will be done
  // as the processor executes the next instruction after
  // interrupts are turned on.
  interrupts ();  // one cycle
  sleep_cpu ();  // one cycle

  //////////////
  //sleep here//
  //////////////

  //ISR routine runs
  
  sleep_disable();

  //reset atmega
  //asm volatile (" jmp 0");  //restart code from beginning
  //reset by watchdog is recommended by the datasheet and is probably better
  wdt_enable(WDTO_15MS);
  while (1) {}
}

void VarioPower::init() {
  //setup pins and analog reference
  analogReference(INTERNAL);
  //turn on pullup for button
  PORTD |= bit(INTPIN);
  
  //trun on LDO
  DDRD |= bit(PD5);
  PORTD |= bit(PD5);
  
  beepStatus = 0;  //this needed?
  nextEvent = 0;

  marioSounds.bootUp();

  //if voltage too low goto sleep
  //R1: 10M, R2: 3M
  if (analogRead(A1) < 730) { //smaller 3.4V
    this->sleep();
  }
  
  //need to update?
  if (!(PIND & bit(INTPIN))) {
    cli();
    SP = RAMEND;
    void* bootloader = (void*)0x7800;
    goto *bootloader;
  }
  
  //reset mcu after 1 s on button push if code hangs somewhere
  EIFR = 3;  //clear flag for interrupts
  attachInterrupt(digitalPinToInterrupt(INTPIN), reset, FALLING);
}

bool VarioPower::update() {
  if (!(PIND & bit(INTPIN))) {
    this->sleep();
  }

  uint16_t volts = analogRead(A1);
  if (volts < 740) { //3.45V
    if (volts < 708) {
      this->sleep();  //3.3V
    }
    else {
      if (nextEvent <= millis()) {
        uint16_t nextAdd;
        beepStatus++;

        if (beepStatus < 4) {
          marioSounds.lowVoltage();
          nextAdd = 200;
        }
        else {
          nextAdd = 60000;
          beepStatus = 0;
        }

        nextEvent += nextAdd;

      }
    }
  }
  else {
    beepStatus = 0;
  }
  
  if (beepStatus > 0 && beepStatus < 4) {
    return false;
  }
  return true;
}
