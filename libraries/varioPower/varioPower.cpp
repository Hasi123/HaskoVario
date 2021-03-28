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

//can´t be part of the class
void wakeUp() {
  //disable interrupts to avoid interrupt bootloop
  noInterrupts();
}

void VarioPower::sleep() {
  //disable interfaces to be able to reconfigure pins
  //disable TWI
  TWCR &= ~(1 << TWEN);
  //disable USART
  UCSR0B = 0;
  //disable SPI
  SPCR &= ~(1 << SPE);

  //turn off LDO
  PORTD &= ~(1 << PD5);

  //write all used pins to output low to completely eliminate any power drain
  DDRB |= (1 << PB4) | (1 << PB5) | (1 << PB3) | (1 << PB0);
  PORTB &= ~((1 << PB4) | (1 << PB5) | (1 << PB3) | (1 << PB0));
  DDRC |= (1 << PC4) | (1 << PC5);
  PORTC &= ~((1 << PC4) | (1 << PC5));
  DDRD |= (1 << PD0) | (1 << PD1) | (1 << PD2);
  PORTD &= ~((1 << PD0) | (1 << PD1) | (1 << PD2));

  //play shutdown sound also eliminates needing to debounce
  marioSounds.shutDown();

  //disable ADC
  ADCSRA = 0;

  noInterrupts();
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  //enable external interrupt to wake CPU
  EIFR = 3;  // clear flag for interrupts (shouldn't be needed)
  attachInterrupt (digitalPinToInterrupt(INTPIN), wakeUp, LOW);  //only LOW level interrupt is allowed to wake from sleep

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
  //disable watchdog to avoid WDT bootloop
  //actually not needed since done in optiboot bootloader
  //MCUSR = 0;
  //wdt_disable();

  //setup pins and analog reference
  //LDO is enabled by bootloader
  analogReference(INTERNAL);
  pinMode(INTPIN, INPUT_PULLUP);
  
  //trun on LDO
  DDRD |= (1 << PD5);
  PORTD |= (1 << PD5);
  
  beepStatus = 0;  //this needed?
  nextEvent = 0;

  marioSounds.bootUp();

  //if voltage too low goto sleep
  //R1: 10M, R2: 3M
  if (analogRead(A1) < 730) { //smaller 3.4V
    this->sleep();
  }
  
  //need to update?
  if (!digitalRead(INTPIN)) {
    cli();
    SP = RAMEND;
    void* bootloader = (void*)0x7800;
    goto *bootloader;
  }
}

bool VarioPower::update() {
  if (!(PIND & (1 << INTPINREG))) {
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
