/*
 *******************************************************************************
 * USB-MIDI to Legacy Serial MIDI converter
 * Copyright (C) 2012-2016 Yuuichi Akagawa
 *
 * Idea from LPK25 USB-MIDI to Serial MIDI converter
 *   by Collin Cunningham - makezine.com, narbotic.com
 *
 * This is sample program. Do not expect perfect behavior.
 *******************************************************************************
 */

#include <usbh_midi.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

#ifdef USBCON
#define _MIDI_SERIAL_PORT Serial1
#else
#define _MIDI_SERIAL_PORT Serial
#endif
//////////////////////////
// MIDI Pin assign
// 2 : GND
// 4 : +5V(Vcc) with 220ohm
// 5 : TX
//////////////////////////

USB Usb;
USBHub  Hub1(&Usb);
USBH_MIDI  Midi1(&Usb);
USBH_MIDI  Midi2(&Usb);

void MIDI_poll();
void doDelay(unsigned long t1, unsigned long t2, unsigned long delayTime);

void setup()
{
  _MIDI_SERIAL_PORT.begin(31250);

  if (Usb.Init() == -1) {
    while(1); //halt
  }//if (Usb.Init() == -1...
  delay( 200 );
}

void loop()
{
  unsigned long t1;

  Usb.Task();
  t1 = micros();
  if( Usb.getUsbTaskState() == USB_STATE_RUNNING )
  {
    MIDI_poll();
  }
  //delay(1ms)
  doDelay(t1, micros(), 1000);
}

// Poll USB MIDI Controler and send to serial MIDI
void MIDI_poll()
{
    byte outBuf[ 3 ];
    uint8_t size;

    do {
      if( (size=Midi1.RecvData(outBuf)) > 0 ){
        //MIDI Output
        _MIDI_SERIAL_PORT.write(outBuf, size);
      }
    }while(size>0);
    do {
      if( (size=Midi2.RecvData(outBuf)) > 0 ){
        //MIDI Output
        _MIDI_SERIAL_PORT.write(outBuf, size);
      }
    }while(size>0);
}

// Delay time (max 16383 us)
void doDelay(unsigned long t1, unsigned long t2, unsigned long delayTime)
{
    unsigned long t3;

    if( t1 > t2 ){
      t3 = (4294967295 - t1 + t2);
    }else{
      t3 = t2 - t1;
    }

    if( t3 < delayTime ){
      delayMicroseconds(delayTime - t3);
    }
}
