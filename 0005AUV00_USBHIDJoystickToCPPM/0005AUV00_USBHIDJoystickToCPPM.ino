/*
  Convertidor Joystick Logitech Extreme3DPro USB Host Shield serie a PPM
  Por Francisco Carabaza
  febrero 2021

  Control de versiones:
  V00 12/feb/2021. Versión inicial.

  Este código utiliza la librería de Ben Vrewer Bowman para leer info del Joystick desde la USB Host Shield:
  https://github.com/BenBrewerBowman/Arduino_Logitech_3D_Joystick

  También se necesita tener instalada la librería: "USB_Host_Shield_2.0-master".
*/


#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include "le3dp_rptparser2.0.h"
#include <SPI.h>

USB                                             Usb;
USBHub                                          Hub(&Usb);
HIDUniversal                                    Hid(&Usb);
JoystickEvents                                  JoyEvents;
JoystickReportParser                            Joy(&JoyEvents);

//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 8  //set the number of chanels
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs) 8 canales 22500.
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 2  //set PPM signal output pin on the arduino
#define MinPulse 980              //set the min usecs of pulse 
#define MaxPulse 2020             //set the max usecs of pulse
#define default_servo_value 1500  //set the default servo value
#define MinInput 0                //set the min input from USB
#define MaxInput 1023                //set the max input from USB

//////////////////////////////////////////////////////////////////


int ppm[chanel_number]; // This array holds the servo values for the ppm signal change theese values in your code (usually servo values move between 1000 and 2000)*/
unsigned long time;
char USBData; // the USB data received

void setup() {
  // initialize serial communications:
  Serial.begin(115200); //Puerto para depurar

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif

  Serial.println("Start");
  if (Usb.Init() == -1)
    Serial.println("OSC did not start.");
  delay( 200 );

  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t>(PSTR("SetReportParser"), 1  );

  //initiallize default ppm value
  ppm[0] = default_servo_value;
  ppm[1] = default_servo_value;
  ppm[2] = default_servo_value;
  ppm[3] = default_servo_value;
  ppm[4] = default_servo_value - 6;
  ppm[5] = default_servo_value - 6;
  ppm[6] = default_servo_value - 6;
  ppm[7] = default_servo_value - 6;

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)

  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

void loop() {

  int Xval;   // 0 - 1023
  int Yval;   // 0 - 1023
  int Hat;    // 0 - 15;
  int Twist;  // 0 - 255
  int Slider; // 0 - 255
  int Button; // 0 - 12 (0 = No button)

  Usb.Task();                                                    //Use to read joystick input to controller
  JoyEvents.PrintValues();                                       //Returns joystick values to user
  JoyEvents.GetValues(Xval, Yval, Hat, Twist, Slider, Button);   //Copies joystick values to user

  ppm[0] = Xval;
  ppm[0] = map(ppm[0], MinInput, MaxInput, MinPulse, MaxPulse);
  ppm[0] = constrain(ppm[0], MinPulse, MaxPulse);

  ppm[1] = Yval;
  ppm[1] = map(ppm[1], MinInput, MaxInput, MinPulse, MaxPulse);
  ppm[1] = constrain(ppm[1], MinPulse, MaxPulse);

  ppm[2] = Slider;
  ppm[2] = map(ppm[2], 0, 250, MinPulse, MaxPulse);
  ppm[2] = constrain(ppm[2], MinPulse, MaxPulse);

  ppm[3] = Twist;
  ppm[3] = map(ppm[3], 0, 255, MinPulse, MaxPulse);
  ppm[3] = constrain(ppm[3], MinPulse, MaxPulse);

  ppm[4] = 512;
  ppm[4] = map(ppm[4], MinInput, MaxInput, MinPulse, MaxPulse);
  ppm[4] = constrain(ppm[4], MinPulse, MaxPulse);

  ppm[5] = 512;
  ppm[5] = map(ppm[5], MinInput, MaxInput, MinPulse, MaxPulse);
  ppm[5] = constrain(ppm[5], MinPulse, MaxPulse);

  ppm[6] = 512;
  ppm[6] = map(ppm[6], MinInput, MaxInput, MinPulse, MaxPulse);
  ppm[6] = constrain(ppm[6], MinPulse, MaxPulse);

  ppm[7] = 512;
  ppm[7] = map(ppm[7], MinInput, MaxInput, MinPulse, MaxPulse);
  ppm[7] = constrain(ppm[7], MinPulse, MaxPulse);

}

ISR(TIMER1_COMPA_vect) { //leave this alone
  static boolean state = true;
  TCNT1 = 0;
  if (state) { //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else { //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
    digitalWrite(sigPin, !onState);
    state = true;
    if (cur_chan_numb >= chanel_number) {
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;//
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }
  }
}
