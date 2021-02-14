/*
  Convertidor Joystick Logitech Extreme3DPro USB Host Shield serie a PPM
  Por Francisco Carabaza
  febrero 2021

  Los canales de salida de la señal PPM estan controlados por los siguientes controles del joystick:
  Eje X controla el canal 1.
  Eje Y controla el canal 2.
  Slider controla el canal 3.
  Twist controla el canal 4.
  Hat izquierda/derecha, control incremental del canal 5.
  Hat arriba/abajo, control incremental del canal 6.
  Botón 3, centra los canales 5 y 6.
  Botones 7 al 13 controlan el canal 7 para control de modos.
  Botón 1 (gatillo) controla el canal 8.


  Control de versiones:
  V00 12/feb/2021, versión inicial.
  V01 13/feb/2021, control incremental de canales 5 y 6 desde el hat y control de modos en canales 7 y 8.

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
#define MaxInput 1023             //set the max input from USB



//////////////////////////////////////////////////////////////////


int ppm[chanel_number]; // This array holds the servo values for the ppm signal change theese values in your code (usually servo values move between 1000 and 2000)*/
unsigned long time;
char USBData; // the USB data received
unsigned int hatValX = 32768;
unsigned int hatValY = 32768;
unsigned int incrementoHat = 10;
unsigned int modeCH = 0; // 1 - 6
unsigned int modeVal = 0;// 0 - 100

int Xval;   // 0 - 1023
int Yval;   // 0 - 1023
int Hat;  // 0 - 15;
int Twist;  // 0 - 255
int Slider; // 0 - 255
int Button; // 0 - 12 (0 = No button)

void setup() {
  // initialize serial communications:
  Serial.begin(115200); //Puerto para depurar

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif

  //Serial.println("Start");
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

  while (Hat != 8) { // quieto aquí hasta que el Hat está en reposo
    Usb.Task();                                                    //Use to read joystick input to controller
    JoyEvents.GetValues(Xval, Yval, Hat, Twist, Slider, Button);   //Copies joystick values to user
  }


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

  Usb.Task();                                                    //Use to read joystick input to controller
  //JoyEvents.PrintValues();                                       //Returns joystick values to user
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

  switch (Hat) {
    case 0: //Hat arriba
      hatValY += incrementoHat;
      break;
    case 1: //Hat arriba y derecha
      hatValX += incrementoHat;
      hatValY += incrementoHat;
      break;
    case 2: //Hat derecha
      hatValX += incrementoHat;
      break;
    case 3: //Hat derecha y abajo
      hatValX += incrementoHat;
      hatValY -= incrementoHat;
      break;
    case 4: //Hat abajo
      hatValY -= incrementoHat;
      break;
    case 5: //Hat abajo e izquierda
      hatValX -= incrementoHat;
      hatValY -= incrementoHat;
      break;
    case 6: //Hat izquierda
      hatValX -= incrementoHat;
      break;
    case 7: //Hat izquierda y arriba
      hatValX -= incrementoHat;
      hatValY += incrementoHat;
      break;
    case 8: //Hat suelto
      break;
    default: //Posición no conocida del Hat
      break;
  }
  hatValX = constrain(hatValX, 200, 65336);
  hatValY = constrain(hatValY, 200, 65336);

  ppm[4] = map(hatValX, 0, 65534, MinPulse, MaxPulse);
  ppm[4] = constrain(ppm[4], MinPulse, MaxPulse);

  ppm[5] = map(hatValY, 0, 65534, MinPulse, MaxPulse);
  ppm[5] = constrain(ppm[5], MinPulse, MaxPulse);

  switch (Button) {
    case 0: //Sin pulsación de botones
      ppm[7] = 0;
      break;
    case 1: //Gatillo índice pulsado
      ppm[7] = 1;
      break;
    case 2: //Botón pulgar 2 pulsado
      break;
    case 3: ////Botón pulgar 3 pulsado. Resetear valores del Hat
      hatValX = 32768;
      hatValY = 32768;
      break;
    case 4: ////Botón pulgar 4 pulsado
      break;
    case 5: ////Botón pulgar 5 pulsado
      break;
    case 6: ////Botón pulgar 6 pulsado
      break;
    case 7: //Botón inferior 7 pulsado, modo 0
      modeCH = 0;
      break;
    case 8: //Botón inferior 8 pulsado, modo 1
      modeCH = 1;
      break;
    case 9: //Botón inferior 9 pulsado, modo 2
      modeCH = 2;
      break;
    case 10: //Botón inferior 10 pulsado, modo 3
      modeCH = 3;
      break;
    case 11: //Botón inferior 11 pulsado, modo 4
      modeCH = 4;
      break;
    case 12: //Botón inferior 12 pulsado, modo 5
      modeCH = 5;
      break;
    default: //Posición no conocida del Hat, modo 6
      break;
  }

  ppm[6] = map(modeCH, 0, 5, MinPulse, MaxPulse);
  ppm[6] = constrain(ppm[6], MinPulse, MaxPulse);

  ppm[7] = map(ppm[7], 0, 1, MinPulse, MaxPulse);
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
