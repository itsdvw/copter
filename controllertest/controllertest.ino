/*
 Example sketch for the Xbox 360 USB library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <XBOXUSB.h>

#define MOTORS 3
#define MOTORS_OFF 20
#define MOTORS_MIN 25
#define MOTORS_MAX 254

USB Usb;
XBOXUSB Xbox(&Usb);

void setup() {
  pinMode(MOTORS, OUTPUT);
  Serial.begin(115200);
  //while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));
}

void loop() {
  Usb.Task();
  if (Xbox.Xbox360Connected) {
    if (Xbox.getButtonPress(R2)) {
      //Serial.print("L2: ");
      //Serial.print(Xbox.getButtonPress(L2));
      //Serial.print("\tR2: ");
      int in = Xbox.getButtonPress(R2);
      int out = map(in, 0, 255, MOTORS_MIN, MOTORS_MAX);
      Serial.print("R2: ");
      Serial.print(in);
      Serial.print("\tPWM: ");
      Serial.println(out);
      analogWrite(MOTORS, out);
    } else {
      analogWrite(MOTORS, MOTORS_OFF);
    }
  } else {
    analogWrite(MOTORS, MOTORS_OFF);
  }
  delay(5);
}
