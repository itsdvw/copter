/*
 Example sketch for the Xbox 360 USB library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */
 
#include <XBOXUSB.h>

#define MOTORS 3

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
      int temp = Xbox.getButtonPress(R2);
      Serial.print("R2: ");
      Serial.println(temp);
      analogWrite(MOTORS, temp);
    } else {
      digitalWrite(MOTORS, LOW);
    }
  } else {
    digitalWrite(MOTORS, LOW);
  }
  delay(1);
}
