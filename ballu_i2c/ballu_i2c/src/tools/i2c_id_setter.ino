void printHexByte(byte num, bool endl=false);
String hexString(byte num);
void scan_dev(int delay_us=0);

#include <Wire.h>
#define MAX_DEV_ID 0x20
#define ADR0_OPT 0b00011101 // default setting for higher byte of ADR0 is 0b00-11101
int dev_id = 0x00;
int new_id = 0x14;

void setup() {
  Wire.begin();

  Serial.begin(115200);
  while (!Serial); // Leonardo: wait for Serial Monitor
  Serial.println("\nTiny Encoder address writer\n");

  Serial.println("Trying to write ...");

  byte error = 0xFF;
  while(error != 0x00) {
    Wire.beginTransmission(dev_id);

    Wire.write(0x00); // mem address
    Wire.write(ADR0_OPT); // higher byte
    Wire.write(new_id); // lower byte
    // Wire.write(word(ADR0_OPT, new_id), 2); 

  delay(10);
    error = Wire.endTransmission();
    Serial.println("Returned Value: " + hexString(error));
    delay(500);
  }
  
  Serial.println("Finishing writing!");
  scan_dev(2000);
}

void loop() {
  scan_dev(1000);
}
  

void scan_dev(int delay_us=0) {
  int nDevices = 0;
  for (byte address = 0; address < MAX_DEV_ID; ++address) {
    Wire.beginTransmission(address); // The i2c_scanner uses the return value of the Wire.endTransmission to see if a device did acknowledge to the address.
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.println("I2C device found at address " + hexString(address));
      ++nDevices;
    } else if (error == 4) {
      Serial.println("Unknown error at address " + hexString(address));
    }
  }

  Serial.println("Found " + String(nDevices) + " devices.\n\n");
  delay(delay_us);
}

void printHexByte(byte num, bool endl=false) {
  Serial.print("0x");
  if (num < 16) {
      Serial.print("0");
  }
  Serial.print(num, HEX);
  if (endl) {
    Serial.println("");
  }
}

String hexString(byte num) {
  String hex = String(num, HEX);
  if (num < 0xF0) {
    hex = "0x0" + hex;
  } else {
    hex = "0x" + hex;
  }

  return hex;

}
