// Нужен double а не long !
double Map(double v, double in_min, double in_max, double out_min, double out_max) {
  return (v - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// dBm to Quality:
// uint8_t dBm2Quality(int8_t dBm) {
//   if (dBm <= -100)
//     return 0;
//   else if (dBm >= -50)
//     return 100;
//   else
//     return 2 * (dBm + 100);
// }

// void setup() {
//   Wire.begin(GPIO_NUM_6, GPIO_NUM_7);
// }

// void loop() {
//   byte error, address;
//   int nDevices;
//   Serial.println("Scanning...");

//   nDevices = 0;
//   for (address = 1; address < 127; address++) {
//     // The i2c_scanner uses the return value of
//     // the Write.endTransmisstion to see if
//     // a device did acknowledge to the address.
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();

//     if (error == 0) {
//       Serial.print("I2C device found at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.print(address, HEX);
//       Serial.println("  !");

//       nDevices++;
//     } else if (error == 4) {
//       Serial.print("Unknown error at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.println(address, HEX);
//     }
//   }
//   if (nDevices == 0)
//     Serial.println("No I2C devices found\n");
//   else
//     Serial.println("done\n");

//   delay(5000);  // wait 5 seconds for next scan
// }