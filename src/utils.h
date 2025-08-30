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