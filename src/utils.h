// Нужен double а не long !
double Map(double v, double in_min, double in_max, double out_min, double out_max) {
  return (v - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
