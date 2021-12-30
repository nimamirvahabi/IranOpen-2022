#include <tcs3200.h>
// white min max color numbers
#define Wmin_for_green_and_black 250
#define Wmin_for_red 180
// green min max color numbers
#define Gmax_for_green 112
#define Gmin_for_green 70
#define Gmax_for_red 112
#define Gmin_for_red 70
#define Gmax_for_blue 125
#define Gmin_for_blue 100
// red min max color numbers
#define Rmax_for_red 125
#define Rmin_for_red 80
// black min max color numbers
#define Bmax_for_red 80
#define Bmin_for_red 57
#define Bmax_for_green 112
#define Bmin_for_green 60

int red, green, blue, white;

tcs3200 tcs(8, 7, 10, 11, 12); // (S0, S1, S2, S3, output pin)  //

void setup() {
  Serial.begin(9600);
}

void loop() {
  //red = tcs.colorRead('r', 0);    //scaling can also be put to 0%, 20%, and 100% (default scaling is 20%)   ---    read more at: https://www.mouser.com/catalog/specsheets/TCS3200-E11.pdf
  //red = tcs.colorRead('r', 20);
  //red = tcs.colorRead('r', 100);

  red = tcs.colorRead('r', 20);   //reads color value for red
  Serial.print("R= ");
//  red = map(red , 27, 70, 255, 0);
  Serial.print(red);
  Serial.print("    ");

  green = tcs.colorRead('g', 20);   //reads color value for green
  Serial.print("G= ");
//  green = map(green, 25, 70, 255, 0);
  Serial.print(green);
  Serial.print("    ");

  blue = tcs.colorRead('b', 20);   //reads color value for blue
  Serial.print("B= ");
//  blue = map(blue, 25, 70, 255, 0);
  Serial.print(blue);
  Serial.print("    ");

  white = tcs.colorRead('c', 20);   //reads color value for white(clear)
  Serial.print("W(clear)= ");
  Serial.print(white);
  Serial.print("    ");

  //  Serial.print();
  if (white < 100) {
    Serial.println("Black");
  }
  else if (white <= 333 && green < Gmax_for_green && green > Gmin_for_green && red > Gmin_for_red && red < Gmax_for_red && blue > Gmin_for_blue + 1 && blue <= Gmax_for_blue) {
    Serial.println("Green");
  }
  else if (white <= Wmin_for_red && red > Rmin_for_red && red < Rmax_for_red) {
    Serial.println("Red");
  }
  else {
    Serial.println("White");
  }

}
