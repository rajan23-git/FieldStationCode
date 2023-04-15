#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>


tmElements_t tm;

void setup() {
  tm.Hour = 0;
  tm.Minute = 44;
  tm.Second = 30;
  tm.Month = 4;
  tm.Day = 7;
  tm.Year = CalendarYrToTm(2023);
  RTC.write(tm);
}
void loop(){

}
