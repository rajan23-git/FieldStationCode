#include <DS1307RTC.h>
#include <Wire.h>
#include <TimeLib.h>

tmElements_t currTime;
void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  
 
}


void loop() {
  // put your main code here, to run repeatedly:
 while(Serial.available()>0){ 
  
    String timeString = "";  
    if(RTC.read(currTime)){
      
      timeString  = timeString + currTime.Hour +":"+currTime.Minute+":"+currTime.Second + " " + currTime.Month +"/" + currTime.Day+"/"+tmYearToCalendar(currTime.Year);
    }
    
    
    int msg = Serial.read();
    int sensorValue = analogRead(A0);
    float voltage = sensorValue * 5.0 / 1023.0;

    Serial.println(String(timeString)+","+String(voltage));
    
    String timeAdjustValue = Serial.readString();
    if(timeAdjustValue.length()>4){
      
    int dashPosArray[5];
    int i;
    int numDashSeen = 0;
    for(i=0;i<timeAdjustValue.length();i++){
      if(timeAdjustValue[i] == '-'){
        dashPosArray[numDashSeen] = i;
        numDashSeen++;
      }
    }

    String setMonth=timeAdjustValue.substring(0,dashPosArray[0]);
    String setDay = timeAdjustValue.substring(dashPosArray[0] +1,dashPosArray[1]);
    String setYear = timeAdjustValue.substring(dashPosArray[1] +1 ,dashPosArray[2]);
    String setHour = timeAdjustValue.substring(dashPosArray[2]+1,dashPosArray[3]);
    String setMinute = timeAdjustValue.substring(dashPosArray[3]+1,dashPosArray[4]);
    String setSecond = timeAdjustValue.substring(dashPosArray[4]+1,timeAdjustValue.length());
    String testSplit = "";
    testSplit = testSplit + setMonth+"/"+setDay+"/"+setYear+ " " + setHour+":"+setMinute+":"+setSecond;
    
    tmElements_t adjustTimeObject;
    adjustTimeObject.Month = setMonth.toInt();
    adjustTimeObject.Day = setDay.toInt();
    adjustTimeObject.Year = CalendarYrToTm(setYear.toInt());
    adjustTimeObject.Hour = setHour.toInt();
    adjustTimeObject.Minute = setMinute.toInt();
    adjustTimeObject.Second = setSecond.toInt();
    
    RTC.write(adjustTimeObject);
    
    Serial.println(" RTC Adjusted to = "+ String(testSplit));
    
    }
    
    
  }
}
