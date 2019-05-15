#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Uduino.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <I2Cdev.h>


#define BNO055_SAMPLERATE_DELAY_MS (100)
Uduino uduino("UduinoProjectTest1");
Adafruit_BNO055 bno  = Adafruit_BNO055(55);



void setup(void) {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("test start");
  uduino.addCommand("go", LoopyLoop);

 if (!bno.begin())
  {
    Serial.print("! ");
  }
  delay(500);
  
  bno.setExtCrystalUse(true);

}
 
void LoopyLoop(){
  sensors_event_t event; 
  bno.getEvent(&event);
  

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
  }
void loop(void) {

  LoopyLoop();
  
  uduino.update();       //!\ This part is mandatory
 
}
/*void GetVariable(){

  
  }*/
