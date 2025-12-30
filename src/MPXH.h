#include <Wire.h>
#include "SdFat.h"



#ifdef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif // SDCARD_SS_PIN
#define SPI_CLOCK SD_SCK_MHZ(50)
#define SD_CONFIG SdioConfig(FIFO_SDIO)

SdFs sd;
FsFile newFile;

double prevTime = 0;
double curTime  = 0;
double diff     = 0;
double period   = 0;
double hz       = 0;

long i = 0;

// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire
int val;
float calc;
float scaledup;
float pres;
void setup() {
    WIRE.begin();
    if (!sd.begin(SD_CONFIG)) {
      return;
    }

  newFile = sd.open("log.txt", FILE_WRITE);
}


void loop() {
  // byte error, address;
  // int nDevices;
  val = analogRead(A0); 
  calc = (val/1023.0) * (3.3);
  scaledup = calc * 1.5;
  pres = (scaledup+0.204)/0.0204;
  unsigned long timestamp = millis();
    prevTime = micros();
  
  newFile.print(timestamp);
  newFile.print("\t");
  newFile.println(pres);
  i++;

  if (!(i % 100))
  {
    newFile.flush();
    newFile.close();
    newFile = sd.open("log.txt", FILE_WRITE);
  }

 // Serial.println("Scanning...");

//   nDevices = 0;
//   for(address = 1; address < 127; address++ )
//   {
//     // The i2c_scanner uses the return value of
//     // the Write.endTransmisstion to see if
//     // a device did acknowledge to the address.
//     WIRE.beginTransmission(address);
//     error = WIRE.endTransmission();

//     if (error == 0)
//     {
//       Serial.print("I2C device found at address 0x");
//       if (address<16)
//         Serial.print("0");
//       Serial.print(address,HEX);
//       Serial.println("  !");

//       nDevices++;
//     }
//     else if (error==4)
//     {
//       Serial.print("Unknown error at address 0x");
//       if (address<16)
//         Serial.print("0");
//       Serial.println(address,HEX);
//     }
//   }
//   if (nDevices == 0){
//     Serial.println("No I2C devices found\n");
//   }
//   else{
//     Serial.println("done\n");
//   } 
  delay(50);     
}


