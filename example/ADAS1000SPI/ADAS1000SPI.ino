
#include <ADAS1000.h>
#include <SPI.h>
#define   CS    A4
#define   DRDY    A3



unsigned char Lead[24000]={0,};
ADAS1000 ADAS1000;

void printData(unsigned char* data, unsigned int bytes)
{
  
  unsigned long Lead1_cur = 0;
  unsigned long Lead2_cur = 0;
  unsigned long Lead3_cur = 0;
  unsigned long sum = 0;
  

  for (unsigned long u = 4; u < bytes; u = u + 24)
  {
    if (data[u] == 17)
    {
      Lead1_cur = ((unsigned long)data[u + 1] << 16) +
                  ((unsigned long)data[u + 2] << 8) +
                  ((unsigned long)data[u + 3] << 0);
      Lead2_cur = ((unsigned long)data[u + 5] << 16) +
                  ((unsigned long)data[u + 6] << 8) +
                  ((unsigned long)data[u + 7] << 0);
      Lead3_cur = ((unsigned long)data[u + 9] << 16) +
                  ((unsigned long)data[u + 10] << 8) +
                  ((unsigned long)data[u + 11] << 0);
      sum =  (Lead1_cur + Lead3_cur  +Lead2_cur);
      
      
      //Serial.print(Lead1_cur, DEC);
     // Serial.println(" ");
     //Serial.print(Lead2_cur, DEC);
    // Serial.print(" ");  
     //Serial.print(Lead3_cur, DEC);
     //Serial.println(" ");
     Serial.println(sum,DEC);
     //Serial.println(" ");
     
    }
  }

}



void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);
  Serial.begin(115200);
  pinMode(CS, OUTPUT);
  pinMode(DRDY,INPUT_PULLUP);
  digitalWrite(CS, HIGH);
  
  Serial.println("Initializing ADAS1000, PLease wait 2 seconds");
  
  ADAS1000.Init(ADAS1000_2KHZ_FRAME_RATE);
  ADAS1000.SetRegisterValue(ADAS1000_CMREFCTL, 0x85E0000A);
  ADAS1000.SetRegisterValue(ADAS1000_FILTCTL, 0x8B000010);
  ADAS1000.SetRegisterValue(ADAS1000_FRMCTL,0x8A1FCE00);
  ADAS1000.SetRegisterValue(ADAS1000_ECGCTL, 0x81E004AE);
  
}


void loop(void)
{
  ADAS1000.ReadData(Lead,1,1,1,1,0);
  printData(Lead,24);
  //Serial.println(old_sum);
    
}
