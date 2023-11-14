////////////////////////////////////////////////////////////////////////////////////////
// Arduino code for NIJ 3D imaging system using Serail communication.
// 
// Copyright(c) 2023 XYZT Lab, Purdue University.
// Author: Liming Chen
//
// V1.0
// Date: 2023-07-26
// Contact: Liming Chen (chen3496@purdue.edu) 
///////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <stdlib.h>

// I2C config
byte DLP_I2C_ADDR = 0x1B; // 0x36 -> 1bit

// Projector frequency config
float defaultFreq = 60.9;
uint32_t defaultPreExp = 286;
uint32_t defaultExp = 16074;
uint32_t defaultPostExp = 51;

void setup() {
  Serial.end();
  Wire.begin();
  Wire.setClock(100000); // standard mode
  // put your setup code here, to run once:
  Serial.begin(38400);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() 
{
  if (Serial)
  {
    if (Serial.available() > 0)
    {
      byte buffer[32];
      int recvNum = 0;   
      while (Serial.available() > 0) 
      { 
        buffer[recvNum] = Serial.read();
        delay(2);
        recvNum++; 
      }
      if (recvNum > 0)
      {
        if (recvNum == 4 && buffer[0] == 's' && buffer[1] == 'c' && buffer[2] == 'a' && buffer[3] == 'n')
        {
          Serial.print("check");
        }
        else
        {
          uint32_t exp = parseExposureTime(buffer, recvNum);
          if (buffer[0] == 'p' || buffer[0] == 'P')
          {
            displayPreviewPatterns(exp);
          }
          else if (buffer[0] == 'c' || buffer[0] == 'C')
          {
            displayCalibrationPatterns(exp);
          }
          else if (buffer[0] == 'm' || buffer[0] == 'M')
          {
             digitalWrite(LED_BUILTIN, HIGH);
            displayMeasurementPatterns(exp);
          }
          else if (buffer[0] == 'h' || buffer[0] == 'H')
          {
             digitalWrite(LED_BUILTIN, HIGH);
              displayHDRPatterns(exp);
          }
          else if (buffer[0] == 'r' || buffer[0] == 'R')
          {
            displayRealtimePatterns(exp);
          }
          else if (buffer[0] == 'b' || buffer[0] == 'B')
          {
            blackPatternPreview(exp);
          }
          else if (buffer[0] == 'l' || buffer[0] == 'L')
          {
            blackPatternCapture(exp);
          }
          else if (buffer[0] == 's' || buffer[0] == 'S')
          {
            patternStartCtrl(0x00);
          }
        }
      }
    }
  }
}

void uint32ToBytes(uint32_t valueIn, byte* byte1, byte* byte2, byte* byte3, byte* byte4)// byte1: LSB, byte4: MSB
{
  (*byte1) = valueIn;
  (*byte2) = valueIn >> 8;
  (*byte3) = valueIn >> 16;
  (*byte4) = valueIn >> 24;
}

void bytesToUint32(byte byte1, byte byte2, byte byte3, byte byte4, uint32_t* valueOut) // byte1: LSB, byte4: MSB
{
  (*valueOut) = (((uint32_t)byte4)<<24) | (((uint32_t)byte3) << 16) | (((uint32_t)byte2) << 8) | ((uint32_t)byte1);
}

uint32_t parseExposureTime(const char* str, int size)
{
  double freq = defaultFreq;
  uint32_t i = 0;
  str++; // skip the mode
  char freqString[255];
  bool valid = true;

  if (size > 255 || size < 2)
  {
    freq = defaultFreq;
    valid = false;
  }
  else
  {
    while(i < size - 1)
    {
      if ((*str >= '0' && *str <= '9') || (*str == '.'))
      {
        freqString[i] = *str;
        //Serial.print(*str); 
        //Serial.print('\n');
        str++;
        i++;
      }
      else
      {
        freq = defaultFreq;
        valid = false;
        break;
      }
    }

    if (valid == true)
    {
      freqString[i] = '\0';
      //Serial.print(freqString);
      //Serial.print('\n');
      freq = atof(freqString);
    }
  }

  // removed by Song Zhang (song.zhang@VEOptics.com)
  //if (freq <= 0 || freq >= 120)
  //{
  //  freq = defaultFreq; 
  //}

  return  (uint32_t)(1000000.0 / freq); // return exposure time in us 
}

void configSequenceTable(byte mode, byte ind, byte num, uint32_t illum, uint32_t pre_illum, uint32_t post_illum)
{
  byte illum_byte1, illum_byte2, illum_byte3, illum_byte4;
  uint32ToBytes(illum, &illum_byte1, &illum_byte2, &illum_byte3, &illum_byte4);

  byte preIllum_byte1, preIllum_byte2, preIllum_byte3, preIllum_byte4;
  uint32ToBytes(pre_illum, &preIllum_byte1, &preIllum_byte2, &preIllum_byte3, &preIllum_byte4);

  byte postIllum_byte1, postIllum_byte2, postIllum_byte3, postIllum_byte4;
  uint32ToBytes(post_illum, &postIllum_byte1, &postIllum_byte2, &postIllum_byte3, &postIllum_byte4);

  Wire.beginTransmission(DLP_I2C_ADDR);
  Wire.write(0x98);
  Wire.write(mode); // start
  Wire.write(ind); // start pattern index
  Wire.write(num); // numbers of patterns to display
  Wire.write(0x01);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(illum_byte1);
  Wire.write(illum_byte2);
  Wire.write(illum_byte3);
  Wire.write(illum_byte4);
  Wire.write(preIllum_byte1);
  Wire.write(preIllum_byte2);
  Wire.write(preIllum_byte3);
  Wire.write(preIllum_byte4);
  Wire.write(postIllum_byte1);
  Wire.write(postIllum_byte2);
  Wire.write(postIllum_byte3);
  Wire.write(postIllum_byte4);
  Wire.endTransmission();
 // delay(2);
 delayMicroseconds(100);// changed by Song from 2 ms to 100 us
}

void trigAndInternalMode(bool trigEnable)
{
// internal pattern trigger out
  Wire.beginTransmission(DLP_I2C_ADDR);
  Wire.write(0x92);
  if (trigEnable == true)
  {
    Wire.write(0x07);
  }
  else
  {
    Wire.write(0x05);
  }

  Wire.write(0x14); // Delay 20 micro second
  Wire.write(0x00);
  Wire.write(0x00); 
  Wire.write(0x00);
  Wire.endTransmission();
  // delay(2);
 delayMicroseconds(100);// changed by Song from 2 ms to 100 us

  // internal pattern trigger in
  Wire.beginTransmission(DLP_I2C_ADDR);
  Wire.write(0x90);
  Wire.write(0x00);
  Wire.endTransmission();
  // delay(2);
 delayMicroseconds(100);// changed by Song from 2 ms to 100 us

  // internal pattern ready
  Wire.beginTransmission(DLP_I2C_ADDR);
  Wire.write(0x95);
  Wire.write(0x00);
  Wire.endTransmission();
 // delay(2);
 delayMicroseconds(100);// changed by Song from 2 ms to 100 us  

  // internal pattern mode
  Wire.beginTransmission(DLP_I2C_ADDR);
  Wire.write(0x05);
  Wire.write(0x04);
  Wire.endTransmission();
  // delay(2);
 delayMicroseconds(100);// changed by Song from 2 ms to 100 us

  // delay for system initialization
  //delay(20);
}

void patternStartCtrl(byte num)
{
  // internal pattern start
  Wire.beginTransmission(DLP_I2C_ADDR);
  Wire.write(0x9E);
  Wire.write(0x00);
  Wire.write(num); // repeat times
  Wire.endTransmission();
}

bool readValidExposureTime(uint32_t illum, uint32_t* pre_illum, uint32_t* post_illum)
{
  byte illum_byte1, illum_byte2, illum_byte3, illum_byte4;
  uint32ToBytes(illum, &illum_byte1, &illum_byte2, &illum_byte3, &illum_byte4);

  // send I2C read command
  Wire.beginTransmission(DLP_I2C_ADDR);
  Wire.write(0x9D);
  Wire.write(0x01);
  Wire.write(0x00);
  Wire.write(illum_byte1);
  Wire.write(illum_byte2);
  Wire.write(illum_byte3);
  Wire.write(illum_byte4);
  Wire.endTransmission();

  // receive data from I2C
  byte validExpTime[13];
  Wire.requestFrom(DLP_I2C_ADDR, 13);
  byte i = 0;
  while(Wire.available())
  {
    validExpTime[i] = Wire.read();
    i++;
  }
  if (validExpTime[0] & 0x01 == 0x01) // The lowest bit == 1: request exporsure time supported
  {
    bytesToUint32(validExpTime[5], validExpTime[6], validExpTime[7], validExpTime[8], pre_illum);
    bytesToUint32(validExpTime[9], validExpTime[10], validExpTime[11], validExpTime[12], post_illum);
    return true;
  }
  else
  {
    (*pre_illum) = defaultPreExp;
    (*post_illum) = defaultPostExp;
    return false;
  }
}


void displayPreviewPatterns(uint32_t exposureTime)
{
  uint32_t expSet;
  uint32_t pre_illum;
  uint32_t post_illum;
  bool valid = readValidExposureTime(exposureTime, &pre_illum, &post_illum);
  if (valid == true)
  {
    expSet = exposureTime - pre_illum - post_illum; 
  }
  else
  {
    expSet = defaultExp;
  }
  configSequenceTable(0x01, 0x01, 0x01, expSet, pre_illum, post_illum);
  trigAndInternalMode(true);
  patternStartCtrl(0xFF);
}


void displayMeasurementPatterns(uint32_t exposureTime)
{
  uint32_t expSet;
  uint32_t pre_illum;
  uint32_t post_illum;
  bool valid = readValidExposureTime(exposureTime, &pre_illum, &post_illum);
  if (valid == true)
  {
    expSet = exposureTime - pre_illum - post_illum; 
  }
  else
  {
    expSet = defaultExp;
  }
  configSequenceTable(0x01, 0x00, 0x01, expSet, pre_illum, post_illum);
  configSequenceTable(0x00, 0x01, 0x01, expSet, pre_illum, post_illum);
  configSequenceTable(0x00, 0x02, 0x1E, expSet, pre_illum, post_illum);
  //configSequenceTable(0x01, 0x02, 0x1E, expSet, pre_illum, post_illum);
  trigAndInternalMode(true);
  //patternStartCtrl(0x00); // Changed by Song
   patternStartCtrl(0xFF);
}

void displayHDRPatterns(uint32_t exposureTime)
{
  uint32_t expSet;
  uint32_t pre_illum;
  uint32_t post_illum;
  bool valid = readValidExposureTime(exposureTime, &pre_illum, &post_illum);
  if (valid == true)
  {
    expSet = exposureTime - pre_illum - post_illum; 
  }
  else
  {
    expSet = defaultExp;
  }
  configSequenceTable(0x01, 0x00, 0x01, expSet, pre_illum, post_illum);
  configSequenceTable(0x00, 0x01, 0x01, expSet, pre_illum, post_illum);
  configSequenceTable(0x00, 0x02, 0x1E, expSet, pre_illum, post_illum);
  //configSequenceTable(0x01, 0x02, 0x1E, expSet, pre_illum, post_illum);
  trigAndInternalMode(true);
  patternStartCtrl(0x00); // Changed by Song
  // patternStartCtrl(0xFF);
}
void displayRealtimePatterns(uint32_t exposureTime)
{
  uint32_t expSet;
  uint32_t pre_illum;
  uint32_t post_illum;
  bool valid = readValidExposureTime(exposureTime, &pre_illum, &post_illum);
  if (valid == true)
  {
    expSet = exposureTime - pre_illum - post_illum; 
  }
  else
  {
    expSet = defaultExp;
  }
  configSequenceTable(0x01, 0x04, 0x06, expSet, pre_illum, post_illum);
  //configSequenceTable(0x01, 0x01, 0x01, expSet, pre_illum, post_illum);
  //configSequenceTable(0x00, 0x02, 0x05, expSet, pre_illum, post_illum);
  trigAndInternalMode(true);
  // patternStartCtrl(0x63);
  patternStartCtrl(0xFF);
}

void displayCalibrationPatterns(uint32_t exposureTime)
{
  uint32_t expSet;
  uint32_t pre_illum;
  uint32_t post_illum;
  bool valid = readValidExposureTime(exposureTime, &pre_illum, &post_illum);
  if (valid == true)
  {
    expSet = exposureTime - pre_illum - post_illum; 
  }
  else
  {
    expSet = defaultExp;
  }
  configSequenceTable(0x01, 0x00, 0x01, expSet, pre_illum, post_illum);
  configSequenceTable(0x00, 0x01, 0x01, expSet, pre_illum, post_illum);
  configSequenceTable(0x00, 0x02, 0x1E, expSet, pre_illum, post_illum);
  configSequenceTable(0x00, 0x03, 0x20, expSet, pre_illum, post_illum);
  trigAndInternalMode(true);
  patternStartCtrl(0x00);
}

void blackPatternPreview(uint32_t exposureTime)
{
  uint32_t expSet;
  uint32_t pre_illum;
  uint32_t post_illum;
  bool valid = readValidExposureTime(exposureTime, &pre_illum, &post_illum);
  if (valid == true)
  {
    expSet = exposureTime - pre_illum - post_illum; 
  }
  else
  {
    expSet = defaultExp;
  }
  configSequenceTable(0x01, 0x00, 0x01, expSet, pre_illum, post_illum);
  trigAndInternalMode(true);
  patternStartCtrl(0xFF);
}

void blackPatternCapture(uint32_t exposureTime)
{
  uint32_t expSet;
  uint32_t pre_illum;
  uint32_t post_illum;
  bool valid = readValidExposureTime(exposureTime, &pre_illum, &post_illum);
  if (valid == true)
  {
    expSet = exposureTime - pre_illum - post_illum; 
  }
  else
  {
    expSet = defaultExp;
  }
  configSequenceTable(0x01, 0x00, 0x01, expSet, pre_illum, post_illum);
  trigAndInternalMode(true);
  patternStartCtrl(0x00);
}