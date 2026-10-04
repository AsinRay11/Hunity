#include "Wire.h"
#include "Arduino.h"
#include <EEPROM.h>
#include "MeasurementBuffer.h"
#include "TelemetryBuffer.h"
#include "TelecommandBuffer.h"
#include "BitChain.h"
#include "Sensor.h"
#include "hardware/watchdog.h"

#include "pico/stdlib.h"
#include "hardware/clocks.h"


#define EEPROM_SIZE 256
//eeprom addressek
#define EEPROM_SETTED 0 //Csak akkor írja át a paramétereket ha be vannak állítva az eepromban [Base Form (Infinitive):	Set; Simple Past Tense:	Set; Past Participle:	Set]
#define EEPROM_TRUE 15

#define EEPROM_FNLENGTH 1
#define EEPROM_FNDECPOINT 2
#define EEPROM_FNBASE 3
#define EEPROM_MEASURMENT_MODE 4
#define EEPROM_REQUIRED_MEASURMENT_COUNT 5
#define EEPROM_TO_AVERAGE_MEASURMENTS 6
#define EEPROM_PACKET_SIZE 7

#define EEPROM_SENSOR_SET_GAIN 8
#define EEPROM_SENSOR_MEASURMENT_MODE 9
#define EEPROM_SENSOR_INTEGRATION_TIME 10

#define EEPROM_TIMED_MEASURMENT_DELAY 11
#define EEPROM_ENABLED_SPECTRUMS 15
#define EEPROM_SPECTRUM_GROUPING 33

#define EEPROM_KEEP_EVERY_NTH_MESSAGE 51

#define EEPROM_SPIN_PERIOD_MS 58
#define EEPROM_SPIN_PERIOD_DRIFT 62
#define EEPROM_ORBIT_PERIOD_SEC 63
#define EEPROM_ORBIT_PERIOD_DRIFT 65
#define EEPROM_SPIN_DIR_CW 67
#define EEPROM_ORBIT_DIR_CW 68
#define EEPROM_NUM_SAMPLES_IN_CIRCLE 69
#define EEPROM_USE_SPIN_COMPENSATION 70
#define EEPROM_SPIN_COMPENSATION_ZERO_POINT 71
#define EEPROM_BINARY_SPIN_COMPENSATION 73
#define EEPROM_SINGLE_TARGET_POSITION 74
#define EEPROM_SPIN_ZERO_POINT 76
#define EEPROM_ORBIT_ZERO_POINT 80
#define EEPROM_SPIN_DRIFT_DEG_PER_HOUR 84


#define SDA1_PIN 2
#define SCL1_PIN 3 
#define SDA_PIN 0
#define SCL_PIN 1
#define SLAVE_ADDRESS 0x50
#define SYNC_COMMAND 0x54 

#define DEVELOPMENT

Sensor sensor;
byte fnLength = 6;     //Tömörített floatok hossza
byte fnDecPoint  = 2;  //Hol legyen a tizedes vessző a tömörített számban
byte fnBase = 0;       //Tömörítésnél a logaritmus alapja

byte measurmentMode = 0;  //Melyik mérésmódot (mm) használjuk
int timedMeasurmentDelay = 5000; //0-ás mm (timed) delay
byte requiredMeasurmentCount = 1; //Hány mérés kell a feldologozáshoz

uint8_t nthMessagePointer = 0;
uint8_t keepEveryNthMessage = 1;

//Melyik spektrumokat szeretnénk leküldeni
bool enabledSpectrums[] = {true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true};
//Hogyan csoportosítjuk a spektrumkat
//true jelöli a csoport végét
bool spectrumGrouping[]= {true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true};
bool toAverageMeasurments = false; //Megmondja hogy a több mérést hogyan dolgozza fel: true: átlagolja az adatokat, false: nem átlagolja 
byte packetSize = 1;

//long timeSync = 1 730 937 600;//DEBUG
//long timeSync = 0;//DEBUG
long timeSync = 0;

int tsReceiveTime = 0;  //Időszinkron megérkezésének ideje
int lastMeasurment = 0;
bool sentLastPacket = false; //Ha 50% fölött van az elkészített csomagok száma akkor minden másodikat küldi el
MeasurementBuffer buffer;
TelemetryBuffer telemetryBuffer;
TelecommandBuffer telecommandBuffer;
TelemetryBuffer technicalResponseBuffer;
byte receivedData[8];
byte responseData[16];
unsigned long unixTime = 0;

bool sensor0available = false;
bool sensor1available = false;
bool sensor2available = false;


//------------------ Targeted measurement variables
#define MAX_SAMPLES_IN_CIRCLE 32
//24 * 60 * 60 * 1000
#define MILLISECONDS_IN_A_DAY 86400000
//60*60*1000
#define MILLISECONDS_IN_AN_HOUR 3600000

//Required parameters set by telecommands
uint32_t spinPeriodMs = 10000;
int8_t spinPeriodDrift = 0;//ms / day
uint16_t orbitPeriodSec = 5280;
int16_t orbitPeriodDrift = 0;// ms / day -512..511
float spinDriftDegPerHour = 0;
bool spinDirCW = true;
bool orbitDirCW = true;
uint8_t numSamplesInCircle = 12;//Might need multiple spin revolutions to collect all data points - Min 1, max MAX_SAMPLES_IN_CIRCLE 
bool useSpinCompensation = true;
uint16_t spinCompensationZeroPoint = 0; //0..359 - if set to 120, 0 will be added to spin pos at 120°orbit point, 60° will be added to spin when orbit reaches 180°, etc
bool binarySpinCompensation = false;//To abruptly add 180° offset to the spin at the orbit's specified point and the one 180° opposite of it
uint16_t singleTargetPosition = 270;//0..359, the singular target to lock onto
//Set automatically to the current unix time if requested or no previous zero point is set
uint32_t spinZeroPoint = 1733576734;//UNIX time in seconds
uint32_t orbitZeroPoint = 1733576734;//UNIX time in seconds

//Parameters calculated automatically when integrationTime or orbit parameters ure updated
uint8_t targetPositionNumbers[MAX_SAMPLES_IN_CIRCLE];//Stores position numbers in the order of taking. Can be converted to degrees with targetNumberToDegrees()
int halfMeasurementTime = 1500;//~500-2500ms
uint16_t halfMeasurementLengthDegrees;//So we can trigger the measurement this much before the target and the middle will go through the target
uint8_t numRevolutionsPerBatch = 2;
uint8_t currentTargetIndex = 0;
uint16_t spinPosAtLastMeasurementCheck = 0;
bool targetingSetupCompleted = false;

bool initialSensorConfigCompleted = false;
//------------------ End of targeted measurement variables


int encodeFloat(float num) 
{
  int newNum = 0;

  float logNum = log(num+1) / log(2+0.25*fnBase); //Vesszük a szám b alapú logaritmusát

  logNum += pow(2, -1 * fnDecPoint - 1); //Érték hozzáadása hogy floorból round legyen

  for (float i = fnLength - fnDecPoint - 1; i >= -1 * fnDecPoint; i--) //Egyszerűsítjük a számot
  {
    if (logNum > pow(2, i))//Megnézzük hogy milyen legyen a következő számjegy
    {
      newNum = newNum << 1 | 1;
      logNum -= pow(2, i);
    }
    else
    {
      newNum = newNum << 1;
    }
  }

  return newNum;
}

float decodeFloat(int num) 
{
  float newNum = num / pow(2, fnDecPoint);
  return pow(2+0.25*fnBase, newNum)-1;
}

unsigned long runtimeToUnixTime(long time)
{
  return timeSync + (time-tsReceiveTime)/1000;
}

uint64_t msSinceUnixDate(uint64_t unixDate, uint64_t nowMs){//EG: date: 1733570000 timeSync: 1733576000; -> 6000 seconds until synchronising time -> 6 000 000 ms.  Then we add the Ms elapsed since the onboard time of timeSync
  if(timeSync == -1)return 0;
  //Serial.println("[MCU] TimeSyncDate: " + String(timeSync) + "   unixDate: " + String(unixDate));
  return (uint64_t(timeSync - unixDate))*1000 + nowMs - tsReceiveTime;
}

unsigned long runtimeToDailyTime(long time)
{
  return (timeSync + (time-tsReceiveTime)/1000) % 86400;
}

bool measure()
{
  bool measurementCalled;
  switch(measurmentMode)
  {
    case 0:
      measurementCalled = timedMeasurment();
      break;
    case 1:
      measurementCalled = needsSweepMeasurement(millis());
      if(measurementCalled)takeMeasurment();//The timedMeasurment() calls the function internally, but the targeted only checks
      break;
    case 2:
      measurementCalled = needsTargetedMeasurement(millis());
      if(measurementCalled)takeMeasurment();
      break;
    case 5:
      measurementCalled = timedMeasurment();
      break;
  }

  return measurementCalled;
}

bool timedMeasurment()
{
  bool measurementCalled = false;

  if(millis() > timedMeasurmentDelay+lastMeasurment)
  {
    if(millis() - lastMeasurment > 2*timedMeasurmentDelay)lastMeasurment = millis();
    else lastMeasurment += timedMeasurmentDelay;

    //Serial.println("[MCU] " + String(timedMeasurmentDelay) + " elapsed. Taking measurement");
    //lastMeasurment = millis();
    takeMeasurment();
    measurementCalled = true;
  }

  return measurementCalled;
}

float incrementExponentially(float num) {
  if(num >= 20000)return 0;
  if (num < 1.0f) {
    return num + 0.1f;  // Linear increment for numbers < 1
  } else {
    int power = static_cast<int>(std::pow(10, std::floor(std::log10(num))));
    return (num / power + 1) * power;  // Exponential stepping
  }
}
float dummyPayload = 0;
void takeMeasurment()
{
  if(!initialSensorConfigCompleted)configSensorFromEEPROM();
  //[DEBUG]
  long measurementStart = millis();

  //Sorrend: ABCDEF GHRISJ TUVWKL
  float data[18];
  uint8_t sensorTemps[3];

  digitalWrite(LED_BUILTIN, HIGH);
  //Tracking sensor status
  Wire1.beginTransmission(AS726X_ADDR);
  byte error = Wire1.endTransmission();
  sensor0available = error == 0;

  sensor.chooseMaster();

  byte dev_sel = sensor.virtualReadRegister(AS72XX_DEV_SEL);
  sensor1available = dev_sel >> 4 & 1;
  sensor2available = dev_sel >> 5 & 1;
  //Serial.println("[MCU] Dev-sel: " + String(dev_sel));//DEBUG

  sensor.takeMeasurements();
  sensorTemps[0] = sensor.getTemperature();
  float calibrationMultiplier = getCalibrationMultiplier(sensorTemps[0]);

  data[8] = sensor.getCalibratedR() * calibrationMultiplier * getSpectrumMultiplier(8);
  data[10] = sensor.getCalibratedS() * calibrationMultiplier * getSpectrumMultiplier(10);
  data[12] = sensor.getCalibratedT() * calibrationMultiplier * getSpectrumMultiplier(12);
  data[13] = sensor.getCalibratedU() * calibrationMultiplier * getSpectrumMultiplier(13);
  data[14] = sensor.getCalibratedV() * calibrationMultiplier * getSpectrumMultiplier(14);
  data[15] = sensor.getCalibratedW() * calibrationMultiplier * getSpectrumMultiplier(15);

  sensor.chooseFirstSlave();
  //sensor.takeMeasurements();
  sensorTemps[1] = sensor.getTemperature();
  calibrationMultiplier = getCalibrationMultiplier(sensorTemps[1]);

  data[6] = sensor.getCalibratedG() * calibrationMultiplier * getSpectrumMultiplier(6);
  data[7] = sensor.getCalibratedH() * calibrationMultiplier * getSpectrumMultiplier(7);
  data[9] = sensor.getCalibratedI() * calibrationMultiplier * getSpectrumMultiplier(9);
  data[11] = sensor.getCalibratedJ() * calibrationMultiplier * getSpectrumMultiplier(11);
  data[16] = sensor.getCalibratedK() * calibrationMultiplier * getSpectrumMultiplier(16);
  data[17] = sensor.getCalibratedL() * calibrationMultiplier * getSpectrumMultiplier(17);

  sensor.chooseSecondSlave();
  //sensor.takeMeasurements();
  sensorTemps[2] = sensor.getTemperature();
  calibrationMultiplier = getCalibrationMultiplier(sensorTemps[2]);

  data[0] = sensor.getCalibratedA() * calibrationMultiplier * getSpectrumMultiplier(0);
  data[1] = sensor.getCalibratedB() * calibrationMultiplier * getSpectrumMultiplier(1);
  data[2] = sensor.getCalibratedC() * calibrationMultiplier * getSpectrumMultiplier(2);
  data[3] = sensor.getCalibratedD() * calibrationMultiplier * getSpectrumMultiplier(3);
  data[4] = sensor.getCalibratedE() * calibrationMultiplier * getSpectrumMultiplier(4);
  data[5] = sensor.getCalibratedF() * calibrationMultiplier * getSpectrumMultiplier(5);
 
  buffer.addData(millis(), data, sensorTemps);

  //Serial.println("Measurement taken. " + String(millis() - measurementStart) + "ms, IT: " + String(sensor.getIntegrationTime()));
  digitalWrite(LED_BUILTIN, LOW);
}

void processData(){
  switch(measurmentMode){
    case 0: processData_mode0();break;
    case 1: processData_mode1();break;
    case 2: processData_mode0();break;//Only the timing is really different
    case 5: processData_mode5();break;
    default: break;
  }
}

void processData_mode0()
{
  float rData[requiredMeasurmentCount][18];
  long time;//Since boot

    //Lekéri a szükséges x mérés adatait
    for(byte i = 0; i < requiredMeasurmentCount; i++)
    {
      Measurement d;
      d = buffer.getData();

      for(byte j = 0; j < 18; j++)
      {
        rData[i][j] = d.sensorReadings[j];
      }

      if(i == 0)
      {
        time = d.measurementTime;
      }

    }

    //Csoportosítja az adatokat az átlagolt csatornák között

    float groupedData[requiredMeasurmentCount][18];
    byte gdLength = 0;

    float sum[requiredMeasurmentCount];
    byte count = 0;

    for(byte i = 0; i < 18; i++)
    {
      if(enabledSpectrums[i])
      {
        if(spectrumGrouping[i])
        {
          for(byte j = 0; j < requiredMeasurmentCount; j++)
          {
            groupedData[j][gdLength] = (sum[j] + rData[j][i]) / (count + 1);
            sum[j] = 0;
          } 
          gdLength++;
          count = 0;
        }
        else
        {
          for(byte j = 0; j < requiredMeasurmentCount; j++)
          {
            sum[j] += rData[j][i];
          }
          count++;

        }
      }
    }

    //A több mérés blokk megegyező csatornáit összevonja egy tömbbé (vagy megegyező csatorna csoportjait ha átlagolva vannak)
    byte cdLength = 0;//CombinedData

    if(toAverageMeasurments)
    {
      cdLength = gdLength;
    }
    else
    {
      cdLength = requiredMeasurmentCount*gdLength;
    }

    float combinedData[cdLength];
    //Átlagolja a méréseket
    if(toAverageMeasurments)
    {
      for(byte i = 0; i < gdLength; i++)
      {
        float sum = 0;
        for(byte j = 0; j < requiredMeasurmentCount; j++)
        {
          sum += groupedData[j][i];
        }
        combinedData[i] = sum / requiredMeasurmentCount;
      }
    }
    //A méréseket külön-külön leküldi
    else
    {
      for(byte i = 0; i < requiredMeasurmentCount; i++)
      {
        for(byte j = 0; j < gdLength; j++)
        {
          combinedData[gdLength*i + j] = groupedData[i][j];
        }
      }
    }

    //Tömöríti az adatokat
    int compressedData[cdLength];
    for(byte i = 0; i < cdLength; i++)
    {
      compressedData[i] = encodeFloat(combinedData[i]);
    }

    //Megformázza az adatokat

    BitChain processedData[packetSize];
    byte pdIndex = 0;
    byte counterLength = 0;
    if(packetSize != 1)
    {
      counterLength = ceil(log(packetSize-1)/log(2));
    }

    processedData[0].write(0, 1);//Nem technikai üzenet
    processedData[0].write(1, 1);//Első üzenet
    processedData[0].write(runtimeToDailyTime(time), 17);

    for(byte i = 0; i < cdLength; i++)
    {
      byte overFlow = processedData[pdIndex].write(compressedData[i], fnLength);

      if(overFlow && pdIndex < packetSize-1)
      {
        byte overFlownData = compressedData[i] & ((int)pow(2, overFlow) - 1);
        pdIndex++;

        processedData[pdIndex].write(0,1);//Nem technikai üzenet
        processedData[pdIndex].write(0, 1);//Több adatcsomagból álló üzenet.
        processedData[pdIndex].write(pdIndex-1, counterLength);
        processedData[pdIndex].write(overFlownData, overFlow);
      }
    }

    //Elküldi a csoamgot

    nthMessagePointer = (nthMessagePointer + 1) % keepEveryNthMessage;//Throw away every message except every Nth
    if(nthMessagePointer != 0)return;

    byte remainingSpacePercentage = telemetryBuffer.remainingSpacePercentage();

    if(remainingSpacePercentage >= 50)
    {
      for(byte i = 0; i < pdIndex+1; i++)
      {
        telemetryBuffer.addData(processedData[i].adatok);
      }
    }
    else if(remainingSpacePercentage >= 10 && !sentLastPacket)
    {
      for(byte i = 0; i < pdIndex+1; i++)
      {
        telemetryBuffer.addData(processedData[i].adatok);
      }
      sentLastPacket = true;
    }
    else
    {
      sentLastPacket = false;
    }
}

void processData_mode1()
{
  //Serial.println("[MCU] processData_mode1() called. numSamplesInCircle: " + String(numSamplesInCircle));
  float rData[numSamplesInCircle][18];
  long time;//Since boot

  //Lekéri a szükséges x mérés adatait
  for(byte i = 0; i < numSamplesInCircle; i++)
  {
    Measurement d;
    d = buffer.getData();
    uint8_t positionNumber = targetPositionNumbers[i];

    for(byte j = 0; j < 18; j++)
    {
      rData[positionNumber][j] = d.sensorReadings[j];
    }

    if(positionNumber == 0)
    {
      time = d.measurementTime;
    }
  }

  //Csoportosítja az adatokat az összeolvasztott csatornák között

  float groupedData[numSamplesInCircle][18];
  byte gdLength = 0;

  float sum[numSamplesInCircle];
  byte count = 0;

  for(byte i = 0; i < 18; i++)
  {
    if(enabledSpectrums[i])
    {
      if(spectrumGrouping[i])
      {
        for(byte j = 0; j < numSamplesInCircle; j++)
        {
          groupedData[j][gdLength] = (sum[j] + rData[j][i]) / (count + 1);
          sum[j] = 0;
        } 
        gdLength++;
        count = 0;
      }
      else
      {
        for(byte j = 0; j < numSamplesInCircle; j++)
        {
          sum[j] += rData[j][i];
        }
        count++;
      }
    }
  }

  //A több mérés blokk megegyező csatornáit összevonja egy tömbbé (vagy megegyező csatorna csoportjait ha átlagolva vannak)
  byte cdLength = 0;//CombinedData
  cdLength = numSamplesInCircle*gdLength;

  float combinedData[cdLength];//A0,B0,CD0,EFG0,J0,A1,B1,CD1,EFG1,A2,...
  //Átlagolja a méréseket
  //A méréseket külön-külön leküldi
  for(byte i = 0; i < numSamplesInCircle; i++)
  {
    for(byte j = 0; j < gdLength; j++)
    {
      combinedData[gdLength*i + j] = groupedData[i][j];
    }
  }

  //Tömöríti az adatokat
  int compressedData[cdLength];
  for(byte i = 0; i < cdLength; i++)
  {
    compressedData[i] = encodeFloat(combinedData[i]);
  }

  //Megformázza az adatokat

  BitChain processedData[packetSize];
  byte pdIndex = 0;
  byte counterLength = 0;
  if(packetSize != 1)
  {
    counterLength = ceil(log(packetSize-1)/log(2));
  }

  processedData[0].write(0, 1);//Nem technikai üzenet
  processedData[0].write(1, 1);//Első üzenet
  processedData[0].write(runtimeToDailyTime(time), 17);

  for(byte i = 0; i < cdLength; i++)
  {
    byte overFlow = processedData[pdIndex].write(compressedData[i], fnLength);

    if(overFlow && pdIndex < packetSize-1)//TODO DEBUG Won't processedData[0].write(0,1) be called twice? Once outside loop, once inside
    {
      byte overFlownData = compressedData[i] & ((int)pow(2, overFlow) - 1);
      pdIndex++;

      processedData[pdIndex].write(0,1);//Nem technikai üzenet
      processedData[pdIndex].write(0, 1);//Több adatcsomagból álló üzenet.
      processedData[pdIndex].write(pdIndex-1, counterLength);
      processedData[pdIndex].write(overFlownData, overFlow);
    }
  }

  //Elküldi a csoamgot

  //Serial.println("[MCU] mm1 data packet compiled " + String(keepEveryNthMessage)); //DEBUG

  nthMessagePointer = (nthMessagePointer + 1) % keepEveryNthMessage;//Throw away every message except every Nth
  if(nthMessagePointer != 0)return;

  byte remainingSpacePercentage = telemetryBuffer.remainingSpacePercentage();

  if(remainingSpacePercentage >= 50)
  {
    for(byte i = 0; i < pdIndex+1; i++)
    {
      telemetryBuffer.addData(processedData[i].adatok);
    }
  }
  else if(remainingSpacePercentage >= 10 && !sentLastPacket)
  {
    for(byte i = 0; i < pdIndex+1; i++)
    {
      telemetryBuffer.addData(processedData[i].adatok);
    }
    sentLastPacket = true;
  }
  else
  {
    sentLastPacket = false;
  }
}

union FloatIntUnion {
    float f;
    int i;
};//Some black magic

int floatToIntByBits(float value) {
    FloatIntUnion converter;
    converter.f = value;
    return converter.i;
}

float intToFloatByBits(int value) {
    FloatIntUnion converter;
    converter.i = value;
    return converter.f;
}


void processData_mode5()
{
/*byte 0-3	    4-7	   8-11    12-15
0 Time&stuff    Temps       
1 head + number	CH0	   CH1     CH2
2 head + number	CH3    CH4     CH5
3 head + number	CH6    CH7     CH8
4 head + number	CH9    CH10    CH11
5 head + number	CH12   CH13    CH14
6 head + number	CH15   CH16    CH17*/

  int packetCount = 7;
  float rData[18];
  uint8_t sensorTemps[3];
  long time;//Since boot

  //Lekéri a mérés adatait
  Measurement d;
  d = buffer.getData();

  for(byte j = 0; j < 18; j++)
  {
    rData[j] = d.sensorReadings[j];
  }

  for(byte j = 0; j < 3; j++)
  {
    sensorTemps[j] = d.sensorTemps[j]; 
  }

  time = d.measurementTime;
  //Megformázza az adatokat

  BitChain processedData[packetCount];//

  processedData[0].write(0, 1);//Nem technikai üzenet
  processedData[0].write(1, 1);//Több adatcsomagból álló üzenet
  processedData[0].write(runtimeToDailyTime(time), 17);//22 bit / 32
  processedData[0].write(0, 13);//10 üres bit, hogy jó helyen folytatódjon
  processedData[0].write((int) analogReadTemp(), 8);
  processedData[0].write(sensor0available | (sensor1available << 1) | (sensor2available << 2), 8);

  for(int i = 0; i < 3; i++){
    processedData[0].write(sensorTemps[i], 8);
  }

  for(byte i = 1; i <= 6; i++)
  {
    processedData[i].write(0, 1);//Nem technikai üzenet
    processedData[i].write(0, 1);//Több adatcsomagból álló üzenet (nem az első adatcsomag)
    processedData[i].write(i-1, 3);//sorszám
    processedData[i].write(i, 32-5);//padding
    for(int j = 0; j < 3; j++){
      processedData[i].write(floatToIntByBits(rData[(i-1)*3+j]), 32);
    }
  }

  //Elküldi a csoamgot

  nthMessagePointer = (nthMessagePointer + 1) % keepEveryNthMessage;//Throw away every message except every Nth
  if(nthMessagePointer != 0)return;

  byte remainingSpacePercentage = telemetryBuffer.remainingSpacePercentage();

  if(remainingSpacePercentage >= 50)
  {
    for(byte i = 0; i < packetCount; i++)
    {
      telemetryBuffer.addData(processedData[i].adatok);
    }
  }
  else if(remainingSpacePercentage >= 10 && !sentLastPacket)
  {
    for(byte i = 0; i < packetCount; i++)
    {
      telemetryBuffer.addData(processedData[i].adatok);
    }
    sentLastPacket = true;
  }
  else
  {
    sentLastPacket = false;
  }
}

void makeTechnicalResponse()
{
  Wire1.beginTransmission(AS726X_ADDR);
  byte error = Wire1.endTransmission();
  sensor0available = error == 0;
  sensor.chooseMaster();
  byte dev_sel = sensor.virtualReadRegister(AS72XX_DEV_SEL);
  sensor1available = dev_sel >> 4 & 1;
  sensor2available = dev_sel >> 5 & 1;


  BitChain technicalData;

  technicalData.write(1,1); //technikai válasz
  technicalData.write(0,1); //Nem 'response' telecommandra
  technicalData.write(runtimeToDailyTime(millis()),17);
  technicalData.write(sensor0available, 1);
  technicalData.write(sensor1available, 1);
  technicalData.write(sensor2available, 1);
  technicalData.write(EEPROM.read(EEPROM_SETTED) == EEPROM_TRUE, 1);
  technicalData.write(telemetryBuffer.remainingSpacePercentage(),7);
  technicalData.write(fnLength,5);
  technicalData.write(fnDecPoint,5);
  technicalData.write(fnBase,3);
  technicalData.write(requiredMeasurmentCount,4);
  
  for(byte i = 0; i < 18; i++)
  {
    technicalData.write(enabledSpectrums[i],1);
  }
  for(byte i = 0; i < 18; i++)
  {
    technicalData.write(spectrumGrouping[i],1);
  }

  technicalData.write(toAverageMeasurments,1);
  technicalData.write(packetSize,4);
  technicalData.write(sensor.getGain(),2);
  technicalData.write(sensor.getMeasurementMode(),2);
  technicalData.write(sensor.getIntegrationTime(),8);
  technicalData.write(sensor.getTemperature(),8);

  technicalData.write(measurmentMode, 3);

  switch(measurmentMode)
  {
    case 0: technicalData.write(timedMeasurmentDelay,16);  break;
    case 1: {
      technicalData.write(numSamplesInCircle, 5);
      technicalData.write(useSpinCompensation, 1);
      technicalData.write(spinCompensationZeroPoint, 9);
      technicalData.write(binarySpinCompensation, 1);
    }break;
    case 2: {
      technicalData.write(useSpinCompensation, 1);
      technicalData.write(binarySpinCompensation, 1);
      technicalData.write(singleTargetPosition, 9);
    }
    case 5: technicalData.write(timedMeasurmentDelay,16);  break;  
    default: technicalData.write(0,16);  break;  
  }

  for(byte i = 0; i < 16; i++)
  {
    responseData[i] = technicalData.adatok[i];   
  }

  technicalResponseBuffer.addData(responseData);

  //Serial.println("[MCU] Technical data created!");//DEBUG
}

void serialPrintData()
{
  //Serial.println("========================================================================");
  Serial.print("[MCU] AS72651 - ");
  sensor.chooseMaster();
  sensor.takeMeasurements();
  long currentMillis = millis();
  Serial.print("Time: "); Serial.print(runtimeToDailyTime(currentMillis)); Serial.print("."); Serial.println((currentMillis % 1000)/100);
  float calM = getCalibrationMultiplier(sensor.getTemperature());
  Serial.print("Heat compensation: ");Serial.println(calM);
  Serial.print("[R: "); Serial.print(sensor.getCalibratedR()*calM,8); Serial.println("], ");
  Serial.print("[S: "); Serial.print(sensor.getCalibratedS()*calM,8); Serial.println("], ");
  Serial.print("[T: "); Serial.print(sensor.getCalibratedT()*calM,8); Serial.println("], ");
  Serial.print("[U: "); Serial.print(sensor.getCalibratedU()*calM,8); Serial.println("], ");
  Serial.print("[V: "); Serial.print(sensor.getCalibratedV()*calM,8); Serial.println("], ");
  Serial.print("[W: "); Serial.print(sensor.getCalibratedW()*calM,8); Serial.println("], ");
  //Serial.println();

  Serial.print("[MCU] AS72652 - ");
  sensor.chooseFirstSlave();
  sensor.takeMeasurements();
  currentMillis = millis();
  Serial.print("Time: "); Serial.print(runtimeToDailyTime(currentMillis)); Serial.print("."); Serial.println((currentMillis % 1000)/100);
  calM = getCalibrationMultiplier(sensor.getTemperature());
  Serial.print("Heat compensation: ");Serial.println(calM);
  Serial.print("[G: "); Serial.print(sensor.getCalibratedG()*calM,8); Serial.println("], ");
  Serial.print("[H: "); Serial.print(sensor.getCalibratedH()*calM,8); Serial.println("], ");
  Serial.print("[I: "); Serial.print(sensor.getCalibratedI()*calM,8); Serial.println("], ");
  Serial.print("[J: "); Serial.print(sensor.getCalibratedJ()*calM,8); Serial.println("], ");
  Serial.print("[K: "); Serial.print(sensor.getCalibratedK()*calM,8); Serial.println("], ");
  Serial.print("[L: "); Serial.print(sensor.getCalibratedL()*calM,8); Serial.println("], ");
  //Serial.println();

  Serial.print("[MCU] AS72653 - ");
  sensor.chooseSecondSlave();
  sensor.takeMeasurements();
  currentMillis = millis();
  Serial.print("Time: "); Serial.print(runtimeToDailyTime(currentMillis)); Serial.print("."); Serial.println((currentMillis % 1000)/100);
  calM = getCalibrationMultiplier(sensor.getTemperature());
  Serial.print("Heat compensation: ");Serial.println(calM);
  Serial.print("[A: "); Serial.print(sensor.getCalibratedA()*calM,8); Serial.println("], ");
  Serial.print("[B: "); Serial.print(sensor.getCalibratedB()*calM,8); Serial.println("], ");
  Serial.print("[C: "); Serial.print(sensor.getCalibratedC()*calM,8); Serial.println("], ");
  Serial.print("[D: "); Serial.print(sensor.getCalibratedD()*calM,8); Serial.println("], ");
  Serial.print("[E: "); Serial.print(sensor.getCalibratedE()*calM,8); Serial.println("], ");
  Serial.print("[F: "); Serial.print(sensor.getCalibratedF()*calM,8); Serial.println("], ");
  //Serial.println(sensor.getTemperature());
  //Serial.println();
}

void setup() {
  set_sys_clock_khz(100 * 1000, true);
  Serial.begin(115200); //DEBUG
  pinMode(LED_BUILTIN, OUTPUT);
  //delay(2000); //DEBUG
  Wire1.setSDA(SDA1_PIN);
  Wire1.setSCL(SCL1_PIN);
  Wire1.begin();
  sensor.begin(Wire1);
  //delay(100);//Wait a moment for the sensor to boot - See github issue #10
  EEPROM.begin(EEPROM_SIZE);
  readParamsFromEEPROM();
  makeTechnicalResponse();
  if(measurmentMode == 1 || measurmentMode == 2){
    createCommandResponse15();//Return trajectory info
    createCommandResponse19();//Return spinDriftDegPerHour
  }
  
  //OBC
  Wire.setSDA(SDA_PIN);  // SDA pin beállítása
  Wire.setSCL(SCL_PIN);  // SCL pin beállítása
  Wire.begin(SLAVE_ADDRESS); // I2C busz inicializálása slave módban
  Wire.onReceive(receiveEvent); // Adat fogadás esemény kezelése
  Wire.onRequest(requestEvent); // Adat kérés esemény kezelése
}

//long lastDebugTick = 0; //DEBUG

void loop() 
{
  if(millis() > 2073600000)reboot(nullptr);//millis timers would start overflowing after ~24.8 days. Safer to just reboot in the unlikely case of running continuously for 25+ days
  while(telecommandBuffer.availableData()>0){
    Telecommand result = telecommandBuffer.getData();
    handleTelecommand(result.data);
  }
  if(measure() && ((measurmentMode != 1 && buffer.availableData() >= requiredMeasurmentCount) || (measurmentMode == 1 && buffer.availableData() >= numSamplesInCircle)))
  {
    if(timeSync > -1)
    {
      processData();
      //Serial.println("[MCU] Datapacket created ");//DEBUG
    }
  }

  //[DEBUG]
  //serialPrintData();
  /*if(lastDebugTick != (millis()/100)){//[DEBUG]
    lastDebugTick = millis()/100;
    Serial.println("[MCU] Orbit position: " + String(getOrbitPos(millis())) + "°  spin position: " + String(getSpinPos(millis(), false)) + "°");
  }*/
}

bool configSensorFromEEPROM(){
  int targetGain = EEPROM.read(EEPROM_SENSOR_SET_GAIN);
  int targetMeasurmentMode = EEPROM.read(EEPROM_SENSOR_MEASURMENT_MODE);
  int targetIT = EEPROM.read(EEPROM_SENSOR_INTEGRATION_TIME);
  sensor.setGain(targetGain);
  sensor.setMeasurementMode(targetMeasurmentMode);
  sensor.setIntegrationTime(targetIT);
  int actualGain = sensor.getGain();
  int actualMeasurmentMode = sensor.getMeasurementMode();
  int actualIT = sensor.getIntegrationTime();
  bool success = actualGain == targetGain && actualMeasurmentMode == targetMeasurmentMode && actualIT == targetIT;
  if(success){
    initialSensorConfigCompleted = true;
  }
  return success;
}

void readParamsFromEEPROM()
{
  if(EEPROM.read(EEPROM_SETTED) == EEPROM_TRUE)//random szám 0b00001111
  {
    fnLength = EEPROM.read(EEPROM_FNLENGTH);
    fnDecPoint = EEPROM.read(EEPROM_FNDECPOINT);
    fnBase = EEPROM.read(EEPROM_FNBASE);
    measurmentMode = EEPROM.read(EEPROM_MEASURMENT_MODE);
    requiredMeasurmentCount = EEPROM.read(EEPROM_REQUIRED_MEASURMENT_COUNT);
    toAverageMeasurments = EEPROM.read(EEPROM_TO_AVERAGE_MEASURMENTS);
    packetSize = EEPROM.read(EEPROM_PACKET_SIZE);

    configSensorFromEEPROM();

    int tmDelay = 0; //timedMeasurmentDelay
    for(byte i = 0; i < 4; i++)
    {
      tmDelay = tmDelay << 8 | EEPROM.read(EEPROM_TIMED_MEASURMENT_DELAY + i);
    }
    timedMeasurmentDelay = tmDelay;

    for(byte i = 0; i < 18; i++)
    {
      enabledSpectrums[i] = EEPROM.read(EEPROM_ENABLED_SPECTRUMS + i);
      spectrumGrouping[i] = EEPROM.read(EEPROM_SPECTRUM_GROUPING + i);
    }

    keepEveryNthMessage = EEPROM.read(EEPROM_KEEP_EVERY_NTH_MESSAGE);

    EEPROM.get(EEPROM_SPIN_PERIOD_MS, spinPeriodMs);
    EEPROM.get(EEPROM_SPIN_PERIOD_DRIFT, spinPeriodDrift);
    EEPROM.get(EEPROM_ORBIT_PERIOD_SEC, orbitPeriodSec);
    EEPROM.get(EEPROM_ORBIT_PERIOD_DRIFT, orbitPeriodDrift);
    spinDirCW = EEPROM.read(EEPROM_SPIN_DIR_CW);
    orbitDirCW = EEPROM.read(EEPROM_ORBIT_DIR_CW);
    numSamplesInCircle = EEPROM.read(EEPROM_NUM_SAMPLES_IN_CIRCLE);
    useSpinCompensation = EEPROM.read(EEPROM_USE_SPIN_COMPENSATION);
    EEPROM.get(EEPROM_SPIN_COMPENSATION_ZERO_POINT, spinCompensationZeroPoint);
    binarySpinCompensation = EEPROM.read(EEPROM_BINARY_SPIN_COMPENSATION);
    EEPROM.get(EEPROM_SINGLE_TARGET_POSITION, singleTargetPosition);
    EEPROM.get(EEPROM_SPIN_ZERO_POINT, spinZeroPoint);
    EEPROM.get(EEPROM_ORBIT_ZERO_POINT, orbitZeroPoint);
    EEPROM.get(EEPROM_SPIN_DRIFT_DEG_PER_HOUR, spinDriftDegPerHour);


    //Serial.println("[MCU] reading from EEPROM is successful");//DEBUG
  }else{
    initialSensorConfigCompleted = true;//We don't need config from eeprom, so we can consider it completed
  }
}

void writeParamsToEEPROM()
{
  EEPROM.write(EEPROM_FNBASE, fnBase);
  EEPROM.write(EEPROM_FNLENGTH,fnLength);
  EEPROM.write(EEPROM_FNDECPOINT,fnDecPoint);
  EEPROM.write(EEPROM_MEASURMENT_MODE,measurmentMode);
  EEPROM.write(EEPROM_REQUIRED_MEASURMENT_COUNT,requiredMeasurmentCount);
  EEPROM.write(EEPROM_TO_AVERAGE_MEASURMENTS,toAverageMeasurments);
  EEPROM.write(EEPROM_PACKET_SIZE,packetSize);
  EEPROM.write(EEPROM_SENSOR_SET_GAIN,sensor.getGain());
  EEPROM.write(EEPROM_SENSOR_MEASURMENT_MODE,sensor.getMeasurementMode());
  EEPROM.write(EEPROM_SENSOR_INTEGRATION_TIME,sensor.getIntegrationTime());

  for(byte i = 0; i < 4; i++)
  {
    EEPROM.write(EEPROM_TIMED_MEASURMENT_DELAY + 3 - i, (timedMeasurmentDelay >> 8*i) & 0xFF);
  }

  for(byte i = 0; i < 18; i++)
  {
    EEPROM.write(EEPROM_ENABLED_SPECTRUMS + i, enabledSpectrums[i]);
    EEPROM.write(EEPROM_SPECTRUM_GROUPING + i, spectrumGrouping[i]);
  }

  EEPROM.write(EEPROM_KEEP_EVERY_NTH_MESSAGE, keepEveryNthMessage);

  EEPROM.put(EEPROM_SPIN_PERIOD_MS, spinPeriodMs);
  EEPROM.put(EEPROM_SPIN_PERIOD_DRIFT, spinPeriodDrift);
  EEPROM.put(EEPROM_ORBIT_PERIOD_SEC, orbitPeriodSec);
  EEPROM.put(EEPROM_ORBIT_PERIOD_DRIFT, orbitPeriodDrift);
  EEPROM.put(EEPROM_SPIN_DIR_CW, spinDirCW);
  EEPROM.put(EEPROM_ORBIT_DIR_CW, orbitDirCW);
  EEPROM.put(EEPROM_NUM_SAMPLES_IN_CIRCLE, numSamplesInCircle);
  EEPROM.put(EEPROM_USE_SPIN_COMPENSATION, useSpinCompensation);
  EEPROM.put(EEPROM_SPIN_COMPENSATION_ZERO_POINT, spinCompensationZeroPoint);
  EEPROM.put(EEPROM_BINARY_SPIN_COMPENSATION, binarySpinCompensation);
  EEPROM.put(EEPROM_SINGLE_TARGET_POSITION, singleTargetPosition);
  EEPROM.put(EEPROM_SPIN_ZERO_POINT, spinZeroPoint);
  EEPROM.put(EEPROM_ORBIT_ZERO_POINT, orbitZeroPoint);
  EEPROM.put(EEPROM_SPIN_DRIFT_DEG_PER_HOUR, spinDriftDegPerHour);

  EEPROM.write(EEPROM_SETTED, EEPROM_TRUE);

  EEPROM.commit();

  //Serial.println("[MCU] parameters wrote to EEPROM");//DEBUG
}
