//#0 Létrehoz egy technikai választ az adott típusból
//1. byte: válasz típusa
bool requestTechnicalMessage(byte data[8])
{
  switch(data[1]){
    case 15: createCommandResponse15();break;//Return trajectory info
    case 19: createCommandResponse19();break;//Return spinDriftDegPerHour
    case 115: createCommandResponse115();break;//Return trajectory status
  }
  return false;
}

//#1 Kitörli az eepromot és rebootolja magát
//Nincs paraméter
bool eepromClearAndReboot(byte data[8])
{
  eepromClear(data);
  reboot(data);
  return false;
}

//#2 Rebootlolja magát
//Nincs paraméter
bool reboot(byte data[8])
{
  //DO NOT ACCESS data, IT IS PASSED AS nullptr SOMETIMES
  watchdog_reboot(0,SRAM_END,0);
  return false;
}

//#3 Szoftveresen rebootolja a szenzort
//Nincs paraméter
bool rebootSensor(byte data[8])
{
  sensor.softReset();
  return true;
}

//#4 Kitölri az eepromot
//Nincs paraméter
bool eepromClear(byte data[8])
{
  for(int i = 0; i < EEPROM_SIZE - 1; i++)
  {
    EEPROM.write(i,0);
  }
  EEPROM.commit();
  return false;
}

//#5 A megadott című eeprom helyre a megadott bájtot írja (legfeljebb 3-szor)
//Ha megtalálja hogy melyik paraméterhez tartozik akkor azt is átírja
//Paraméterek:
//1. bájt: hányszor írjon bájtot
//2. bájt: 1. írás címe; 3. bájt: írás értéke
//4. bájt: 2. írás címe; 5. bájt: írás értéke
//6. bájt: 3. írás címe; 7. bájt: írás értéke
bool eepromWrite(byte data[8])
{
  byte length = data[1] & 0b00000011; //0-3 ig
  for(byte i = 0; i < length; i++)
  {
    switch(data[2*i+2])
    {
      case EEPROM_FNLENGTH: fnLength = data[2*i+3];break; 
      case EEPROM_FNDECPOINT: fnDecPoint = data[2*i+3];break; 
      case EEPROM_FNBASE: fnBase = data[2*i+3];break;
      case EEPROM_MEASURMENT_MODE: measurmentMode = data[2*i+3];break;
      case EEPROM_REQUIRED_MEASURMENT_COUNT: requiredMeasurmentCount = data[2*i+3];break;
      case EEPROM_TO_AVERAGE_MEASURMENTS: toAverageMeasurments = data[2*i+3];break;
      case EEPROM_PACKET_SIZE: packetSize = data[2*i+3];break;
      case EEPROM_SENSOR_SET_GAIN: sensor.setGain(data[2*i+3]);break;
      case EEPROM_SENSOR_MEASURMENT_MODE: sensor.setMeasurementMode(data[2*i+3]);break;
      case EEPROM_SENSOR_INTEGRATION_TIME:  sensor.setIntegrationTime(data[2*i+3]);break;
    }//TODO: add more parameters to updatate the variable connected to the address

    EEPROM.write(data[2*i+2], data[2*i+3]);
  }

  EEPROM.commit();

  return false;
}

//#6 A megadott címet elküldi a szenzornak, és utána 1 bájt adatot küld
//Paraméterek:
//1. bájt: cím
//2. bájt: adat
bool relayToAsx(byte data[8])
{
  sensor.virtualWriteRegister(data[1], data[2]);
  return true;
}

//#7 Beállítja a szenzort a megadott paraméterek alapján
//Paraméterek:
//1. bájt: Gain
//2. bájt: Measurment mode
//3. bájt: Integration time
bool configSensor(byte data[8])
{
  sensor.setGain(data[1]);
  sensor.setMeasurementMode(data[2]);
  sensor.setIntegrationTime(data[3]);
  setupTargeting(false);//Targeting needs to adjust to the new integration time
  return true;
}

//#8 Beállítja a mérési módot, és annak paramétereit
//Paraméterek:
//1.bájt: mérési mód
//Többi bájt: Az adott mérési mód paraméterei
//Ha mérési mód:0
//    akkor: mérés gyakorisága: 2.bájt-tól 5. bájt-ig
//Ha mérési mód:1 [N] Máté lusta leírni
//    akkor:https://docs.google.com/spreadsheets/d/1ktxAy9JKZGtQRNbdboOkl_tYYjvsWYJcnA055t6Mg7A/edit?gid=0#gid=0
//Ha mérési mód:2
//    akkor: https://docs.google.com/spreadsheets/d/1ktxAy9JKZGtQRNbdboOkl_tYYjvsWYJcnA055t6Mg7A/edit?gid=0#gid=0
//Ha mérési mód:5
//    akkor: mérés gyakorisága: 2.bájt-tól 5. bájt-ig
bool configMeasurmentMode(byte data[8])
{

  measurmentMode = data[1];
  switch(measurmentMode)
  {
    case 0:{
      int tmDelay = 0; //timedMeasurmentDelay
      for(byte i = 0; i < 4; i++)
      {
        tmDelay = tmDelay << 8 | data[i+2];
      }
      timedMeasurmentDelay = tmDelay;
    }break;

    case 1:{
      numSamplesInCircle = (data[2] >> 3);
      requiredMeasurmentCount = numSamplesInCircle;
      useSpinCompensation = (data[2] >> 2) & 0b00000001;
      spinCompensationZeroPoint = ((((uint16_t) data[2]) & 0b00000011) << 7) | (data[3] >> 1);
      binarySpinCompensation = data[3] & 0b00000001;
      bool resetZero = data[4] >> 7;
      //Serial.println("Entering mode 'Sweep' with numSamples: " + String(numSamplesInCircle) +  " spinComp: " + String(useSpinCompensation) + " SC zero point: " + String(spinCompensationZeroPoint) + " BSC: " + String(binarySpinCompensation));//DEBUG
      setupTargeting(resetZero);
    }break;
    case 2:{
      useSpinCompensation = data[2] >> 7;
      spinCompensationZeroPoint = (((uint16_t) data[2] & 0b01111111) << 2)  | ((data[3] & 0b11000000) >> 6);
      binarySpinCompensation = (data[3] & 0b00100000) >> 5;
      singleTargetPosition = (((uint16_t) (data[3] & 0b00011111)) << 4) | ((data[4] & 0b11110000) >> 4);
      //Serial.println("Entering mode 'Targeted' with spinComp: " + String(useSpinCompensation) + " SC zero point: " + String(spinCompensationZeroPoint) + " BSC: " + String(binarySpinCompensation) + " single target pos: " + String(singleTargetPosition));//DEBUG
      setupTargeting(false);
    }break;

    case 5:{
      int tmDelay = 0; //timedMeasurmentDelay
      requiredMeasurmentCount = 1;
      for(byte i = 0; i < 4; i++)
      {
        tmDelay = tmDelay << 8 | data[i+2];
      }
      timedMeasurmentDelay = tmDelay;
    }break;
  }

  return true;
}

//#9 Beállítja a szenzorok csoportosítását
//Paraméterek:
//Itt BitChaint használunk:
//1.bájt-tól 18 bit: spektrumok kiválasztása
//Utána 18 bit: spektrumok csoportosítása
bool configSensorGrouping(byte data[8])
{
  BitChain bc(8);

  for(byte i = 0; i < 7; i++)
  {
    bc.adatok[i] = data[i+1];
  }

  for(byte i = 0; i < 18; i++)
  {
    enabledSpectrums[i] = bc.read(1);
  }

  for(byte i = 0; i < 18; i++)
  {
    spectrumGrouping[i] = bc.read(1);
  }

  return true;
}

//#10 Beállítja az adatfeldolgozás paraméterei (kivétel speltrum csoportosítás)
//Paraméterek:
//1. bájt: Egy adat mérete
//2. bájt: Egy adat tizedesvesszőjének helye
//3. bájt: Logaritmus alapja
//4. bájt: Szükséges mérések száma a feldolgozáshoz
//5. bájt: Átlagolja-e vagy nem a különböző méréseket
//6. bájt: Hány adatcsomagban legyen a feldolgozott adat
//7. bájt: Hány elkészített üzenet(csomag)onként küldjön el egyet ténylegesen. A többit eldobja
bool configProcessing(byte data[8])
{
  fnLength = data[1];
  fnDecPoint = data[2];
  fnBase = data[3];
  requiredMeasurmentCount = data[4];
  toAverageMeasurments = data[5];
  packetSize = data[6];
  keepEveryNthMessage = data[7];

  return true;
}

//#11 Visszaállítja a paramétereket az alapértelmezettre
bool setDefaultProcessing(byte data[8])
{
  fnLength = 6;
  fnDecPoint  = 2;
  fnBase = 0;

  measurmentMode = 0;
  timedMeasurmentDelay = 5000;
  requiredMeasurmentCount = 1;

  for(byte i = 0; i < 18; i++)
  {
    spectrumGrouping[i] = true;
    enabledSpectrums[i] = true;
  }

  toAverageMeasurments = false;
  packetSize = 1;

  keepEveryNthMessage = 1;

  sensor.setGain(3);
  sensor.setMeasurementMode(3);
  sensor.setIntegrationTime(50);

  spinPeriodMs = 10000;//
  spinPeriodDrift = 0;//ms / day
  orbitPeriodSec = 5280;
  orbitPeriodDrift = 0;// ms / day -512..511
  spinDirCW = true;
  orbitDirCW = true;
  numSamplesInCircle = 12;//Might need multiple spin revolutions to collect all data points - Min 1, max MAX_SAMPLES_IN_CIRCLE 
  useSpinCompensation = true;
  spinCompensationZeroPoint = 0; //0..359 - if set to 120, 0 will be added to spin pos at 120°orbit point, 60° will be added to spin when orbit reaches 180°, etc
  binarySpinCompensation = false;//To abruptly add 180° offset to the spin at the orbit's specified point and the one 180° opposite of it
  singleTargetPosition = 270;//0..359, the singular target to lock onto
  //Set automatically to the current unix time if requested or no previous zero point is set
  spinZeroPoint = 1733576734;//UNIX time in seconds
  orbitZeroPoint = 1733576734;//UNIX time in seconds
  setupTargeting(false);

  return true;
}

//#12 Az eepromból a megadott byteokat egy technikai válaszban leküldi
//Paraméterek:
//1. bájt: memóriacímek száma
//2-8. bájtok memóriacímei
//Válasz:
//0. bájt: 11000000 (technikai válasz és response)
//1-3. bájt: telecommand megérkezésének ideje
//4. bájt: telecommand kód
//5-10. bájt: memória tartalma
bool readFromEeprom(byte data[8])
{
  BitChain response;
  response.write(0b11000000,8);
  response.write(runtimeToDailyTime(millis()),24);
  response.write(data[0],8);

  uint8_t count = data[1];
  if(count > 6){count = 6;}
  for(uint8_t i = 2; i < count+2; i++)
  {
    response.write(EEPROM.read(data[i]),8);    
  }

  technicalResponseBuffer.addData(response.adatok);

  return false;
}

//#13 A szenzortól a megadott parancsokra érkező választ egy technikai válaszban leküldi
//Paraméterek:
//1. bájt: parancsok száma
//2-8. parancsok kódja
//Válasz:
//0. bájt: 11000000 (technikai válasz és response)
//1-3. bájt: telecommand megérkezésének ideje
//4. bájt: telecommand kód
//5-10. bájt: szenzor válasza
bool readFromAsx(byte data[8])
{
    BitChain response;
  response.write(0b11000000,8);
  response.write(runtimeToDailyTime(millis()),24);
  response.write(data[0],8);

  uint8_t count = data[1];
  if(count > 6){count = 6;}
  for(uint8_t i = 2; i < count+2; i++)
  {
    response.write(sensor.virtualReadRegister(data[i]),8);    
  }

  technicalResponseBuffer.addData(response.adatok);

  return false;
}

//#14 Kiüríti a telemetry buffert
//Nincs paraméter
bool clearTelemetry(byte data[8])
{
  telemetryBuffer.clear();
  return false;
}

//#15 beállítja a röppálya adatait
bool updateTrajectoryInfo(byte data[8]){
  spinPeriodMs = (((uint32_t)data[1]) << 12) | (((uint32_t)data[2]) << 4) | ((data[3] & 0b11110000) >> 4);
  long tempSpinPeriodDrift = ((data[3] & 0b00001111) << 4) | ((data[4] & 0b11110000) >> 4);
  if(tempSpinPeriodDrift >= 128)tempSpinPeriodDrift -= 256;
  spinPeriodDrift = tempSpinPeriodDrift;
  orbitPeriodSec = (((uint32_t)data[4] & 0b00001111) << 9) | (((uint32_t)data[5]) << 1) | (((uint32_t)data[6] & 0b10000000) >> 7);
  orbitPeriodDrift = (((uint32_t)data[6] & 0b01111111) << 3) | (((uint32_t)data[7]) >> 5);
  if(orbitPeriodDrift >= 512)orbitPeriodDrift -= 1024;
  spinDirCW = (data[7] >> 4) & 0b00000001;
  orbitDirCW = (data[7] >> 3) & 0b00000001;
  bool resetZeroPoints = (data[7] >> 2) & 0b00000001;
  //Serial.println("[MCU] updated trajectory info:   spinPeriod: " + String(spinPeriodMs) + "ms   spin drift: " + String(spinPeriodDrift) + "ms/day   orbit period: " + String(orbitPeriodSec) + "s   orbit drift: " + String(orbitPeriodDrift) + "ms/day");//Debug
  setupTargeting(resetZeroPoints);
  createCommandResponse15();
  return true;
}

void createCommandResponse15(){
  BitChain response;
  response.write(0b11000000,8);
  response.write(runtimeToDailyTime(millis()),24);
  response.write(15,8);//Command code
  response.write(spinPeriodMs,20);
  int spinPeriodDriftResponse = spinPeriodDrift;
  if(spinPeriodDriftResponse < 0)spinPeriodDriftResponse += 256;
  response.write(spinPeriodDriftResponse,8);
  response.write(orbitPeriodSec, 13);
  long orbitPeriodDriftResponse = orbitPeriodDrift;
  if(orbitPeriodDriftResponse < 0)orbitPeriodDriftResponse += 1024;
  response.write(orbitPeriodDriftResponse, 10);
  response.write(spinDirCW, 1);
  response.write(orbitDirCW, 1);
  response.write(spinZeroPoint, 32);
  //Serial.println("[MCU] Sending current trajectory info:   spinPeriod: " + String(spinPeriodMs) + "ms   spin drift: " + String(spinPeriodDrift) + "ms/day   orbit period: " + String(orbitPeriodSec) + "s   orbit drift: " + String(orbitPeriodDrift) + "ms/day   spinZeroPoint:" + String(spinZeroPoint));//Debug
  technicalResponseBuffer.addData(response.adatok);
}

//Transmits the current orbit and spin positions, and up to date (incl. acceleration) periods
void createCommandResponse115(){
  BitChain response;
  response.write(0b11000000,8);
  response.write(runtimeToDailyTime(millis()),24);
  response.write(115,8);//Command code
  long ms = millis();
  response.write(getSpinPos(ms, true), 9);//Absolute spin pos (w/o spin compensation)
  //Serial.print("[MCU] currentSpinPosAbs: " + String(getSpinPos(ms, true)));
  response.write(getSpinPos(ms, false), 9);//relative spin pos (if spin compensation enabled, othervise same as absolute)
  //Serial.print("   currentSpinPosRel: " + String(getSpinPos(ms, false)));
  response.write(getOrbitPos(ms), 9);//0..359
  //Serial.print("   currentOrbitPos: " + String(getOrbitPos(ms)));
  //Serial.println("[MCU] orbitZeroPoint: " + String(orbitZeroPoint));
  uint64_t msSinceZero = msSinceUnixDate(orbitZeroPoint, ms);
  //Serial.println("[MCU] msSinceZero: " + String((unsigned long int)msSinceZero));
  uint32_t currentSpinPeriodMs = getCurrentSpinPeriodMs(msSinceZero);
  uint16_t currentOrbitPeriodSec = getCurrentOrbitPeriodMs(msSinceZero) / 1000;
  response.write(currentSpinPeriodMs, 20);
  //Serial.print("   currentSpinPeriodMs: " + String(currentSpinPeriodMs));
  response.write(currentOrbitPeriodSec, 13);
  //Serial.print("   currentOrbitPeriodSec: " + String(currentOrbitPeriodSec));
  uint64_t hoursSinceZero = msSinceZero / MILLISECONDS_IN_AN_HOUR;
  //Serial.println("[MCU] Hours since zero: " + String((unsigned long int)hoursSinceZero));
  response.write(hoursSinceZero, 24);
  technicalResponseBuffer.addData(response.adatok);
}

//#16 Átállítja a paramétereket egy pontos mérési módra (99.9% pontosság)
//Nincs paraméter
//fnlength: 13
//fnDecPoint: 9
//fnBase: 1
bool setPreciseProcessing(byte data[8])
{
  fnLength = 13;
  fnDecPoint = 9;
  fnBase = 1;

  measurmentMode = 0;
  timedMeasurmentDelay = 5000;
  requiredMeasurmentCount = 1;

  for(byte i = 0; i < 18; i++)
  {
    spectrumGrouping[i] = true;
    enabledSpectrums[i] = true;
  }

  toAverageMeasurments = false;
  packetSize = 2;

  keepEveryNthMessage = 1;

  sensor.setGain(3);
  sensor.setMeasurementMode(3);
  sensor.setIntegrationTime(50);
  return true;
}

//#17 Átállítja a praméterkeket úgy hogy csoportosítja a szomszédos spektrumokat
//Nincs paraméter
//fnlength: 12
//fnDecPoint: 8
//fnBase: 1
//Spektrum grouping: 010101010101010101
bool setGroupNeighboursProcessing(byte data[8])
{
  fnLength = 12;
  fnDecPoint  = 8;
  fnBase = 1;

  measurmentMode = 0;
  timedMeasurmentDelay = 5000;
  requiredMeasurmentCount = 1;

  for(byte i = 0; i < 18; i++)
  {
    spectrumGrouping[i] = i % 2 == 1;
    enabledSpectrums[i] = true;
  }

  toAverageMeasurments = false;
  packetSize = 1;

  keepEveryNthMessage = 1;

  sensor.setGain(3);
  sensor.setMeasurementMode(3);
  sensor.setIntegrationTime(50);
  return true;
}

//#18 Átállítja a praméterkeket úgy hogy csoportosítja a szomszédos spektrumokat
//Nincs paraméter
//fnlength: 12
//fnDecPoint: 8
//fnBase: 1
//Spektrum grouping: 010101010101010101
bool setSweepProcessingPreset(byte data[8])
{
  fnLength = 5;
  fnDecPoint  = 1;
  fnBase = 4;

  measurmentMode = 1;
  requiredMeasurmentCount = 12;
  numSamplesInCircle = 12;

  for(byte i = 0; i < 18; i++)
  {
    spectrumGrouping[i] = i % 2 == 1;
    enabledSpectrums[i] = true;
  }

  toAverageMeasurments = false;
  packetSize = 5;

  keepEveryNthMessage = 1;
  setupTargeting(false);
  return true;
}

//#19
//byte 1 - 4 : value
bool setSpinDriftDegPerHour(byte data[8]){
  int rawData = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4];
  spinDriftDegPerHour = intToFloatByBits(rawData);
  setupTargeting(false);
  //Serial.println("[MCU] received updated spinDriftDegPerHour: " + String(spinDriftDegPerHour));
  createCommandResponse19();//Echo back the updated drift
  return true;
}

void createCommandResponse19(){
  BitChain response;
  response.write(0b11000000,8);
  response.write(runtimeToDailyTime(millis()),24);
  response.write(19,8);//Command code
  int intval = floatToIntByBits(spinDriftDegPerHour);
  response.adatok[5] = (intval >> 24) & 0b11111111;
  response.adatok[6] = (intval >> 16) & 0b11111111;
  response.adatok[7] = (intval >> 8 ) & 0b11111111;
  response.adatok[8] = intval         & 0b11111111;
  technicalResponseBuffer.addData(response.adatok);
}


