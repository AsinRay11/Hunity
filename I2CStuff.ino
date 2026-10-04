void receiveEvent(int howMany) {
  int i = 0;
  while (Wire.available() && i < howMany) {
    receivedData[i++] = Wire.read();
  }

  if (howMany == 5 && receivedData[0] == SYNC_COMMAND) {
    //Serial.println("[MCU] time sync received")//DEBUG
    // UNIX idő beállítása little endian módon
    unixTime = (unsigned long)receivedData[4] << 24 | 
               (unsigned long)receivedData[3] << 16 |
               (unsigned long)receivedData[2] << 8  | 
               (unsigned long)receivedData[1];

    handleTimeSync(unixTime);
  }
  else if(howMany == 8)
  {
    telecommandBuffer.addData(receivedData);
  }
}

// Válasz adás a masternek
void requestEvent() {
  //Serial.println("[MCU] data request ack");
  if(technicalResponseBuffer.availableData()>0){
    Telemetry t=technicalResponseBuffer.getData();
    Wire.write(t.data,16);
  }
  else if(telemetryBuffer.availableData() > 0){
    Telemetry t = telemetryBuffer.getData();
    Wire.write(t.data, 16); // Küldjünk vissza 16 bájtot
  }
  else{//No data
    for (byte i = 0; i < 15; i++)
    {
      responseData[i] = 0;
    }
    if(timeSync == -1)
    {
      responseData[4] = 255;
    }
    responseData[0] = 0b11000000;

    long time = runtimeToDailyTime(millis());
    for(byte i = 0; i < 3; i++)
    {
      responseData[3-i] = (time >> 8*i) & 0xFF;
    }

    responseData[14] = (int) analogReadTemp();
    responseData[15] = sensor0available | (sensor1available << 1) | (sensor2available << 2);
    Wire.write(responseData, 16); // Küldjünk vissza 16 bájtot
  }
}

void handleTelecommand(byte data[8])
{

  bool writeToEeprom = true;

  /*Serial.print("[MCU] "); //DEBUG
  Serial.print(data[0]);
  Serial.println(" commandcode received");*/

  switch(data[0])//address
  {
    case 0: writeToEeprom = requestTechnicalMessage(data);break;
    case 1: writeToEeprom = eepromClearAndReboot(data);break; //eeprom clear & reboot
    case 2: writeToEeprom = reboot(data);break; //reboot
    case 3: writeToEeprom = rebootSensor(data);break; //reboot sensor
    case 4: writeToEeprom = eepromClear(data);break; //eeprom clear
    case 5: writeToEeprom = eepromWrite(data);break; //eeprom write
    case 6: writeToEeprom = relayToAsx(data);break; //relay to asx
    case 7: writeToEeprom = configSensor(data);break; //config sensor
    case 8: writeToEeprom = configMeasurmentMode(data);break; //set measurment mode
    case 9: writeToEeprom = configSensorGrouping(data);break; //config sensor grouing
    case 10: writeToEeprom = configProcessing(data);break; //config processing
    case 11: writeToEeprom = setDefaultProcessing(data);break; //default processing
    case 12: writeToEeprom = readFromEeprom(data);break; //read from eeprom
    case 13: writeToEeprom = readFromAsx(data);break; //read from sensor
    case 14: writeToEeprom = clearTelemetry(data);break; //clear telemetry
    case 15: writeToEeprom = updateTrajectoryInfo(data);break;
    case 16: writeToEeprom = setPreciseProcessing(data);break; //Pontos mérési mód
    case 17: writeToEeprom = setGroupNeighboursProcessing(data);break; //Szomszédok csoportosítása mód
    case 18: writeToEeprom = setSweepProcessingPreset(data);break;//Körmérés 12 ponttal
    case 19: writeToEeprom = setSpinDriftDegPerHour(data);break;//Pörgési irány finomhangolása
  }

  makeTechnicalResponse();
  
  if(writeToEeprom){writeParamsToEEPROM();}

  buffer.clear();
  currentTargetIndex = 0;
}

void handleTimeSync(unsigned long data)
{
  bool lemaradva = timeSync == -1;

  tsReceiveTime = millis();
  timeSync = data;

  if(lemaradva)
  {
    while(buffer.availableData() >= requiredMeasurmentCount)
    {
      processData();
    }  
  }
}
