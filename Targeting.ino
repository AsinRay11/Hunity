

//Out: 0..359°, the current position on the orbit circle [f ellipses]
long getOrbitDegreesSinceZero(long msNow){
  if(timeSync == -1)return 0;
  uint64_t msSinceZero = msSinceUnixDate(orbitZeroPoint, msNow);
  //time / period * 360 = degrees
  return (msSinceZero * 360) / ((orbitPeriodSec*1000 + getCurrentOrbitPeriodMs(msSinceZero)) / 2);
}

uint32_t getCurrentOrbitPeriodMs(uint64_t msSinceZero){
  return ((int64_t)orbitPeriodSec) * 1000 + ((int64_t)orbitPeriodDrift*(int64_t)msSinceZero / (int64_t)MILLISECONDS_IN_A_DAY);
}

uint16_t getOrbitPos(long msNow){
  if(timeSync == -1)return 0;
  long degreesSinceZero = getOrbitDegreesSinceZero(msNow);
  degreesSinceZero = degreesSinceZero % 360;
  if(!orbitDirCW)degreesSinceZero = 359 - degreesSinceZero;
  return degreesSinceZero;
}

long getSpinDegreesSinceZero(long msNow){
  if(timeSync == -1)return 0;
  uint64_t msSinceZero = msSinceUnixDate(spinZeroPoint, msNow);
  return (msSinceZero * 360) / ((spinPeriodMs + getCurrentSpinPeriodMs(msSinceZero))/2) + (long)(msSinceZero * spinDriftDegPerHour / MILLISECONDS_IN_AN_HOUR);
}

uint32_t getCurrentSpinPeriodMs(uint64_t msSinceZero){
  return ((int64_t)spinPeriodMs) + ((int64_t)spinPeriodDrift*(int64_t)msSinceZero / MILLISECONDS_IN_A_DAY);
}

//Out: 0..359°, the current position on the spin circle
uint16_t getSpinPos(long msNow, bool forceDisableSpinCompensation){
  if(timeSync == -1)return 0;
  long degreesSinceZero = getSpinDegreesSinceZero(msNow);
  degreesSinceZero = (degreesSinceZero+360) % 360;
  if(!spinDirCW)degreesSinceZero = 359 - degreesSinceZero;
  //Serial.println("[MCU] GetSpinPos - degreesSinceZero: " + String(degreesSinceZero));
  if(useSpinCompensation && !forceDisableSpinCompensation){
    //I DONT WANT TO DO THIS BRAINFUCK MATH OF WHEN TO SUBTRACT WHAT. There are abrupt changes, there are smooth changes, and its different whether the two things spin the same way or not
    long spinCompensation = getOrbitPos(msNow) + 360;
    spinCompensation -= spinCompensationZeroPoint;
    if(binarySpinCompensation){
      spinCompensation /= 180;
      spinCompensation *= 180;//No fractions in integer math, should round 0..179 to 0° and 180..359 to 180°
    }
    degreesSinceZero -= (spinCompensation%360);//ADD OR MINUS HERE??
  }
  degreesSinceZero = (degreesSinceZero+720) % 360;
  return degreesSinceZero;
}

bool setupTargeting(bool resetZeroPoints){
  if(timeSync == -1){
    //Serial.println("[MCU] Error! cannot set up targeting without timesync");//DEBUG
    return false;
  }
  long msNow = millis();
  if(resetZeroPoints || spinZeroPoint == 0 || orbitZeroPoint == 0){
    spinZeroPoint = runtimeToUnixTime(msNow);
    orbitZeroPoint = spinZeroPoint;
  }

  spinPosAtLastMeasurementCheck = 0;

  targetPositionNumbers[currentTargetIndex] = 0;

  uint64_t msSinceZero = msSinceUnixDate(spinZeroPoint, msNow);
  int integrationTime = sensor.getIntegrationTime();
  halfMeasurementTime = 350 /*700*/ + 3 /*6*/ * integrationTime;
  //Serial.println("[MCU] Sweep setup - IT: " + String(integrationTime) + " ~ " + String(halfMeasurementTime*2) + "ms");
  int measurementTime = halfMeasurementTime * 2;
  halfMeasurementLengthDegrees = halfMeasurementTime * 360 / getCurrentSpinPeriodMs(msSinceZero);
  //Serial.println("[MCU] halfMeasurementTime: " + String(halfMeasurementTime) + "ms   halfMeasurementLength: " + String(halfMeasurementLengthDegrees) + "deg");
  numRevolutionsPerBatch = 1;

  bool requiredTargetPosAdded[numSamplesInCircle];
  uint8_t numPositionsAdded = 0;

  for(int i = 0; i < numSamplesInCircle; i++){
    requiredTargetPosAdded[i] = false;
  }

  int timeBetweenTwoTargets = getCurrentSpinPeriodMs(msSinceZero) / numSamplesInCircle;
  int timeSinceLastTarget = timeBetweenTwoTargets;

  int currentPosNumber = 0;
  if(!spinDirCW)currentPosNumber = numSamplesInCircle - 1;//In CCW dir, we create the order in reverse

  //Put them in an order that is achievable with the current measurement length
  while(numPositionsAdded < numSamplesInCircle){
    if((numPositionsAdded == 0 || timeSinceLastTarget > measurementTime) && !requiredTargetPosAdded[currentPosNumber]){
      targetPositionNumbers[numPositionsAdded] = currentPosNumber;
      requiredTargetPosAdded[currentPosNumber] = true;
      //Serial.println("Added #" + String(currentPosNumber) + " - " + String(targetNumberToDegrees(targetPositionNumbers[numPositionsAdded])) + "° to index " + String(numPositionsAdded) + ", " + String(timeSinceLastTarget) + "ms after the previous");//DEBUG
      numPositionsAdded++;
      timeSinceLastTarget = 0;
    }
    timeSinceLastTarget += timeBetweenTwoTargets;
    if(spinDirCW){
      currentPosNumber++;
      if(currentPosNumber >= numSamplesInCircle){
        numRevolutionsPerBatch += 1;
        currentPosNumber -= numSamplesInCircle;
      }
    }else{
      currentPosNumber--;
      if(currentPosNumber < 0){
        numRevolutionsPerBatch += 1;
        currentPosNumber += numSamplesInCircle;
      }
    }
  }
  targetingSetupCompleted = true;
  return true;
}

uint16_t targetNumberToDegrees(uint8_t i){
  return i*360/numSamplesInCircle;
}



//Auto resets its output to false after one true output, until the next point is reached
bool needsSweepMeasurement(long msNow){
  if(!targetingSetupCompleted){
    bool retrySuccesful = setupTargeting(false);
    if(!retrySuccesful)return false;
  }
  if(timeSync == -1 || (!targetingSetupCompleted)){
    //DEBUG:
    //if(timeSync == -1)Serial.println("[MCU] No timeSync for sweep!");
    //if(!targetingSetupCompleted)Serial.println("[MCU] Targeting not set up for sweep!");
    return false;
  }
  bool ok = false;
  //Serial.println("timeSync: " + String(timeSync) + "   targetingSetupCompleted: " + String(targetingSetupCompleted));
  uint16_t currentPos = getSpinPos(msNow, false);
  uint16_t prevPos = spinPosAtLastMeasurementCheck;
  uint16_t offsetTargetPos = targetNumberToDegrees(targetPositionNumbers[currentTargetIndex]);
  if(spinDirCW)offsetTargetPos = offsetTargetPos + 360 - halfMeasurementLengthDegrees;
  else offsetTargetPos += halfMeasurementLengthDegrees;
  offsetTargetPos = offsetTargetPos % 360;

  //God save anyone that has to figure this out without the 6 cases drawn up 
  if((spinDirCW && (currentPos == prevPos - 1 || (prevPos == 0 && currentPos == 359))) ||
     (!spinDirCW&& (currentPos == prevPos + 1 || (prevPos == 359 && currentPos == 0)))){
    //FUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU
  }else{
    if((spinDirCW &&  ( (prevPos < offsetTargetPos && offsetTargetPos <= currentPos)  ||  (offsetTargetPos <= currentPos && currentPos < prevPos)  ||  (currentPos < prevPos && prevPos < offsetTargetPos)  )) || 
      (!spinDirCW && ( (currentPos <= offsetTargetPos && offsetTargetPos < prevPos)  ||  (prevPos < currentPos && currentPos <= offsetTargetPos)  ||  (offsetTargetPos < prevPos && prevPos < currentPos)  )) ){
      //Serial.println("[MCU] Waiting for point #" + String(targetPositionNumbers[currentTargetIndex]) + " at " + String(targetNumberToDegrees(targetPositionNumbers[currentTargetIndex])) + "° - current: " + String(currentPos));
      //Serial.println("Spot deemed worthy of taking a measurement!");//DEBUG
      //Serial.println("[MCU] Took measurement for point #" + String(targetPositionNumbers[currentTargetIndex]) + " at abs " + String(getSpinPos(msNow, true)) + "°, rel " + String(getSpinPos(msNow, false)) + " nominal target pos: " + String(targetNumberToDegrees(targetPositionNumbers[currentTargetIndex])));
      currentTargetIndex = (currentTargetIndex + 1) % numSamplesInCircle;
      ok = true;
    }
  }
  
  spinPosAtLastMeasurementCheck = currentPos;
  //Take spinDir into consideration - DONE
  return ok;
}

long debugMs = 0;
bool needsTargetedMeasurement(long msNow){
  if(!targetingSetupCompleted){
    bool retrySuccesful = setupTargeting(false);
    if(!retrySuccesful)return false;
  }
  bool ok = false;
  //Serial.println("timeSync: " + String(timeSync) + "   targetingSetupCompleted: " + String(targetingSetupCompleted));
  uint16_t currentPos = getSpinPos(msNow, false);
  uint16_t prevPos = spinPosAtLastMeasurementCheck;
  uint16_t offsetTargetPos = singleTargetPosition;
  if(spinDirCW)offsetTargetPos = offsetTargetPos + 360 - halfMeasurementLengthDegrees;
  else offsetTargetPos += halfMeasurementLengthDegrees;
  offsetTargetPos = offsetTargetPos % 360;
  //if(millis() > debugMs + 500){Serial.println("[MCU] Waiting for point " + String(singleTargetPosition) + "* - current rel: " + String(currentPos) + "   current abs: "+ String(getSpinPos(msNow, true)) + "   orbit: " + String(getOrbitPos(msNow)) + "   halfMeasurementDegs: " + String(halfMeasurementLengthDegrees));debugMs = millis();}
  //God save anyone that has to figure this out without the 6 cases drawn up 
  if((spinDirCW && (currentPos == prevPos - 1 || (prevPos == 0 && currentPos == 359))) ||
     (!spinDirCW&& (currentPos == prevPos + 1 || (prevPos == 359 && currentPos == 0)))){
    //FUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU
    //[MCU] TRIGGERED for point 135* - current rel: 231   current abs: 336   orbit: 105 prevPos: 232
    //Notice prevPos being larger than currentPos while we are in a CW dir and pos should be strictly monoton increasing?
    //WELL THERE IS  a compensation step in calculating the current position THAT SUBTRACTS orbit pos as an INTEGER, so it RANDOMLY STEPS BACK

  }else{
    if((spinDirCW &&  ( (prevPos < offsetTargetPos && offsetTargetPos <= currentPos)  ||  (offsetTargetPos <= currentPos && currentPos < prevPos)  ||  (currentPos < prevPos && prevPos < offsetTargetPos)  )) || 
      (!spinDirCW && ( (currentPos <= offsetTargetPos && offsetTargetPos < prevPos)  ||  (prevPos < currentPos && currentPos <= offsetTargetPos)  ||  (offsetTargetPos < prevPos && prevPos < currentPos)  )) ){
      /*Serial.println(" (prevPos < offsetTargetPos && offsetTargetPos <= currentPos): " + String((prevPos < offsetTargetPos && offsetTargetPos <= currentPos)));
      Serial.println(" (offsetTargetPos <= currentPos && currentPos < prevPos): " + String((offsetTargetPos <= currentPos && currentPos < prevPos)));
      Serial.println(" (currentPos < prevPos && prevPos < offsetTargetPos): " + String((currentPos < prevPos && prevPos < offsetTargetPos)));
      Serial.println(" (currentPos <= offsetTargetPos && offsetTargetPos < prevPos): " + String((currentPos <= offsetTargetPos && offsetTargetPos < prevPos)));
      Serial.println(" (prevPos < currentPos && currentPos <= offsetTargetPos): " + String((prevPos < currentPos && currentPos <= offsetTargetPos)));
      Serial.println(" (offsetTargetPos < prevPos && prevPos < currentPos): " + String((offsetTargetPos < prevPos && prevPos < currentPos)));*/
      //Serial.println("[MCU] TRIGGERED for point " + String(singleTargetPosition) + "* - current rel: " + String(currentPos) + "   current abs: "+ String(getSpinPos(msNow, true)) + "   orbit: " + String(getOrbitPos(msNow)) + " prevPos: " + String(prevPos));
      //Serial.println("Spot deemed worthy of taking a measurement!");//DEBUG
      ok = true;
    }
  }
  spinPosAtLastMeasurementCheck = currentPos;
  //Take spinDir into consideration - DONE
  return ok;
}