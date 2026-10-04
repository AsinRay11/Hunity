class Sensor
{
  #define AS726X_ADDR 0x49
  #define AS726x_DEVICE_TYPE 0x00
  #define AS726x_HW_VERSION 0x01
  #define AS726x_CONTROL_SETUP 0x04
  #define AS726x_INT_T 0x05
  #define AS726x_DEVICE_TEMP 0x06
  #define AS726x_LED_CONTROL 0x07
  #define AS72XX_SLAVE_STATUS_REG 0x00
  #define AS72XX_SLAVE_WRITE_REG 0x01
  #define AS72XX_SLAVE_READ_REG 0x02
  #define AS72XX_SLAVE_TX_VALID 0x02
  #define AS72XX_SLAVE_RX_VALID 0x01
  #define AS72XX_DEV_SEL 0x4F
  #define POLLING_DELAY 5
  #define MAX_RETRIES 3
  #define TIMEOUT 3000
  
  #define A_Register 0x14
  #define B_Register 0x18
  #define C_Register 0x1C
  #define D_Register 0x20
  #define E_Register 0x24
  #define F_Register 0x28
  #define G_Register 0x14
  #define H_Register 0x18
  #define R_Register 0x14
  #define I_Register 0x1C
  #define S_Register 0x18
  #define J_Register 0x20
  #define T_Register 0x1C
  #define U_Register 0x20
  #define V_Register 0x24
  #define W_Register 0x28
  #define K_Register 0x24
  #define L_Register 0x28



  public: 

    TwoWire *_i2cPort;

    Sensor()
    {

    }

    bool begin(TwoWire &wirePort, uint8_t gain = 3, uint8_t measurmentMode = 3)
    {
        _i2cPort = &wirePort;

        if(setBulbCurrent(0b00)) return false;
        if(disableBulb()) return false;
        if(setIndicatorCurrent(0b11)) return false;
        if(disableIndicator()) return false;
        if(setIntegrationTime(50)) return false;
        if(setGain(gain)) return false;
        if(setMeasurementMode(measurmentMode)) return false;
        return true;
    }


    int readRawValue(uint8_t reg)
    {
      return (virtualReadRegister(reg) << 8) | virtualReadRegister(reg+1);
    }    


    float getCalibratedA(){return getCalibratedValue(A_Register);}
    float getCalibratedB(){return getCalibratedValue(B_Register);}
    float getCalibratedC(){return getCalibratedValue(C_Register);}
    float getCalibratedD(){return getCalibratedValue(D_Register);}
    float getCalibratedE(){return getCalibratedValue(E_Register);}
    float getCalibratedF(){return getCalibratedValue(F_Register);}
    float getCalibratedG(){return getCalibratedValue(G_Register);}
    float getCalibratedH(){return getCalibratedValue(H_Register);}
    float getCalibratedR(){return getCalibratedValue(R_Register);}
    float getCalibratedI(){return getCalibratedValue(I_Register);}
    float getCalibratedS(){return getCalibratedValue(S_Register);}
    float getCalibratedJ(){return getCalibratedValue(J_Register);}
    float getCalibratedT(){return getCalibratedValue(T_Register);}
    float getCalibratedU(){return getCalibratedValue(U_Register);}
    float getCalibratedV(){return getCalibratedValue(V_Register);}
    float getCalibratedW(){return getCalibratedValue(W_Register);}
    float getCalibratedK(){return getCalibratedValue(K_Register);}
    float getCalibratedL(){return getCalibratedValue(L_Register);}

    float getCalbiratedWithIndex(byte i)
    { 
      float value;
      switch (i) 
      {
        case 0: value = getCalibratedA(); break;
        case 1: value = getCalibratedB(); break;
        case 2: value = getCalibratedC(); break;
        case 3: value = getCalibratedD(); break;
        case 4: value = getCalibratedE(); break;
        case 5: value = getCalibratedF(); break;
        case 6: value = getCalibratedG(); break;
        case 7: value = getCalibratedH(); break;
        case 8: value = getCalibratedR(); break;
        case 9: value = getCalibratedI(); break;
        case 10: value = getCalibratedS(); break;
        case 11: value = getCalibratedJ(); break;
        case 12: value = getCalibratedT(); break;
        case 13: value = getCalibratedU(); break;
        case 14: value = getCalibratedV(); break;
        case 15: value = getCalibratedW(); break;
        case 16: value = getCalibratedK(); break;
        case 17: value = getCalibratedL(); break;
      }   

      return value;
    }

    int setBulbCurrent(uint8_t current)
    {
	    if (current > 0b11) current = 0b11; //Limit to two bits

										//Read, mask/set, write
	    uint8_t value = virtualReadRegister(AS726x_LED_CONTROL); //Read
	    value &= 0b11001111; //Clear ICL_DRV bits
	    value |= (current << 4); //Set ICL_DRV bits with user's choice
	    return virtualWriteRegister(AS726x_LED_CONTROL, value); //Write
    }

    int disableBulb()
    {
	    //Read, mask/set, write
	    uint8_t value = virtualReadRegister(AS726x_LED_CONTROL);
	    value &= ~(1 << 3); //Clear the bit
	    return virtualWriteRegister(AS726x_LED_CONTROL, value);
    }

    int enableBulb()
    {
	    //Read, mask/set, write
	    uint8_t value = virtualReadRegister(AS726x_LED_CONTROL);
	    value |= (1 << 3); //Set the bit
	    return virtualWriteRegister(AS726x_LED_CONTROL, value);
    }

    int setIndicatorCurrent(uint8_t current)
    {
	    if (current > 0b11) current = 0b11;
	    //Read, mask/set, write
	    uint8_t value = virtualReadRegister(AS726x_LED_CONTROL); //Read
	    value &= 0b11111001; //Clear ICL_IND bits
	    value |= (current << 1); //Set ICL_IND bits with user's choice
	    return virtualWriteRegister(AS726x_LED_CONTROL, value); //Write
    }

    int disableIndicator()
    {
	    //Read, mask/set, write
	    uint8_t value = virtualReadRegister(AS726x_LED_CONTROL);
	    value &= ~(1 << 0); //Clear the bit
	    return virtualWriteRegister(AS726x_LED_CONTROL, value);
    }

    int enableIndicator()
    {
	    //Read, mask/set, write
	    uint8_t value = virtualReadRegister(AS726x_LED_CONTROL);
	    value |= (1 << 0); //Set the bit
	    return virtualWriteRegister(AS726x_LED_CONTROL, value);
    }

    int setIntegrationTime(uint8_t integrationValue)
    {
	    return virtualWriteRegister(AS726x_INT_T, integrationValue); //Write
    }

    uint8_t getIntegrationTime()
    {
	    uint8_t value = virtualReadRegister(AS726x_INT_T); //Read
	    return value;
    }

    int setGain(uint8_t gain)
    {
	    if (gain > 0b11) gain = 0b11;
	    //Read, mask/set, write
	    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
	    value &= 0b11001111; //Clear GAIN bits
	    value |= (gain << 4); //Set GAIN bits with user's choice
	    return virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write
    }

    uint8_t getGain()
    {
	    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
	    return (value & 0b00110000) >> 4; //Isolate GAIN bits.
    }

    int setMeasurementMode(uint8_t mode)
    {
	    if (mode > 0b11) mode = 0b11;

	    //Read, mask/set, write
	    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
	    value &= 0b11110011; //Clear BANK bits
	    value |= (mode << 2); //Set BANK bits with user's choice
	    return virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write
    }

    uint8_t getMeasurementMode()
    {
	    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
	    return (value & 0b00001100) >> 2; //Isolate BANK bits.
    }

    int enableInterrupt()
    {
	    //Read, mask/set, write
	    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
	    value |= 0b01000000; //Set INT bit
	    return virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write
    }

    //Disables the interrupt pin
    int disableInterrupt()
    {
	    //Read, mask/set, write
	    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
	    value &= 0b10111111; //Clear INT bit
	    return virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write
    }

    int softReset()
    {
	    //Read, mask/set, write
	    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
	    value |= (1 << 7); //Set RST bit
	    return virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write
    }

    uint8_t getTemperature()
    {
	    return (virtualReadRegister(AS726x_DEVICE_TEMP));
    }

    int clearDataAvailable()
    {
	    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP);
	    value &= ~(1 << 1); //Set the DATA_RDY bit
	    return virtualWriteRegister(AS726x_CONTROL_SETUP, value);
    }

    bool dataAvailable()
    {
	    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP);
	    return (value & (1 << 1)); //Bit 1 is DATA_RDY
    }

    float convertBytesToFloat(uint32_t myLong)
    {
	    float myFloat;
	    memcpy(&myFloat, &myLong, 4); //Copy uint8_ts into a float
	    return (myFloat);
    }

    int takeMeasurements()
    {
	    //Clear DATA_RDY flag when using Mode 3
	    if(clearDataAvailable()) return -1;

	    //Goto mode 3 for one shot measurement of all channels
	    //if(setMeasurementMode(3)) return -1;

	    uint32_t timeout = millis() + TIMEOUT;

	    //Wait for data to be ready
	    while (dataAvailable() == false)
	    {
		    delay(POLLING_DELAY);
		    if(millis() > timeout) return -1;
	    }

	    //Readings can now be accessed via getViolet(), getBlue(), etc
	    return 0;
    }

    int takeMeasurementsWithBulb()
    {
	    //enableIndicator(); //Tell the world we are taking a reading. 
	    //The indicator LED is red and may corrupt the readings

	    if(enableBulb()) return -1; //Turn on bulb to take measurement

	    if(takeMeasurements()) return -1;

	    if(disableBulb()) return -1; //Turn off bulb to avoid heating sensor
	    //disableIndicator();

	    return 0;
    }

    float getCalibratedValue(uint8_t calAddress)
    {
      return millis()/2000;
	    uint8_t b0, b1, b2, b3;
	    b0 = virtualReadRegister(calAddress + 0);
	    b1 = virtualReadRegister(calAddress + 1);
	    b2 = virtualReadRegister(calAddress + 2);
	    b3 = virtualReadRegister(calAddress + 3);

	    //Channel calibrated values are stored big-endian
	    uint32_t calBytes = 0;
	    calBytes |= ((uint32_t)b0 << (8 * 3));
	    calBytes |= ((uint32_t)b1 << (8 * 2));
	    calBytes |= ((uint32_t)b2 << (8 * 1));
	    calBytes |= ((uint32_t)b3 << (8 * 0));

	    return (convertBytesToFloat(calBytes));
    }

    int getChannel(uint8_t channelRegister)
    {
	    int colorData = virtualReadRegister(channelRegister) << 8; //High uint8_t
	    colorData |= virtualReadRegister(channelRegister + 1); //Low uint8_t
	    return(colorData);
    }

    void chooseSensor(uint8_t sensorNumber)
    {
      //clearDataAvailable();
      virtualWriteRegister(0x4F, sensorNumber);
    }

    void chooseMaster()//AS72651
    {
      chooseSensor(0x00);
    }

    void chooseFirstSlave()//AS72652
    {
      chooseSensor(0x02);
    }

    void chooseSecondSlave()//AS72653
    {
      chooseSensor(0x01);
    }




    uint8_t virtualReadRegister(uint8_t virtualAddr)
    {
      uint8_t status;
      uint8_t retries = 0;

      //Do a prelim check of the read register
      status = readRegister(AS72XX_SLAVE_STATUS_REG);
      if ((status & AS72XX_SLAVE_RX_VALID) != 0) //There is data to be read
      {
        uint8_t incoming = readRegister(AS72XX_SLAVE_READ_REG); //Read the uint8_t but do nothing with it
      }

      //Wait for WRITE flag to clear
      while (1)
      {
        status = readRegister(AS72XX_SLAVE_STATUS_REG);
        if (status == 0xFF) return status;
        if ((status & AS72XX_SLAVE_TX_VALID) == 0) break; // If TX bit is clear, it is ok to write
        delay(POLLING_DELAY);
        if(retries++ > MAX_RETRIES) return 0xFF;
      }

      // Send the virtual register address (bit 7 should be 0 to indicate we are reading a register).
      if(writeRegister(AS72XX_SLAVE_WRITE_REG, virtualAddr)) return 0xFF;

      retries = 0;

      //Wait for READ flag to be set
      while (1)
      {
        status = readRegister(AS72XX_SLAVE_STATUS_REG);
        if (status == 0xFF) return status;
        if ((status & AS72XX_SLAVE_RX_VALID) != 0) break; // Read data is ready.
        delay(POLLING_DELAY);
        if(retries++ > MAX_RETRIES) return 0xFF;
      }

      uint8_t incoming = readRegister(AS72XX_SLAVE_READ_REG);
      return (incoming);
    }

    //Write to a virtual register in the AS726x
      int virtualWriteRegister(uint8_t virtualAddr, uint8_t dataToWrite)
    {
      uint8_t status;
      uint8_t retries = 0;

      //Wait for WRITE register to be empty
      while (1)
      {
        status = readRegister(AS72XX_SLAVE_STATUS_REG);
        if (status == 0xFF) return -1;
        if ((status & AS72XX_SLAVE_TX_VALID) == 0) break; // No inbound TX pending at slave. Okay to write now.
        delay(POLLING_DELAY);
        if(retries++ > MAX_RETRIES) return -1;
      }

      // Send the virtual register address (setting bit 7 to indicate we are writing to a register).
      writeRegister(AS72XX_SLAVE_WRITE_REG, (virtualAddr | 0x80));

      retries = 0;

      //Wait for WRITE register to be empty
      while (1)
      {
        status = readRegister(AS72XX_SLAVE_STATUS_REG);
        if (status == 0xFF) return -1;
        if ((status & AS72XX_SLAVE_TX_VALID) == 0) break; // No inbound TX pending at slave. Okay to write now.
        delay(POLLING_DELAY);
        if(retries++ > MAX_RETRIES) return -1;
      }

      // Send the data to complete the operation.
      writeRegister(AS72XX_SLAVE_WRITE_REG, dataToWrite);

      return 0;
    }

    //Reads from a give location from the AS726x
    uint8_t readRegister(uint8_t addr)
    {
      uint8_t err = 0xFF;

      _i2cPort->beginTransmission(AS726X_ADDR);
      if(_i2cPort->write(addr) == 0) return err;
      if(_i2cPort->endTransmission()) return err;

      if(_i2cPort->requestFrom(AS726X_ADDR, 1) == 0) return err;
      if (_i2cPort->available()) {
        return (_i2cPort->read());
      }
      else {
        return err; //Error
      }

      return 0;
    }

    //Write a value to a spot in the AS726x
    int writeRegister(uint8_t addr, uint8_t val)
    {
      uint8_t err = 0xFF;

      _i2cPort->beginTransmission(AS726X_ADDR);
      if(_i2cPort->write(addr) == 0) return (int)err;
      if(_i2cPort->write(val) == 0) return (int)err;
      if(_i2cPort->endTransmission()) return (int)err;

      return 0;
    }

};