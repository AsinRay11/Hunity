struct Measurement
{
	unsigned long measurementTime; //time between boot and measurement of data in milliseconds
	float sensorReadings[18];
  uint8_t sensorTemps[3];
};

class MeasurementBuffer
{
  static const int maxRows = 360;//The number of rows the buffer has. 360*18*4=> 25920 byte (10%); 
	int lastIn=maxRows-1;
	int lastOut=maxRows-1;
	Measurement rows[MeasurementBuffer::maxRows];

	public: MeasurementBuffer() = default;

  //add the sensor data and the time of the measurement in milliseconds since boot to the buffer
  //returns if it was successfull; returns false if the buffer is full (and adds nothing)
	bool addData(unsigned long measurementTime, float readings[18]) {
		if (this->remainingSpace() != 0) {
			Measurement measurement;
			measurement.measurementTime = measurementTime;
			for (size_t i = 0; i < 18; i++)
			{
				measurement.sensorReadings[i] = readings[i];
			}
			if (lastIn >= MeasurementBuffer::maxRows - 1) {
				lastIn = -1;
			}
			rows[++lastIn] = measurement;
			return true;
		}
		else { return false; }
	}
  //Overload with sensor temperature data
	bool addData(unsigned long measurementTime, float readings[18], uint8_t sensorTemps[3]) {
		if (this->remainingSpace() != 0) {
			Measurement measurement;
			measurement.measurementTime = measurementTime;
			for (size_t i = 0; i < 18; i++)
			{
				measurement.sensorReadings[i] = readings[i];
			}
      for (size_t i = 0; i < 3; i++)
			{
				measurement.sensorTemps[i] = sensorTemps[i];
			}
			if (lastIn >= MeasurementBuffer::maxRows - 1) {
				lastIn = -1;
			}
			rows[++lastIn] = measurement;
			return true;
		}
		else { return false; }
	}
  //returns the data theat has been in the buffer the longest, and hasnt been read yet
  //if there is no avalable data to send it returns a datapacket containing only "0"s
	Measurement getData() {
		Measurement result;
		if (this->availableData() != 0) {
			if (lastOut >= MeasurementBuffer::maxRows - 1) {
				lastOut = -1;
			}
			result = rows[++lastOut];
		}
		else {
			result.measurementTime = 0;
			for (size_t i = 0; i < 18; i++) {
				result.sensorReadings[i] = 0;
			}
		}
		return result;
	}

  //returns the number of datapackets can be read from the buffer
	int availableData() {
		int available;
		if (lastIn < lastOut) {
			available = (MeasurementBuffer::maxRows - lastOut) + lastIn; 
		}
		else {
			available = lastIn - lastOut;
		}
		return available;
	}
  //returns the number of datapackets that can be stored on the buffer
	int remainingSpace() {
		return MeasurementBuffer::maxRows - availableData()-1; //if the 2 pointers are equal, it counts as an empty buffer --> we dont want to fill out all rows
	}

  void clear()
  {
    lastIn=maxRows-1;
	  lastOut=maxRows-1;
  }
};