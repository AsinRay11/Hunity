// the purpose of this struct is bieng able to return it in a function (getData())
struct Telemetry{
  byte data[16];
};
class TelemetryBuffer
{
  static const int maxRows = 1000;//The number of rows the buffer has. 100*16 = 1600bytes (0.6%)
	int lastIn=maxRows-1;
	int lastOut=maxRows-1;
  Telemetry rows[maxRows];

	public: TelemetryBuffer() = default;

  //adds the Telemetry to the buffer
  //returns if it was successfull; returns false if the buffer is full (and adds nothing)
	bool addData(byte data[16]) {
		if (this->remainingSpace() != 0) {
			if (lastIn >= maxRows - 1) {
				lastIn = -1;
			}
      lastIn++;
      for (size_t i = 0; i < 16; i++)
			{
				rows[lastIn].data[i]=data[i];
			}
			return true;
		}
		else { return false; }
	}
  //returns the Telemetry that has been in the buffer the longest, and hasnt been read yet
  //if there is no avalable data to send the Telemetry will only contain zeros
	Telemetry getData() {
		Telemetry result;
		if (this->availableData() != 0) {
			if (lastOut >= maxRows - 1) {
				lastOut = -1;
			}
      lastOut++;
      for (size_t i = 0; i < 16; i++) {
				result.data[i] = rows[lastOut].data[i];
			}
		}
		else {
			for (size_t i = 0; i < 16; i++) {
				result.data[i] = 0;
			}
		}
		return result;
	}

  //returns the number of datapackets can be read from the buffer
	int availableData() {
		int available;
		if (lastIn < lastOut) {
			available = (maxRows - lastOut) + lastIn; 
		}
		else {
			available = lastIn - lastOut;
		}
		return available;
	}
  //returns the number of datapackets that can be stored on the buffer
	int remainingSpace() {
		return maxRows - availableData()-1; //if the 2 pointers are equal, it counts as an empty buffer --> we dont want to fill out all rows
	}

  //returns remaining space as a percentage of total rowcount
  byte remainingSpacePercentage(){
    float percentage= (float)remainingSpace() / (float) TelemetryBuffer::maxRows;
    return (int)(percentage*100);
  }

  void clear()
  {
    lastIn=maxRows-1;
	  lastOut=maxRows-1;
  }
};