struct Telecommand{
  byte data[8];
};
class TelecommandBuffer
{
  static const int maxRows = 20;//The number of rows the buffer has.
	int lastIn=maxRows-1;
	int lastOut=maxRows-1;
  Telecommand rows[maxRows];

	public: TelecommandBuffer() = default;

  //adds the Telecommand to the buffer
  //returns if it was successfull; returns false if the buffer is full (and adds nothing)
	bool addData(byte data[8]) {
		if (this->remainingSpace() != 0) {
			if (lastIn >= maxRows - 1) {
				lastIn = -1;
			}
      lastIn++;
      for (size_t i = 0; i < 8; i++)
			{
				rows[lastIn].data[i]=data[i];
			}
			return true;
		}
		else { return false; }
	}
  //returns the Telecommand that has been in the buffer the longest, and hasnt been read yet
  //if there is no avalable data to send the Telecommand will only contain zeros
	Telecommand getData() {
		Telecommand result;
		if (this->availableData() != 0) {
			if (lastOut >= maxRows - 1) {
				lastOut = -1;
			}
      lastOut++;
      for (size_t i = 0; i < 8; i++) {
				result.data[i] = rows[lastOut].data[i];
			}
		}
		else {
			for (size_t i = 0; i < 8; i++) {
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
    float percentage= (float)remainingSpace() / (float) TelecommandBuffer::maxRows;
    return (int)(percentage*100);
  }
};