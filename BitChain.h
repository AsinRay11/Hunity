class BitChain
{
public:

    uint8_t adatok[16];
    uint8_t length;

    uint8_t pointerByte = 0;
    uint8_t pointerBit = 0;


    BitChain(uint8_t length_ = 16)
    {
        length = length_;
        for (uint8_t i = 0; i < length; i++)
        {
            adatok[i] = 0;
        }
    }

    int write(int data, uint8_t bitCount, bool fillAllBits = true)
    {

        if (!fillAllBits)
        {
            if (getRemainingBitCount() < bitCount)
            {
                return bitCount;
            }
        }

        int toShift = bitCount + pointerBit - 8;
        int copy = data & ((int)pow(2,bitCount)-1);

        if (toShift > 0)
        {
            copy = copy >> toShift;
        }
        else
        {
            copy = copy << ( - 1 * toShift);
        }

        if(pointerByte < length)
        {
          adatok[pointerByte] = adatok[pointerByte] | copy;
        }

        bool overFlow = bitCount > 8 - pointerBit;

        if (overFlow)
        {
            pointerByte++;
            pointerBit = 0;
        }
        else
        {
            pointerBit += bitCount;
        }

        if (overFlow)
        {
            if (pointerByte < length)
            {
                return write(data, toShift);
            }
            else
            {
                return toShift;
            }
        }
        else
        {
            return 0;
        }
    }

    int read(uint8_t bitCount)
    {
        int byteCount = 1 + (bitCount + pointerBit - 1) / 8;
        uint8_t toShift = 8 * byteCount - pointerBit - bitCount;
        int data = 0;

        bool overFlow = byteCount + pointerByte > length;

        if (overFlow)
        {
            byteCount = length - pointerByte;
            toShift = 0;
        }

        for (uint8_t i = 0; i < byteCount; i++)
        {
            data = data << 8;
            data = data | adatok[pointerByte + i];
        }

        data = data >> toShift;

        pointerByte += (pointerBit + bitCount) / 8;
        pointerBit = (pointerBit + bitCount) % 8;

        if (overFlow)
        {
            return data & ((int)pow(2, bitCount - getOverFlownBitCount()) - 1);
        }
        else
        {
            return data & ((int)pow(2, bitCount) - 1);
        }

    }

    int getRemainingBitCount()
    {
        return (8 * ((int)length - (int)pointerByte)) - (int)pointerBit;
    }

    uint8_t getOverFlownBitCount()
    {
        int vissza = getRemainingBitCount();
        if (vissza > 0) { vissza = 0; }
        else { vissza *= -1; }

        return vissza;
    }

    void setPointer(uint8_t byteNumber, uint8_t bitNumber)
    {
        pointerBit = bitNumber;
        pointerByte = byteNumber;
    }
};