#include "Encoder.h"

Encoder::Encoder(uint8_t pinChannelA, uint8_t pinChannelB)
{
    this->pinChannelA = pinChannelA;
    this->pinChannelB = pinChannelB;

    this->encoder.attachFullQuad(this->pinChannelA, this->pinChannelB);
    this->encoder.clearCount();
}

void Encoder::set(int16_t value)
{
    this->encoder.setCount(value);
}

int16_t Encoder::getCount()
{
    return (int16_t)this->encoder.getCount();
}

void Encoder::clear()
{
    this->encoder.clearCount();
}
