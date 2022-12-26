#ifndef ENCODER_H
#define ENCODER_H

#include <ESP32Encoder.h> // lib from https://github.com/madhephaestus/ESP32Encoder

class Encoder
{
public:
    Encoder(uint8_t pinChannelA, uint8_t pinChannelB);

    void set(int16_t value);

    int16_t getCount();

    void clear();

private:
    // using hardware pulse counter
    ESP32Encoder encoder;

    // connections
    uint8_t pinChannelA;
    uint8_t pinChannelB;
};
#endif