#ifndef ENVSENSING_H
#define ENVSENSING_H

#include <bluefruit.h>

template <class T>
class EnvSensingChr
{
  private:
    T _data;
    uint16_t _gain;

  public:
    EnvSensingChr(uint16_t uuid, uint16_t gain);
    T getData();
    T getDataGain();
    void setData(T data);
    BLECharacteristic chr;
};
#endif

