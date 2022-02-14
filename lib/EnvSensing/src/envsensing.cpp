#include "envsensing.h"

template <class T>
EnvSensingChr<T>::EnvSensingChr(uint16_t uuid, uint16_t gain) {
    _data = 0;
    _gain = gain;
    chr = BLECharacteristic(uuid);
}

template <class T>
T EnvSensingChr<T>::getData(void) {
    return _data;
}

template <class T>
T EnvSensingChr<T>::getDataGain(void) {
    return _data * _gain;
}

template <class T>
void EnvSensingChr<T>::setData(T data) {
    _data = data;
}

/* Force compile to those types */
template class EnvSensingChr<uint8_t>;
template class EnvSensingChr<uint16_t>;
template class EnvSensingChr<int16_t>;
template class EnvSensingChr<uint32_t>;
