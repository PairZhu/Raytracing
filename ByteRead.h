#ifndef _BYTE_READ_H_
#define _BYTE_READ_H_

#include <EEPROM.h>

template <typename T>
union ByteData {
    T data;
    uint8_t bytes[sizeof(T)];
};

template <typename T>
T ByteRead(uint8_t addr) {
    ByteData<T> data;
    for (uint8_t i = 0; i < sizeof(T); i++) {
        data.bytes[i] = EEPROM.read(addr++);
    }
    return data.data;
}

template <typename T>
void ByteWrite(uint8_t addr, T value) {
    ByteData<T> data;
    data.data = value;
    for (uint8_t i = 0; i < sizeof(T); i++) {
        EEPROM.write(addr++, data.bytes[i]);
    }
}

#endif  // !_BYTE_READ_H_