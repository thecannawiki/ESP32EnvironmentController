#include "Arduino.h"
#include <buffer.h>

Buffer humidityBuffer = {{}, 0, 0};
Buffer errorBuffer = {{}, 0, 0};

/// This is bad 
enum BufferStatus humidityBufferWrite(float byte){
    uint8_t next_index = (humidityBuffer.newest_index+1) % BUFFER_SIZE;
    humidityBuffer.data[humidityBuffer.newest_index] = byte;
    humidityBuffer.newest_index = next_index;
    return BUFFER_OK;
}
enum BufferStatus errorBufferWrite(float byte){
    uint8_t next_index = (errorBuffer.newest_index+1) % BUFFER_SIZE;
    errorBuffer.data[errorBuffer.newest_index] = byte;
    errorBuffer.newest_index = next_index;
    return BUFFER_OK;
}


