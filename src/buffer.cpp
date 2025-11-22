#include "Arduino.h"
#include <buffer.h>

BufferStatus Buffer::write(float byte) {
    uint8_t next_index = (newest_index + 1) % BUFFER_SIZE;
    data[newest_index] = byte;
    newest_index = next_index;
    return BUFFER_OK;
}

void Buffer::printData() const {
    for (int i = 0; i < BUFFER_SIZE; ++i) {
        Serial.print(data[i]);
        Serial.print(" ");
    }
    Serial.println("");
}

float Buffer::avgOfLastN(int n) const {
    if(n < 1){n=1;}
    float sum = 0;
    int start = newest_index -1;
    for (int i = 0; i < n; ++i) {
        int cindex = (start - i + BUFFER_SIZE) % BUFFER_SIZE;

        float val = data[cindex];
        if(val <=0){
            if(sum == 0){return 0;}
            return sum / i +1;
        }
        sum += val;
    } 

    return sum / n;
}