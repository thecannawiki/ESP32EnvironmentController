
enum BufferStatus {BUFFER_OK, BUFFER_EMPTY, BUFFER_FULL};
#define BUFFER_SIZE  16
struct Buffer {
    float data[BUFFER_SIZE];
    uint8_t newest_index;
    uint8_t oldest_index;
};

extern Buffer humidityBuffer;
extern Buffer errorBuffer;

enum BufferStatus humidityBufferWrite(float byte);
enum BufferStatus errorBufferWrite(float byte);