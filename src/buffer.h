#ifndef BUFFER   //ensure the buffer isn't redefined on every import of this header
#define BUFFER

    enum BufferStatus {BUFFER_OK, BUFFER_EMPTY, BUFFER_FULL};
    #define BUFFER_SIZE 20

    class Buffer {
        public:
            float data[BUFFER_SIZE];     // Array to hold buffer data
            uint8_t newest_index = 0;    // Index of the newest element
            uint8_t oldest_index = 0;
            
            // Member function to write data to the buffer
            BufferStatus write(float byte);
            void printData() const;
            float avgOfLastN(int n) const;
    };

#endif