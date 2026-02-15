#ifndef BUFFER_H
    #define BUFFER_H

    #include <stdint.h>
    #include <iostream>

    enum BufferStatus {BUFFER_OK, BUFFER_EMPTY, BUFFER_FULL};

    template <size_t SIZE>
    class Buffer {
    public:
        float data[SIZE];
        uint16_t newest_index = 0;
        uint16_t oldest_index = 0;

        BufferStatus write(float byte);
        void printData() const;
        float avgOfLastN(int n) const;
        float PID_D_Diff(int lookback_length) const;
        static constexpr size_t size() { return SIZE; }
        int countVal(float val, int lookback) const;
    };


    // Write to circular buffer
    template <size_t SIZE>
    BufferStatus Buffer<SIZE>::write(float byte) {
        data[newest_index] = byte;
        newest_index = (newest_index + 1) % SIZE;

        // If we wrapped and overwrote oldest, move oldest forward too
        if (newest_index == oldest_index) {
            oldest_index = (oldest_index + 1) % SIZE;
            return BUFFER_FULL;  // Data was overwritten
        }

        return BUFFER_OK;
    }

    // Print contents (in circular-buffer order)
    template <size_t SIZE>
    void Buffer<SIZE>::printData() const {
        uint16_t index = oldest_index;

        std::cout << "[Buffer contents]: ";
        for (uint16_t i = 0; i < SIZE; i++) {
            std::cout << data[index] << " ";
            index = (index + 1) % SIZE;
        }
        std::cout << std::endl;
    }

    // Average of last N values
    template <size_t SIZE>
    float Buffer<SIZE>::avgOfLastN(int n) const {
        if (n <= 0 || n > SIZE) return 0.0f;

        float sum = 0;
        int count = 0;

        // Walk BACKWARD from newest_index
        int idx = (newest_index + SIZE - 1) % SIZE;

        while (count < n) {
            sum += data[idx];
            idx = (idx + SIZE - 1) % SIZE;
            count++;
        }

        return sum / n;
    }
    template <size_t SIZE>
    float Buffer<SIZE>::PID_D_Diff(int lookback_length) const {
        int lookback_index = this->newest_index-lookback_length;
        int cindex = this->newest_index-1;
        if(lookback_index < 0){
        lookback_index = this->size() + lookback_index;
        }
        if(cindex < 0){
        cindex = this->size() + cindex;
        }
        float old = this->data[lookback_index];
        float current = this->data[cindex];
        if(old ==-1.0f || old == 0.0f || current ==-1.0f || current== 0.0f){
        return 0;
        }

        return current - old;
    }
    template <size_t SIZE>
    int Buffer<SIZE>::countVal(float val, int lookback) const {
        int votes = 0;
        for(int i=1; i<=lookback; i++){
            int index = (this->newest_index - i + this->size()) % this->size();
            float frame  = this->data[index];
            if(frame == val){
                votes = votes + 1;
            }
            
        }
        return votes;
    }


#endif
