#include <Arduino.h>

class SensorBuffer {
  private:
    float *buffer;          // Pointer to the buffer array
    float bufferSum;        // Sum of all elements in the buffer
    uint8_t bufferSize;     // Size of the buffer
    uint8_t bufferIndex;    // Current index in the buffer for the next value
    uint8_t valuesInBuffer; // Number of values currently in the buffer

  public:
    // Constructor
    SensorBuffer(uint8_t size) {
      bufferSize = size;
      buffer = new float[bufferSize];
      bufferSum = 0;
      bufferIndex = 0;
      valuesInBuffer = 0;
    }

    // Destructor
    ~SensorBuffer() {
      delete[] buffer;
    }

    // Add a new value to the buffer
    void addValue(float value) {
      if (valuesInBuffer < bufferSize) { // Buffer not full
        buffer[valuesInBuffer++] = value; // Add value to the buffer
        bufferSum += value; // Update sum
      } else { // Buffer full, replace the oldest value
        bufferSum -= buffer[bufferIndex]; // Subtract the oldest value
        buffer[bufferIndex] = value; // Replace the oldest value with the new one
        bufferSum += value; // Update sum
        bufferIndex = (bufferIndex + 1) % bufferSize; // Update the index
      }
    }

    // Calculate and return the average of the values in the buffer
    float getAverage() {
      if (valuesInBuffer == 0) return 0; // Avoid division by zero
      return bufferSum / valuesInBuffer;
    }
};