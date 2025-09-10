#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <vector>
#include <cstdint>
#include <chrono>
#include <thread>

// Based on the working NASA_USLI_2026_Controls implementation
#define I2C_FILE "/dev/i2c-1"
#define IMU_ADDRESS_A 0x6A
#define IMU_ADDRESS_B 0x6B
#define WHO_AM_I 0x0F
#define CTRL1_XL 0x10  // Accelerometer control register
#define CTRL2_G 0x11   // Gyroscope control register

class SimpleI2C {
private:
    int i2c_fd;
    uint8_t device_addr;

public:
    SimpleI2C(uint8_t deviceAddr) : device_addr(deviceAddr) {
        i2c_fd = open(I2C_FILE, O_RDWR);
        if (i2c_fd < 0) {
            throw std::runtime_error("Error opening I2C device file");
        }
        
        if (ioctl(i2c_fd, I2C_SLAVE, deviceAddr) < 0) {
            close(i2c_fd);
            throw std::runtime_error("Error setting I2C device address");
        }
    }
    
    ~SimpleI2C() {
        if (i2c_fd >= 0) {
            close(i2c_fd);
        }
    }
    
    uint8_t readRegister(uint8_t regAddr) {
        if (i2c_fd < 0) {
            throw std::runtime_error("I2C file not open");
        }
        
        // Write register address
        if (write(i2c_fd, &regAddr, 1) != 1) {
            throw std::runtime_error("Failed to write register address");
        }
        
        // Small delay for signal propagation
        std::this_thread::sleep_for(std::chrono::microseconds(3));
        
        // Read response
        uint8_t val;
        if (read(i2c_fd, &val, 1) != 1) {
            throw std::runtime_error("Failed to read register data");
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(3));
        return val;
    }
    
    void writeRegister(uint8_t regAddr, uint8_t data) {
        if (i2c_fd < 0) {
            throw std::runtime_error("I2C file not open");
        }
        
        uint8_t buf[2] = {regAddr, data};
        if (write(i2c_fd, buf, 2) != 2) {
            throw std::runtime_error("Failed to write register data");
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(3));
    }
    
    std::vector<uint8_t> readMultiple(uint8_t regAddr, uint8_t bytes) {
        if (i2c_fd < 0) {
            throw std::runtime_error("I2C file not open");
        }
        
        // Write starting register address
        if (write(i2c_fd, &regAddr, 1) != 1) {
            throw std::runtime_error("Failed to write register address");
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(3));
        
        // Read multiple bytes
        std::vector<uint8_t> results(bytes);
        if (read(i2c_fd, results.data(), bytes) != bytes) {
            throw std::runtime_error("Failed to read multiple bytes");
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(bytes));
        return results;
    }
};

int main() {
    std::cout << "=== Working I2C Test (Based on NASA_USLI_2026_Controls) ===" << std::endl;
    
    try {
        // Test both possible I2C addresses
        uint8_t addresses[] = {IMU_ADDRESS_A, IMU_ADDRESS_B};
        const char* addr_names[] = {"0x6A", "0x6B"};
        
        SimpleI2C* imu = nullptr;
        int working_addr_idx = -1;
        
        // Find which address works
        for (int i = 0; i < 2; i++) {
            std::cout << "\nTesting I2C address " << addr_names[i] << "..." << std::endl;
            
            try {
                imu = new SimpleI2C(addresses[i]);
                uint8_t who_am_i = imu->readRegister(WHO_AM_I);
                
                std::cout << "WHO_AM_I response: 0x" << std::hex << (int)who_am_i << std::dec << std::endl;
                
                if (who_am_i == 0x69) {  // LSM6DS3/LSM6DS33
                    std::cout << "🎉 LSM6DS3 detected at address " << addr_names[i] << "!" << std::endl;
                    working_addr_idx = i;
                    break;
                } else if (who_am_i == 0x6C) {  // LSM6DSO
                    std::cout << "🎉 LSM6DSO detected at address " << addr_names[i] << "!" << std::endl;
                    working_addr_idx = i;
                    break;
                } else if (who_am_i != 0x00 && who_am_i != 0xFF) {
                    std::cout << "⚠️  Unknown sensor detected (WHO_AM_I: 0x" << std::hex << (int)who_am_i << std::dec << ")" << std::endl;
                    working_addr_idx = i;
                    break;
                }
                
                delete imu;
                imu = nullptr;
                
            } catch (const std::exception& e) {
                std::cout << "❌ Failed: " << e.what() << std::endl;
                if (imu) {
                    delete imu;
                    imu = nullptr;
                }
            }
        }
        
        if (working_addr_idx == -1) {
            std::cout << "\n❌ No sensor detected on either I2C address" << std::endl;
            std::cout << "\nTroubleshooting:" << std::endl;
            std::cout << "1. Check I2C is enabled: sudo raspi-config → Interface Options → I2C" << std::endl;
            std::cout << "2. Check wiring:" << std::endl;
            std::cout << "   - Pi Pin 1 (3.3V) → Sensor VCC" << std::endl;
            std::cout << "   - Pi Pin 6 (GND)  → Sensor GND" << std::endl;
            std::cout << "   - Pi Pin 3 (SDA)  → Sensor SDA" << std::endl;
            std::cout << "   - Pi Pin 5 (SCL)  → Sensor SCL" << std::endl;
            std::cout << "3. Verify 3.3V power at sensor with multimeter" << std::endl;
            return -1;
        }
        
        std::cout << "\n=== Configuring Sensor ===" << std::endl;
        
        // Configure using the exact same settings as the working code
        // CTRL1_XL: 0x54 = 0b01010100 (acceleration normal mode, 16g)
        imu->writeRegister(CTRL1_XL, 0x54);
        std::cout << "✅ Accelerometer configured (16g range)" << std::endl;
        
        // CTRL2_G: 0x5C = 0b01011100 (gyro normal mode, 500 dps)
        imu->writeRegister(CTRL2_G, 0x5C);
        std::cout << "✅ Gyroscope configured (500 dps range)" << std::endl;
        
        std::cout << "\n=== Reading Sensor Data ===" << std::endl;
        
        // Read data using the same method as working code
        for (int i = 0; i < 10; i++) {
            try {
                // Read 12 bytes starting from gyro X low register (0x22)
                // This reads: GyroX_L, GyroX_H, GyroY_L, GyroY_H, GyroZ_L, GyroZ_H,
                //             AccelX_L, AccelX_H, AccelY_L, AccelY_H, AccelZ_L, AccelZ_H
                std::vector<uint8_t> rawData = imu->readMultiple(0x22, 12);
                
                // Convert to 16-bit signed values (same as working code)
                int16_t rawGyroX = (rawData[1] << 8) | rawData[0];
                int16_t rawGyroY = (rawData[3] << 8) | rawData[2];
                int16_t rawGyroZ = (rawData[5] << 8) | rawData[4];
                int16_t rawAccelX = (rawData[7] << 8) | rawData[6];
                int16_t rawAccelY = (rawData[9] << 8) | rawData[8];
                int16_t rawAccelZ = (rawData[11] << 8) | rawData[10];
                
                // Convert to physical units (same scaling as working code)
                float accelX_g = (rawAccelX / 32768.0f) * 16.0f;    // ±16g range
                float accelY_g = (rawAccelY / 32768.0f) * 16.0f;
                float accelZ_g = (rawAccelZ / 32768.0f) * 16.0f;
                
                float gyroX_dps = (rawGyroX / 32768.0f) * 500.0f;   // ±500 dps range
                float gyroY_dps = (rawGyroY / 32768.0f) * 500.0f;
                float gyroZ_dps = (rawGyroZ / 32768.0f) * 500.0f;
                
                std::cout << "Sample " << (i+1) << ": "
                         << "Accel: X=" << accelX_g << "g, Y=" << accelY_g << "g, Z=" << accelZ_g << "g | "
                         << "Gyro: X=" << gyroX_dps << "°/s, Y=" << gyroY_dps << "°/s, Z=" << gyroZ_dps << "°/s"
                         << std::endl;
                
            } catch (const std::exception& e) {
                std::cout << "❌ Error reading data: " << e.what() << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        delete imu;
        std::cout << "\n🎉 I2C test completed successfully!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "❌ Fatal error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
