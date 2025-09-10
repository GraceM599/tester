#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

int main() {
    std::cout << "=== I2C Alternative Test ===" << std::endl;
    std::cout << "If SPI wiring is problematic, try I2C instead!" << std::endl;
    
    std::cout << "\n📍 I2C WIRING (only 4 wires needed):" << std::endl;
    std::cout << "Pi Pin 1 (3.3V) → Sensor VCC" << std::endl;
    std::cout << "Pi Pin 6 (GND)  → Sensor GND" << std::endl;
    std::cout << "Pi Pin 3 (SDA)  → Sensor SDA" << std::endl;
    std::cout << "Pi Pin 5 (SCL)  → Sensor SCL" << std::endl;
    std::cout << "No CS connection needed!" << std::endl;
    
    // Try to open I2C device
    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        std::cout << "\n❌ I2C not enabled. Enable with:" << std::endl;
        std::cout << "sudo raspi-config → Interface Options → I2C → Enable" << std::endl;
        return -1;
    }
    
    // LSM6DS3 I2C address is usually 0x6A or 0x6B
    uint8_t addresses[] = {0x6A, 0x6B};
    
    for (int i = 0; i < 2; i++) {
        std::cout << "\nTrying I2C address 0x" << std::hex << (int)addresses[i] << std::dec << "..." << std::endl;
        
        if (ioctl(fd, I2C_SLAVE, addresses[i]) < 0) {
            std::cout << "Failed to set I2C address" << std::endl;
            continue;
        }
        
        // Try to read WHO_AM_I register (0x0F)
        uint8_t reg = 0x0F;
        if (write(fd, &reg, 1) != 1) {
            std::cout << "Failed to write register address" << std::endl;
            continue;
        }
        
        uint8_t data;
        if (read(fd, &data, 1) != 1) {
            std::cout << "Failed to read data" << std::endl;
            continue;
        }
        
        std::cout << "WHO_AM_I: 0x" << std::hex << (int)data << std::dec << std::endl;
        
        if (data == 0x69) {
            std::cout << "🎉 LSM6DS3 detected via I2C!" << std::endl;
            std::cout << "I2C address: 0x" << std::hex << (int)addresses[i] << std::dec << std::endl;
            close(fd);
            return 0;
        }
    }
    
    std::cout << "\n❌ No sensor detected via I2C" << std::endl;
    std::cout << "Check I2C wiring and sensor power" << std::endl;
    
    close(fd);
    return -1;
}
