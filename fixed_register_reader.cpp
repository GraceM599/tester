#include <sys/types.h>
#include <time.h>
#include <errno.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <string.h>
#include <cmath>
#include <linux/spi/spidev.h>
#include <cstdint>

// LSM6DS3 Register Definitions
#define WHO_AM_I_REG        0x0F
#define CTRL1_XL_REG        0x10  // Accelerometer control
#define CTRL2_G_REG         0x11  // Gyroscope control
#define CTRL3_C_REG         0x12  // Control register 3

// Data registers (correct addresses for LSM6DS3)
#define OUTX_L_G            0x22  // Gyro X axis low byte
#define OUTX_H_G            0x23  // Gyro X axis high byte
#define OUTY_L_G            0x24  // Gyro Y axis low byte
#define OUTY_H_G            0x25  // Gyro Y axis high byte
#define OUTZ_L_G            0x26  // Gyro Z axis low byte
#define OUTZ_H_G            0x27  // Gyro Z axis high byte

#define OUTX_L_XL           0x28  // Accel X axis low byte
#define OUTX_H_XL           0x29  // Accel X axis high byte
#define OUTY_L_XL           0x2A  // Accel Y axis low byte
#define OUTY_H_XL           0x2B  // Accel Y axis high byte
#define OUTZ_L_XL           0x2C  // Accel Z axis low byte
#define OUTZ_H_XL           0x2D  // Accel Z axis high byte

int f_dev;
double acc_lsb_to_g = 0.488;      // For ±16g range
double gyro_lsb_to_degsec = 17.5; // For ±500 dps range

void initSPI() {
    // Use SPI_MODE_0 for LSM6DS3 (CPOL=0, CPHA=0)
    uint8_t mode = SPI_MODE_0;
    int bits_per_word = 8;
    int speed = 500000;  // 500kHz - safe speed for LSM6DS3

    f_dev = open("/dev/spidev0.0", O_RDWR);
    if (f_dev < 0) {
        std::cout << "ERROR: Failed to open SPI communication on /dev/spidev0.0" << std::endl;
        return;
    }

    // Configure SPI parameters with proper error handling
    if(ioctl(f_dev, SPI_IOC_WR_MODE, &mode) < 0){
        std::cout << "ERROR: Failed to set SPI mode" << std::endl;
        close(f_dev);
        return;
    }
    
    if (ioctl(f_dev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
        std::cout << "ERROR: Failed to set bits per word" << std::endl;
        close(f_dev);
        return;
    }

    if (ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        std::cout << "ERROR: Failed to set SPI speed" << std::endl;
        close(f_dev);
        return;
    }
    
    std::cout << "✅ SPI initialized successfully (Mode 0, 500kHz)" << std::endl;
}

// FIXED: Corrected register reading function
uint8_t readRegister(uint8_t register_addr){
    struct spi_ioc_transfer xfer = {0};

    // Create command with read bit set (MSB = 1)
    uint8_t tx_data[2];
    uint8_t rx_data[2];
    
    tx_data[0] = register_addr | 0x80;  // Set read bit
    tx_data[1] = 0x00;  // FIXED: Send 0x00, not 0x05
    
    rx_data[0] = 0x00;
    rx_data[1] = 0x00;
    
    xfer.tx_buf = (__u64)tx_data;
    xfer.rx_buf = (__u64)rx_data;
    xfer.len = 2;

    int retv = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
    if (retv < 0) {
        std::cout << "ERROR: SPI transfer failed for register 0x" 
                  << std::hex << (int)register_addr << std::dec << std::endl;
        return 0xFF;  // Return error value
    }
    
    return rx_data[1];  // Return the data byte
}

// FIXED: Corrected register writing function
int writeRegister(uint8_t register_addr, uint8_t value) {
    struct spi_ioc_transfer xfer = {0};

    uint8_t tx_data[2];
    uint8_t rx_data[2];
    
    tx_data[0] = register_addr & 0x7F;  // Clear read bit for write
    tx_data[1] = value;
    
    rx_data[0] = 0x00;
    rx_data[1] = 0x00;
    
    xfer.tx_buf = (__u64)tx_data;
    xfer.rx_buf = (__u64)rx_data;
    xfer.len = 2;

    int retv = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
    if (retv < 0) {
        std::cout << "ERROR: SPI write failed for register 0x" 
                  << std::hex << (int)register_addr << std::dec << std::endl;
        return -1;
    }
    
    // Small delay after write
    usleep(1000);  // 1ms delay
    return 0;
}

// FIXED: Corrected accelerometer configuration
int setAccConfig() {
    std::cout << "Configuring accelerometer..." << std::endl;
    
    // CTRL1_XL: Configure accelerometer
    // 0x60 = 0110 0000
    // ODR = 0110 (416 Hz), FS = 00 (±2g), BW = 00 (400 Hz)
    // For ±16g: 0x64 = 0110 0100 (FS = 01)
    uint8_t ctrl1_xl_val = 0x64;  // 416 Hz, ±16g
    
    int status = writeRegister(CTRL1_XL_REG, ctrl1_xl_val);
    if (status == 0) {
        acc_lsb_to_g = 0.488;  // mg/LSB for ±16g range
        std::cout << "✅ Accelerometer configured: ±16g range, 416 Hz" << std::endl;
    } else {
        std::cout << "❌ Failed to configure accelerometer" << std::endl;
    }
    
    return status;
}

// FIXED: Corrected gyroscope configuration  
int setGyroConfig() {
    std::cout << "Configuring gyroscope..." << std::endl;
    
    // CTRL2_G: Configure gyroscope
    // 0x60 = 0110 0000
    // ODR = 0110 (416 Hz), FS = 00 (±250 dps)
    // For ±500 dps: 0x64 = 0110 0100 (FS = 01)
    uint8_t ctrl2_g_val = 0x64;  // 416 Hz, ±500 dps
    
    int status = writeRegister(CTRL2_G_REG, ctrl2_g_val);
    if (status == 0) {
        gyro_lsb_to_degsec = 17.5;  // mdps/LSB for ±500 dps range
        std::cout << "✅ Gyroscope configured: ±500 dps range, 416 Hz" << std::endl;
    } else {
        std::cout << "❌ Failed to configure gyroscope" << std::endl;
    }
    
    return status;
}

// FIXED: Corrected data fetching with proper register addresses
void fetchData(){
    // Read accelerometer data (FIXED: using correct register addresses)
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    
    // Read accelerometer data
    uint8_t xl_low = readRegister(OUTX_L_XL);
    uint8_t xl_high = readRegister(OUTX_H_XL);
    accel_x = (int16_t)((xl_high << 8) | xl_low);
    
    uint8_t yl_low = readRegister(OUTY_L_XL);
    uint8_t yl_high = readRegister(OUTY_H_XL);
    accel_y = (int16_t)((yl_high << 8) | yl_low);
    
    uint8_t zl_low = readRegister(OUTZ_L_XL);
    uint8_t zl_high = readRegister(OUTZ_H_XL);
    accel_z = (int16_t)((zl_high << 8) | zl_low);
    
    // Read gyroscope data  
    uint8_t xg_low = readRegister(OUTX_L_G);
    uint8_t xg_high = readRegister(OUTX_H_G);
    gyro_x = (int16_t)((xg_high << 8) | xg_low);
    
    uint8_t yg_low = readRegister(OUTY_L_G);
    uint8_t yg_high = readRegister(OUTY_H_G);
    gyro_y = (int16_t)((yg_high << 8) | yg_low);
    
    uint8_t zg_low = readRegister(OUTZ_L_G);
    uint8_t zg_high = readRegister(OUTZ_H_G);
    gyro_z = (int16_t)((zg_high << 8) | zg_low);
    
    // Convert to physical units
    float accX = ((float)accel_x) * acc_lsb_to_g / 1000.0f;  // Convert to g
    float accY = ((float)accel_y) * acc_lsb_to_g / 1000.0f;
    float accZ = ((float)accel_z) * acc_lsb_to_g / 1000.0f;
    
    float gyroX = ((float)gyro_x) * gyro_lsb_to_degsec / 1000.0f;  // Convert to dps
    float gyroY = ((float)gyro_y) * gyro_lsb_to_degsec / 1000.0f;
    float gyroZ = ((float)gyro_z) * gyro_lsb_to_degsec / 1000.0f;

    // Print data
    std::cout << "Accel: X=" << accX << "g, Y=" << accY << "g, Z=" << accZ << "g | "
              << "Gyro: X=" << gyroX << "°/s, Y=" << gyroY << "°/s, Z=" << gyroZ << "°/s"
              << std::endl;
}

int main() {
    std::cout << "=== FIXED LSM6DS3 Register Reader ===" << std::endl;
    
    initSPI();
    if (f_dev < 0) {
        std::cout << "❌ SPI initialization failed" << std::endl;
        return -1;
    }
    
    // Test WHO_AM_I first
    std::cout << "\nTesting sensor communication..." << std::endl;
    uint8_t who_am_i = readRegister(WHO_AM_I_REG);
    std::cout << "WHO_AM_I: 0x" << std::hex << (int)who_am_i << std::dec << std::endl;
    
    if (who_am_i == 0x69) {
        std::cout << "🎉 LSM6DS3 detected successfully!" << std::endl;
    } else if (who_am_i == 0x6C) {
        std::cout << "🎉 LSM6DSO detected!" << std::endl;
    } else if (who_am_i == 0x00 || who_am_i == 0xFF) {
        std::cout << "❌ No sensor response - check wiring and power" << std::endl;
        close(f_dev);
        return -1;
    } else {
        std::cout << "⚠️  Unknown sensor detected (got 0x" << std::hex << (int)who_am_i 
                  << ", expected 0x69)" << std::dec << std::endl;
        std::cout << "Continuing anyway..." << std::endl;
    }
    
    // Configure sensor
    std::cout << "\nConfiguring sensor..." << std::endl;
    if (setAccConfig() != 0) {
        std::cout << "❌ Failed to configure accelerometer" << std::endl;
        close(f_dev);
        return -1;
    }
    
    if (setGyroConfig() != 0) {
        std::cout << "❌ Failed to configure gyroscope" << std::endl;
        close(f_dev);
        return -1;
    }
    
    // Small delay for configuration to take effect
    usleep(100000);  // 100ms
    
    std::cout << "\nReading sensor data..." << std::endl;
    
    // Read data in a loop
    for (int i = 0; i < 20; i++){
        std::cout << "Sample " << (i+1) << ": ";
        fetchData();
        usleep(100000);  // 100ms delay between readings
    }
    
    close(f_dev);
    std::cout << "\n✅ Test completed successfully!" << std::endl;
    return 0;
}
