
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
//linux is on raspi

//LSM6DSOX pins
// Serial Data pin acts as SDI when SPI is used
// SCL (Serial CLick) pin for SPI clock
//CS (Chip Select) pin
//DO pin for serial data output

//PI pins
//MOSI (19, MOSI0)
//MISO (21, MISO0)
//SCLK (Serial CLock, 23, SCLK0)
//CS (Either 24 or 26)

//run if no workie
//int id = readRegister(0x0F);
//std::cout << "WHO_AM_I: " << std::hex << id << std::endl;

double acc_lsb_to_g;
int f_dev;
double gyro_lsb_to_degsec;

// SDA - MOSI  ; DO - MISO0  ; SCL - SCLK0  ; CS - CE0

void initSPI() {

    uint8_t mode = SPI_MODE_0;

    int bits_per_word = 8;
    int speed = 1000000;

    f_dev = open("/dev/spidev0.0", O_RDWR);
    if (f_dev < 0) {
        std::cout<< "ERR: Failed to open SPI communication on /dev/spidev0.0";
    }


    //config SPI parameters
    if(ioctl(f_dev, SPI_IOC_WR_MODE, &mode) < 0){
        close(f_dev);
    }
    if (ioctl(f_dev, SPI_IOC_RD_MODE, &mode) < 0) {
        close(f_dev);
    }
    /* Set bits per word*/
    if (ioctl(f_dev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
        close(f_dev);
    }
    if (ioctl(f_dev, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word) < 0) {
        close(f_dev);
    }

    /* Set SPI speed*/
    if (ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        close(f_dev);
    }
    if (ioctl(f_dev, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
        close(f_dev);
    }
}

//read register function
int readRegister(uint8_t register_add){
    struct spi_ioc_transfer xfer[1] = {0};
    // Write message for register address
    uint8_t reg_addr = register_add | 0x80;
    uint8_t data[2];
    data[0] = reg_addr;
    data[1] = 0x00;
    xfer[0].tx_buf = (__u64)data;      // output buffer
    xfer[0].rx_buf = (__u64)data;      // input buffer
    xfer[0].len = (__u32)sizeof(data);  // length of data to read
    int retv = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
    if (retv < 0){
        std::cout << "error in spi_read_reg8(): ioctl(SPI_IOC_MESSAGE(2))" << std::endl;
    }
    return data[1];

}

int writeRegister(uint8_t register_addr, uint8_t value) {
    struct spi_ioc_transfer xfer[1] = {0};

    // Write message for register address
    uint8_t data[2];
    data[0] = register_addr;
    data[1] = value;
    xfer[0].tx_buf = (__u64)data;      // output buffer
    xfer[0].rx_buf = (__u64)data;      // input buffer
    xfer[0].len = (__u32)sizeof(data); // length of data to write

    int retv = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);

    if (retv < 0)
    {
        std::cout << "error in spi_write_reg8(): ioctl(SPI_IOC_MESSAGE(2)) return" <<std::endl;
    }
    return retv;
}

int setAccConfig(int config_num){
    int status;
    uint8_t ACCEL_CONFIG_ = 0x10;





    switch(config_num){
        case 0: // range = +- 2 g
            acc_lsb_to_g = 0.061;
            std::cout << "made it to case 0" << std::endl;

            status = writeRegister(ACCEL_CONFIG_, 0x48);
            std::cout << status << std::endl;

            break;
        case 1: // range = +- 4 g
            acc_lsb_to_g = 0.122;
            status = writeRegister(ACCEL_CONFIG_, 0xAA);
            break;
        case 2: // range = +- 8 g
            acc_lsb_to_g = 0.244;
            status = writeRegister(ACCEL_CONFIG_, 0xAE);
            break;
        case 3: // range = +- 16 g
            acc_lsb_to_g = 0.488;
            status = writeRegister(ACCEL_CONFIG_, 0xA6);
            break;
        default: // error
            status = 1;
            break;
    }
    return status;
}

int setGyroConfig(int config_num){
    int status;

    uint8_t GYRO_CONFIG_ = 0x11;
    switch(config_num){
        case 0:  // range = +- 250 deg/s
            gyro_lsb_to_degsec = 8.75;
            status = writeRegister(GYRO_CONFIG_, 0x48);
            break;
        case 1:  // range = +- 500 deg/s
            gyro_lsb_to_degsec = 17.5;
            std::cout << "At gyro case 2" << std::endl;
            status = writeRegister(GYRO_CONFIG_, 0xA4);
            break;
        case 2: // range = +- 1000 deg/s
            gyro_lsb_to_degsec = 35;
            status = writeRegister(GYRO_CONFIG_, 0xA8);
            break;
        case 3: // range = +- 2000 deg/s
            gyro_lsb_to_degsec = 70;
            status = writeRegister(GYRO_CONFIG_, 0xAC);
            break;
        default:
            //put status back to =1
            status = 0;
            break;
    }
    return status;
}

void fetchData(){
    //temporarily changed x-> nx to avoid non defined errors
    int16_t X, Y, Z;
    X = readRegister(0x29) << 8 | readRegister(0x28);
    Y = readRegister(0x2B) << 8 | readRegister(0x2A);
    Z = readRegister(0x2D)  << 8 | readRegister(0x2C);
    std::cout << readRegister(0x29) << std::endl;
    float accX;
    float accY;
    float accZ;
    float gyroX;
    float gyroY;
    float gyroZ;

    double accXoffset = 0;
    double accYoffset = 0;
    double accZoffset = 0;
    //replace 1 in accZ with !upsideDownMounting - upsideDownMounting
    accX = ((float)X) * acc_lsb_to_g / 1000 - accXoffset;
    accY = ((float)Y) * acc_lsb_to_g / 1000 - accYoffset;
    accZ = (1) * ((float)Z) * acc_lsb_to_g / 1000 - accZoffset;

    X = readRegister(0x23) << 8 | readRegister(0x22);
    Y = readRegister(0x25) << 8 | readRegister(0x24);
    Z = readRegister(0x27) << 8 | readRegister(0x26);

    //correcct code replaced w/o offset
    /*
    gyroX = ((float)X) * gyro_lsb_to_degsec / 1000- gyroXoffset;
    gyroY = ((float)Y) * gyro_lsb_to_degsec / 1000 - gyroYoffset;
    gyroZ = ((float)Z) * gyro_lsb_to_degsec / 1000 - gyroZoffset;

    */
    gyroX = ((float)X) * gyro_lsb_to_degsec / 1000;
    gyroY = ((float)Y) * gyro_lsb_to_degsec / 1000;
    gyroZ = ((float)Z) * gyro_lsb_to_degsec / 1000;

    //print data
    std::cout << "Accel: X=" << accX
                  << " g, Y=" << accY
                  << " g, Z=" << accZ << " g | "
                  << "Gyro: X=" << gyroX
                  << " dps, Y=" << gyroY
                  << " dps, Z=" << gyroZ << " dps"
                  << std::endl;
}

int main() {
    // TIP Press <shortcut actionId="RenameElement"/> when your caret is at the <b>lang</b> variable name to see how CLion can help you rename it.
    initSPI();
    int acc_range = 2;
    short gyro_range = 2;
    //replace == with != ; broke just for testing
    if (setAccConfig(0) > 0) {
        uint8_t temp = readRegister(0x0F);
        std::cout << temp << std::endl;
        std::cout << "Error while setting accelerometer config" << std::endl;
        return 0;
    }
    if (setGyroConfig(0) > 0) {
        std::cout << "Error while setting gyroscope config" << std::endl;
        return 0;
    }

    for (int i = 0; i<100; i++){
        fetchData();
        usleep(100000);
    }
    return 1;
}