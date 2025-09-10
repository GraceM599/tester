
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

//L3M6DS3 pins
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

    uint8_t mode = SPI_MODE_3;

    int bits_per_word = 8;
    int speed = 900000;

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
    uint8_t tx[2] = { reg_addr | 0x80, 0x00 };
    uint8_t rx[2] = { 0 };

    struct spi_ioc_transfer tr = {0};
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = 2;
    tr.delay_usecs = 0;
    tr.speed_hz = 1000000;  // 1 MHz
    tr.bits_per_word = 8;

    int ret = ioctl(f_dev, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        std::cout << "SPI Read Error!" << std::endl;
        return 0;
    }

    return rx[1];  // second byte contains register value


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

            status = writeRegister(ACCEL_CONFIG_, 0x20);
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
            status = writeRegister(GYRO_CONFIG_, 0x20);
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

    initSPI();
    int acc_range = 2;
    short gyro_range = 2;


    //replace == with != ; broke just for testing
    if (setAccConfig(0) != 0) {

        std::cout << "Error while setting accelerometer config" << std::endl;
        //should return 0x69
        int id = readRegister(0x0F);
        std::cout << "WHO_AM_I: " << std::hex << id << std::endl;
        return 0;
    }
    if (setGyroConfig(0) != 0) {
        std::cout << "Error while setting gyroscope config" << std::endl;
        return 0;
    }

    for (int i = 0; i<100; i++){
        fetchData();
        usleep(100000);
    }
    return 1;
}