#include "ins.h"
#include "AHRSAlgorithms.h"
#include <chrono>
#include <iomanip>
#include <thread> // for sleep
#include <stdlib.h>
#include <string.h>

typedef enum
{
    _250dps,
    _500dps,
    _1000dps,
    _2000dps
} gyro_full_scale;

typedef enum
{
    _2g,
    _4g,
    _8g,
    _16g
} accel_full_scale;

// === SPI settings ===
uint8_t mode = SPI_MODE_0;
uint8_t bits = 8;
uint32_t speed = 7000000; // 7 MHz
int spi_fd;

int MAG_X_BIAS = 69;  // mGuass
int MAG_Y_BIAS = -4;  // mGuass
int MAG_Z_BIAS = 287; // mGuass

static float gyro_scale_factor;
static float accel_scale_factor;

// Erm Helpers
void printHex(int var)
{
    std::cout << std::hex << var << std::endl;
}

void delay(int ms)
{
    usleep(ms * 1000);
}

void sleep(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// === SPI helper functions ===

// Raw Write to Register, remember to selectBank() first
void _writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg & 0x7F, value}; // MSB=0 for write
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = 0,
        .len = 2,
        .speed_hz = speed,
        .bits_per_word = bits,
    };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
}

// Raw Read from Register, remember to selectBank() first
uint8_t _readRegister(uint8_t reg)
{
    uint8_t tx[2] = {reg | 0x80, 0}; // MSB=1 for read
    uint8_t rx[2] = {0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .speed_hz = speed,
        .bits_per_word = bits,
    };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    return rx[1];
}

// Raw Read from Registers
void _readRegisters(uint8_t startReg, uint8_t *buffer, size_t length)
{
    uint8_t tx[length + 1];
    uint8_t rx[length + 1];
    tx[0] = startReg | 0x80;   // MSB=1 for read
    memset(&tx[1], 0, length); // Dummy bytes to receive data

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = length + 1,
        .speed_hz = speed,
        .bits_per_word = bits,
    };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    memcpy(buffer, &rx[1], length); // skip the first dummy byte
}

// Select Bank directly
void selectBank(userbank bank)
{
    _writeRegister(REG_BANK_SEL, bank);
}

// Select Bank and Write to Register
void writeICM(userbank bank, uint8_t reg, uint8_t value)
{
    selectBank(bank);
    _writeRegister(reg, value);
}

// Select Bank and Read from Register
uint8_t readICM(userbank bank, uint8_t reg)
{
    selectBank(bank);
    return _readRegister(reg);
}

int16_t read16(uint8_t reg)
{
    selectBank(B0);
    uint8_t tx[3] = {reg | 0x80, 0, 0};
    uint8_t rx[3] = {0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 3,
        .speed_hz = speed,
        .bits_per_word = bits,
    };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    return (int16_t)((rx[1] << 8) | rx[2]);
}

static uint8_t readMagRegister(uint8_t reg)
{
    // Configure SLV0 to read 1 byte from the MAG register
    writeICM(B3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR); // Set read flag + MAG address
    writeICM(B3, B3_I2C_SLV0_REG, reg);                    // Set the MAG register to read
    writeICM(B3, B3_I2C_SLV0_CTRL, 0x81);                  // Enable, 1 byte

    // Wait for the ICM to complete I2C transaction
    delay(10); // Wait longer than before (was 10ms before)

    // Now read the fetched data
    return readICM(B0, B0_EXT_SLV_SENS_DATA_00);
}

static uint8_t readMagRegister(uint8_t reg, uint8_t len)
{
    // Configure SLV0 to read 1 byte from the MAG register
    writeICM(B3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR); // Set read flag + MAG address
    writeICM(B3, B3_I2C_SLV0_REG, reg);                    // Set the MAG register to read
    writeICM(B3, B3_I2C_SLV0_CTRL, 0x80 | len);            // Enable, 1 byte

    // Wait for the ICM to complete I2C transaction
    delay(50); // Wait longer than before (was 10ms before)
}

static void writeMagRegister(uint8_t reg, uint8_t val)
{
    writeICM(B3, B3_I2C_SLV0_ADDR, WRITE | MAG_SLAVE_ADDR);
    writeICM(B3, B3_I2C_SLV0_REG, reg);
    writeICM(B3, B3_I2C_SLV0_DO, val);
    writeICM(B3, B3_I2C_SLV0_CTRL, 0x81);
}

void resetMag()
{
    writeMagRegister(0x32, 0x01);
    delay(100); // Give it time
}

void getMag(float &mx, float &my, float &mz)
{
    uint8_t buffer[6];
    selectBank(B0);
    _readRegisters(B0_EXT_SLV_SENS_DATA_00, buffer, 6);

    int16_t rawX = ((int16_t)buffer[1] << 8) | buffer[0];
    int16_t rawY = ((int16_t)buffer[3] << 8) | buffer[2];
    int16_t rawZ = ((int16_t)buffer[5] << 8) | buffer[4];

    mx = (rawX - MAG_X_BIAS) * 0.15f;
    my = (rawY - MAG_Y_BIAS) * 0.15f;
    mz = (rawZ - MAG_Z_BIAS) * 0.15f;
}

void getIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
{
    uint8_t buffer[12];
    selectBank(B0);
    _readRegisters(ACCEL_XOUT_H, buffer, 12);

    int16_t rawAx = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t rawAy = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t rawAz = (int16_t)((buffer[4] << 8) | buffer[5]);
    int16_t rawGx = (int16_t)((buffer[6] << 8) | buffer[7]);
    int16_t rawGy = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t rawGz = (int16_t)((buffer[10] << 8) | buffer[11]);

    ax = rawAx / accel_scale_factor;
    ay = rawAy / accel_scale_factor;
    az = rawAz / accel_scale_factor;

    gx = rawGx / gyro_scale_factor;
    gy = rawGy / gyro_scale_factor;
    gz = rawGz / gyro_scale_factor;
}

void whoamiICM()
{
    uint8_t id = readICM(B0, WHO_AM_I);
    if (id != 0xEA)
    {
        std::cerr << "🥲 ICM-20948 not found! (ID: 0x" << std::hex << (int)id << ")" << std::endl;
        exit(1);
    }

    else
    {
        std::cout << "🥰 ICM-20948 found! (ID: 0x" << std::hex << (int)id << ")" << std::endl;
    }
}

// === Sensor init and read ===
void initICM20948()
{

    // ICM Settings
    gyro_full_scale gy_fvalue = _250dps;
    accel_full_scale acc_fvalue = _2g;

    // I want to reset the ICM to clear anything and verything
    //  Trigger full device reset (bit 7 of PWR_MGMT_1)
    writeICM(B0, B0_PWR_MGMT_1, 0x80);
    delay(1000); // Wait for reboot

    while (readICM(B0, B0_PWR_MGMT_1) & 0x80)
    {
        delay(10); // Wait until reset bit clears
    }

    std::cout << "ICM Reset Done" << std::endl;

    whoamiICM();
    // Where the fuck are all them functions ????

    // Reset Device
    writeICM(B0, B0_PWR_MGMT_1, 0x80 | 0x41);
    delay(100);

    // Select Clock Source
    uint8_t new_val = readICM(B0, B0_PWR_MGMT_1);
    new_val |= 0x01;
    writeICM(B0, B0_PWR_MGMT_1, new_val);

    // Align ODR Enable
    writeICM(B2, B2_ODR_ALIGN_EN, 0x01);

    // Gyro Low Pass Filter
    new_val = readICM(B2, B2_GYRO_CONFIG_1);
    new_val |= 0x00 << 3;
    writeICM(B2, B2_GYRO_CONFIG_1, new_val);

    // Acc Low Pass Filter
    new_val = readICM(B2, B2_ACCEL_CONFIG);
    new_val |= 0x00 << 3;
    writeICM(B2, B2_GYRO_CONFIG_1, new_val);

    // Gyro Sample Rate Divider
    writeICM(B2, B2_GYRO_SMPLRT_DIV, 0x00);

    // Acc Sample Rate Divider
    uint8_t divider_1 = (uint8_t)(0x00 >> 8);
    uint8_t divider_2 = (uint8_t)(0x0F & 0x00);

    writeICM(B2, B2_ACCEL_SMPLRT_DIV_1, divider_1);
    writeICM(B2, B2_ACCEL_SMPLRT_DIV_2, divider_2);

    resetGyroBias();
    calibrateGyro();

    // Set Gyro Scale
    new_val = readICM(B2, B2_GYRO_CONFIG_1);

    switch (gy_fvalue)
    {
    case _250dps:
        new_val |= 0x00;
        gyro_scale_factor = 131.0;
        break;
    case _500dps:
        new_val |= 0x02;
        gyro_scale_factor = 65.5;
        break;
    case _1000dps:
        new_val |= 0x04;
        gyro_scale_factor = 32.8;
        break;
    case _2000dps:
        new_val |= 0x06;
        gyro_scale_factor = 16.4;
        break;
    }

    writeICM(B2, B2_GYRO_CONFIG_1, new_val);

    // Set Acc Scale.
    new_val = readICM(B2, B2_ACCEL_CONFIG);

    switch (acc_fvalue)
    {
    case _2g:
        new_val |= 0x00;
        accel_scale_factor = 16384;
        break;
    case _4g:
        new_val |= 0x02;
        accel_scale_factor = 8192;
        break;
    case _8g:
        new_val |= 0x04;
        accel_scale_factor = 4096;
        break;
    case _16g:
        new_val |= 0x06;
        accel_scale_factor = 2048;
        break;
    }

    writeICM(B2, B2_ACCEL_CONFIG, new_val);

    // Wakeup ICM
    new_val = readICM(B0, B0_PWR_MGMT_1);
    new_val &= 0xBF;
    writeICM(B0, B0_PWR_MGMT_1, new_val);
    delay(100);
}

void enableI2C()
{

    // Reset I2C Master
    uint8_t new_val = readICM(B0, B0_USER_CTRL);
    new_val |= 0x02;
    writeICM(B0, B0_USER_CTRL, new_val);

    // Enable I2C Master
    new_val = readICM(B0, B0_USER_CTRL);
    new_val |= 0x20;
    writeICM(B0, B0_USER_CTRL, new_val);

    // Set I2C Master Clock Frequency (400 kHz)
    new_val = readICM(B3, B3_I2C_MST_CTRL);
    new_val |= 0x07;
    writeICM(B3, B3_I2C_MST_CTRL, new_val);
}

// Please have a look at this: https://devzone.nordicsemi.com/f/nordic-q-a/61234/icm20948-magnetometer-issue

void initMAG()
{

    enableI2C();
    resetMag();

    // Keep checking until the correct magnetometer ID is found
    uint8_t ak09916_id = 0;

    while (ak09916_id != AK09916_ID)
    {
        ak09916_id = readMagRegister(MAG_WIA2);

        if (ak09916_id == AK09916_ID)
        {
            std::cout << "🥰 MAG Found :D" << std::endl;
        }
        else
        {
            std::cout << "🥲 MAG not Found :( Retrying..." << std::endl;
            resetMag();
            enableI2C();
            delay(10); // Optional: small delay to avoid busy looping
        }
    }

    // Set ODR
    writeICM(B0, LP_CONFIG, 0x40);
    delay(10);

    // Set ODR Rate
    writeICM(B0, I2C_MST_ODR_CONFIG, 0x03);
    delay(10);

    // Lets Reset it again lol
    resetMag();
    delay(10);

    // Set to continuous mode of 100Hz
    writeMagRegister(MAG_CNTL2, 0x08);
    delay(10);

    // Start Reading the Magnetometer from HXL
    // Once you start, it will become continuous
    readMagRegister(MAG_HXL, 8);
    delay(10);
}

void calibrateGyro()
{
    std::cout << "Calibrating Gyro" << std::endl;
    int16_t x_gyro_bias, y_gyro_bias, z_gyro_bias;
    int32_t x_bias, y_bias, z_bias;

    for (int i = 0; i < 100; i++)
    {
        x_bias += read16(GYRO_XOUT_H);
        y_bias += read16(GYRO_YOUT_H);
        z_bias += read16(GYRO_ZOUT_H);
        delay(10);
    }

    x_bias /= 100;
    y_bias /= 100;
    z_bias /= 100;

    // Construct the gyro biases for push to the hardware gyro bias registers,
    // which are reset to zero upon device startup.
    // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format.
    // Biases are additive, so change sign on calculated average gyro biases

    x_gyro_bias = -(int16_t)(x_bias / 4);
    y_gyro_bias = -(int16_t)(y_bias / 4);
    z_gyro_bias = -(int16_t)(z_bias / 4);

    delay(10);

    writeICM(B2, B2_XG_OFFS_USRH, (uint8_t)(x_gyro_bias >> 8));
    writeICM(B2, B2_XG_OFFS_USRL, (uint8_t)(x_gyro_bias));

    writeICM(B2, B2_YG_OFFS_USRH, (uint8_t)(y_gyro_bias >> 8));
    writeICM(B2, B2_YG_OFFS_USRL, (uint8_t)(y_gyro_bias));

    writeICM(B2, B2_ZG_OFFS_USRH, (uint8_t)(z_gyro_bias >> 8));
    writeICM(B2, B2_ZG_OFFS_USRL, (uint8_t)(z_gyro_bias));

    std::cout << " 🚲 Gyro Calibration Done" << std::endl;
}

void resetGyroBias()
{
    // Reset all gyro offsets to 0
    writeICM(B2, B2_XG_OFFS_USRH, 0x00); // High byte
    writeICM(B2, B2_XG_OFFS_USRL, 0x00); // Low byte
    writeICM(B2, B2_YG_OFFS_USRH, 0x00);
    writeICM(B2, B2_YG_OFFS_USRL, 0x00);
    writeICM(B2, B2_ZG_OFFS_USRH, 0x00);
    writeICM(B2, B2_ZG_OFFS_USRL, 0x00);

    delay(100);
}

void calibrateACC()
{
    //Populate this pls
}

// === Main loop ===
int main()
{
    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0)
    {
        perror("Can't open SPI device");
        return 1;
    }

    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    initICM20948();
    initMAG();

    float mx, my, mz;
    float ax, ay, az, gx, gy, gz;

    static auto lastUpdate = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> delta = now - lastUpdate;
    lastUpdate = now;

    while (true)
    {
        getMag(mx, my, mz);
        getIMU(ax, ay, az, gx, gy, gz);

        now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> delta = now - lastUpdate;
        lastUpdate = now;

        MahonyQuaternionUpdate(ax, ay, az, gx * DEG_TO_RAD,
                               gy * DEG_TO_RAD, gz * DEG_TO_RAD, my,
                               mx, mz, delta.count());

        // std::cout << "TIME IN SECONDS: " << delta.count() << std::endl;
        // std::cout << "Accel (g): X=" << ax << " Y=" << ay << " Z=" << az << std::endl;
        // std::cout << "Gyro (dps): X=" << gx << " Y=" << gy << " Z=" << gz << std::endl;
        // std::cout << "MAG: X=" << mx << " Y=" << my << " Z=" << mz << std::endl;
        // std::cout << "---------------------------" << std::endl;
        // std::cout << "MAHONY: Q0=" << *getQ() << " QX=" << *getQ() + 1 << " QY=" << *getQ() + 2 << " QZ=" << *getQ() + 3 << std::endl;
        // std::cout << "---------------------------" << std::endl;

        float yaw, pitch, roll;
        yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)), *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));

        pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() * *(getQ() + 2)));

        roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) * *(getQ() + 3)), *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));

        pitch *= RAD_TO_DEG;
        yaw *= RAD_TO_DEG;

        // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
        // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
        // - http://www.ngdc.noaa.gov/geomag-web/#declination
        yaw -= 2.8;
        roll *= RAD_TO_DEG;

        std::cout << std::fixed;
        std::cout << std::setprecision(2);
        std::cout << "---------------------------" << std::endl;
        std::cout << "DETLA:" << delta.count() << ", YAW=" << yaw << " PITCH=" << pitch << " ROLL=" << roll << std::endl;
        std::cout << "---------------------------" << std::endl;

        sleep(10);
    }

    close(spi_fd);
    return 0;
}
