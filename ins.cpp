#include "ins.h"

// === SPI settings ===
uint8_t mode = SPI_MODE_0;
uint8_t bits = 8;
uint32_t speed = 1000000; // 1 MHz
int spi_fd;

int MAG_X_BIAS = 69;  // mGuass
int MAG_Y_BIAS = -4;  // mGuass
int MAG_Z_BIAS = 287; // mGuass

// Erm Helpers
void printHex(int var)
{
    std::cout << std::hex << var << std::endl;
}

void delay(int ms)
{
    usleep(ms * 1000);
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
    delay(50); // Wait longer than before (was 10ms before)

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
    int16_t rawX = ((int16_t)readICM(B0, B0_EXT_SLV_SENS_DATA_01) << 8) | readICM(B0, B0_EXT_SLV_SENS_DATA_00);
    int16_t rawY = ((int16_t)readICM(B0, B0_EXT_SLV_SENS_DATA_03) << 8) | readICM(B0, B0_EXT_SLV_SENS_DATA_02);
    int16_t rawZ = ((int16_t)readICM(B0, B0_EXT_SLV_SENS_DATA_05) << 8) | readICM(B0, B0_EXT_SLV_SENS_DATA_04);

    // Convert to microtesla (0.15 uT/LSB for AK09916)
    mx = (rawX - MAG_X_BIAS) * 0.15f;
    my = (rawY - MAG_Y_BIAS) * 0.15f;
    mz = (rawZ - MAG_Z_BIAS) * 0.15f;
}

void getIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
{
    ax = read16(ACCEL_XOUT_H) / 2048.0f;
    ay = read16(ACCEL_YOUT_H) / 2048.0f;
    az = read16(ACCEL_ZOUT_H) / 2048.0f;

    gx = read16(GYRO_XOUT_H) / 16.4f;
    gy = read16(GYRO_YOUT_H) / 16.4f;
    gz = read16(GYRO_ZOUT_H) / 16.4f;
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

    // I want to reset the ICM to clear anything and verything
    //  Trigger full device reset (bit 7 of PWR_MGMT_1)
    writeICM(B0, B0_PWR_MGMT_1, 0x80);
    delay(100); // Wait for reboot

    whoamiICM();
    // Where the fuck are all them functions ????

    // Reset Device
    //  writeICM(B0, B0_PWR_MGMT_1, 0x80 | 0x41);
    //  delay(100);

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

    // Set Gyro Scale, I want max i.e 2000dps
    new_val = readICM(B2, B2_GYRO_CONFIG_1);
    new_val |= 0x06;
    writeICM(B2, B2_GYRO_CONFIG_1, new_val);

    // Set Acc Scale, I want max i.e
    new_val = readICM(B2, B2_ACCEL_CONFIG);
    new_val |= 0x06;
    writeICM(B2, B2_ACCEL_CONFIG, new_val);

    // Wakeup ICM
    new_val = readICM(B0, B0_PWR_MGMT_1);
    new_val &= 0xBF;
    writeICM(B0, B0_PWR_MGMT_1, new_val);
    delay(100);
}

void initMAG()
{
    // Reset I2C Master
    uint8_t new_val = readICM(B0, B0_USER_CTRL);
    new_val |= 0x02;
    writeICM(B0, B0_USER_CTRL, new_val);

    // Enable I2C Master
    new_val = readICM(B0, B0_USER_CTRL);
    new_val |= 0x20;
    writeICM(B0, B0_USER_CTRL, new_val);

    delay(100);

    // Set I2C Master Clock Frequency (400 kHz)
    new_val = readICM(B3, B3_I2C_MST_CTRL);
    new_val |= 0x07;
    writeICM(B3, B3_I2C_MST_CTRL, new_val);

    resetMag();

    // See who am i
    uint8_t ak09916_id = readMagRegister(MAG_WIA2);

    if (ak09916_id == AK09916_ID)
        std::cout << "🥰 MAG Found :D" << std::endl;
    else
        std::cout << "🥲 MAG not Found :(" << std::endl;

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

    delay(1000);
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

    std::cout << "Calibration Done" << std::endl;
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

    std::cout << "Gyro bias registers reset to 0!" << std::endl;
    delay(100);
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

    while (true)
    {
        getMag(mx, my, mz);
        getIMU(ax, ay, az, gx, gy, gz);

        std::cout << "Accel (g): X=" << ax << " Y=" << ay << " Z=" << az << std::endl;
        std::cout << "Gyro (dps): X=" << gx << " Y=" << gy << " Z=" << gz << std::endl;
        std::cout << "MAG: X=" << mx << " Y=" << my << " Z=" << mz << std::endl;
        std::cout << "---------------------------" << std::endl;

        delay(100);
    }

    close(spi_fd);
    return 0;
}
