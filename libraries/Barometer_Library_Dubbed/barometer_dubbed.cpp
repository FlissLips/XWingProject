#include <Wire.h> // I2C Communication
#include <barometer_dubbed.h>

void barometer_begin()
{
    Wire.setSDA(18);
    Wire.setSCL(19);
    Wire.begin();
    Wire.beginTransmission(HP20X_I2C_DEV_ID);
    Wire.write(HP20X_SOFT_RST);
    Wire.endTransmission();
}

unsigned char barometer_available()
{
    unsigned char Temp = 0;

    /* Send a register reading command */
    Wire.beginTransmission(HP20X_I2C_DEV_ID);
    Wire.write(REG_PARA | HP20X_RD_REG_MODE);
    Wire.endTransmission();
    Wire.requestFrom(HP20X_I2C_DEV_ID, 1);
    while (Wire.available())
    {
        Temp = Wire.read();
    }
    return Temp;
}

unsigned long barometer_read_pressure()
{
    Wire.beginTransmission(HP20X_I2C_DEV_ID);
    Wire.write(HP20X_WR_CONVERT_CMD | HP20X_CONVERT_OSR1024);
    Wire.endTransmission();
    delay(25);
    Wire.beginTransmission(HP20X_I2C_DEV_ID);
    Wire.write(HP20X_READ_P);
    Wire.endTransmission();
    unsigned long TempData = 0;
    unsigned long tmpArray[3] = {0};
    int cnt = 0;

    /* Require three bytes from slave */
    Wire.requestFrom(HP20X_I2C_DEV_ID, 3);

    while (Wire.available()) // slave may send less than requested
    {
        unsigned char c = Wire.read(); // receive a byte as character
        tmpArray[cnt] = (ulong)c;
        cnt++;
    }

    /* MSB */
    TempData = tmpArray[0] << 16 | tmpArray[1] << 8 | tmpArray[2];

    if (TempData & 0x800000)
    {
        TempData |= 0xff000000;
    }
    return TempData;
}

unsigned long barometer_read_altitude()
{
    Wire.beginTransmission(HP20X_I2C_DEV_ID);
    Wire.write(HP20X_WR_CONVERT_CMD | HP20X_CONVERT_OSR1024);
    Wire.endTransmission();
    delay(25);
    Wire.beginTransmission(HP20X_I2C_DEV_ID);
    Wire.write(HP20X_READ_A);
    Wire.endTransmission();
    unsigned long TempData = 0;
    unsigned long tmpArray[3] = {0};
    int cnt = 0;

    /* Require three bytes from slave */
    Wire.requestFrom(HP20X_I2C_DEV_ID, 3);

    while (Wire.available()) // slave may send less than requested
    {
        unsigned char c = Wire.read(); // receive a byte as character
        tmpArray[cnt] = (ulong)c;
        cnt++;
    }
    /* MSB */
    TempData = tmpArray[0] << 16 | tmpArray[1] << 8 | tmpArray[2];

    if (TempData & 0x800000)
    {
        TempData |= 0xff000000;
    }
    return TempData;
}