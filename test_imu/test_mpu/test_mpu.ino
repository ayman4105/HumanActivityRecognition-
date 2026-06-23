#include <Wire.h>                         /* Include I2C communication library */

#define MPU6050_ADDR 0x68                 /* Define MPU6050 I2C address */
#define SDA_PIN 4                         /* Define ESP32-S3 SDA pin */
#define SCL_PIN 5                         /* Define ESP32-S3 SCL pin */

#define PWR_MGMT_1 0x6B                   /* Define power management register */
#define ACCEL_CONFIG 0x1C                 /* Define accelerometer config register */
#define GYRO_CONFIG 0x1B                  /* Define gyroscope config register */
#define ACCEL_XOUT_H 0x3B                 /* Define first sensor data register */

#define ACCEL_SCALE 16384.0f              /* Scale for accelerometer at +/-2g */
#define GYRO_SCALE 131.0f                 /* Scale for gyroscope at +/-250 deg/s */

typedef struct                            /* Define raw IMU data structure */
{
    int16_t acc_x;                        /* Store raw accelerometer X */
    int16_t acc_y;                        /* Store raw accelerometer Y */
    int16_t acc_z;                        /* Store raw accelerometer Z */
    int16_t gyro_x;                       /* Store raw gyroscope X */
    int16_t gyro_y;                       /* Store raw gyroscope Y */
    int16_t gyro_z;                       /* Store raw gyroscope Z */
} ImuRaw_t;                               /* Name raw IMU data type */

typedef struct                            /* Define converted IMU data structure */
{
    float acc_x_g;                        /* Store accelerometer X in g */
    float acc_y_g;                        /* Store accelerometer Y in g */
    float acc_z_g;                        /* Store accelerometer Z in g */
    float gyro_x_dps;                     /* Store gyroscope X in degree per second */
    float gyro_y_dps;                     /* Store gyroscope Y in degree per second */
    float gyro_z_dps;                     /* Store gyroscope Z in degree per second */
} ImuData_t;                              /* Name converted IMU data type */

void writeMPU6050(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU6050_ADDR); /* Start I2C transmission to MPU6050 */
    Wire.write(reg);                      /* Send register address */
    Wire.write(value);                    /* Send value to register */
    Wire.endTransmission();               /* End I2C transmission */
}

int16_t readInt16()
{
    uint8_t high_byte = Wire.read();       /* Read high byte */
    uint8_t low_byte = Wire.read();        /* Read low byte */
    return (int16_t)((high_byte << 8) | low_byte); /* Combine bytes into signed 16-bit value */
}

bool readMPU6050Raw(ImuRaw_t *raw_data)
{
    Wire.beginTransmission(MPU6050_ADDR);  /* Start I2C transmission to MPU6050 */
    Wire.write(ACCEL_XOUT_H);             /* Select first data register */
    uint8_t status = Wire.endTransmission(false); /* Send repeated start without releasing bus */

    if (status != 0)                      /* Check if MPU6050 did not respond */
    {
        return false;                     /* Return false if communication failed */
    }

    uint8_t bytes_read = Wire.requestFrom(MPU6050_ADDR, 14); /* Request 14 bytes from sensor */

    if (bytes_read != 14)                 /* Check if reading failed */
    {
        return false;                     /* Return false if data is incomplete */
    }

    raw_data->acc_x = readInt16();        /* Read raw accelerometer X */
    raw_data->acc_y = readInt16();        /* Read raw accelerometer Y */
    raw_data->acc_z = readInt16();        /* Read raw accelerometer Z */

    readInt16();                          /* Read and ignore temperature */

    raw_data->gyro_x = readInt16();       /* Read raw gyroscope X */
    raw_data->gyro_y = readInt16();       /* Read raw gyroscope Y */
    raw_data->gyro_z = readInt16();       /* Read raw gyroscope Z */

    return true;                          /* Return true if reading succeeded */
}

void convertMPU6050Data(const ImuRaw_t *raw_data, ImuData_t *imu_data)
{
    imu_data->acc_x_g = raw_data->acc_x / ACCEL_SCALE;       /* Convert acc X to g */
    imu_data->acc_y_g = raw_data->acc_y / ACCEL_SCALE;       /* Convert acc Y to g */
    imu_data->acc_z_g = raw_data->acc_z / ACCEL_SCALE;       /* Convert acc Z to g */

    imu_data->gyro_x_dps = raw_data->gyro_x / GYRO_SCALE;    /* Convert gyro X to deg/s */
    imu_data->gyro_y_dps = raw_data->gyro_y / GYRO_SCALE;    /* Convert gyro Y to deg/s */
    imu_data->gyro_z_dps = raw_data->gyro_z / GYRO_SCALE;    /* Convert gyro Z to deg/s */
}

void setup()
{
    Serial.begin(115200);                 /* Start serial communication */
    delay(1000);                          /* Wait for serial monitor */

    Wire.begin(SDA_PIN, SCL_PIN);         /* Start I2C using GPIO4 and GPIO5 */
    Wire.setClock(100000);                /* Set I2C speed to 100 kHz */

    writeMPU6050(PWR_MGMT_1, 0x00);       /* Wake up MPU6050 */
    delay(100);                           /* Wait for sensor startup */

    writeMPU6050(ACCEL_CONFIG, 0x00);     /* Set accelerometer range to +/-2g */
    writeMPU6050(GYRO_CONFIG, 0x00);      /* Set gyroscope range to +/-250 deg/s */

    Serial.println("MPU6050 reading started"); /* Print startup message */
}

void loop()
{
    ImuRaw_t raw_data;                    /* Create raw data variable */
    ImuData_t imu_data;                   /* Create converted data variable */

    bool status = readMPU6050Raw(&raw_data); /* Read raw data from MPU6050 */

    if (status == true)                   /* Check if reading succeeded */
    {
        convertMPU6050Data(&raw_data, &imu_data); /* Convert raw data to real units */

        Serial.print("ACC[g] X: ");       /* Print acc X label */
        Serial.print(imu_data.acc_x_g, 3); /* Print acc X value */

        Serial.print(" | Y: ");           /* Print acc Y label */
        Serial.print(imu_data.acc_y_g, 3); /* Print acc Y value */

        Serial.print(" | Z: ");           /* Print acc Z label */
        Serial.print(imu_data.acc_z_g, 3); /* Print acc Z value */

        Serial.print(" || GYRO[dps] X: "); /* Print gyro X label */
        Serial.print(imu_data.gyro_x_dps, 3); /* Print gyro X value */

        Serial.print(" | Y: ");           /* Print gyro Y label */
        Serial.print(imu_data.gyro_y_dps, 3); /* Print gyro Y value */

        Serial.print(" | Z: ");           /* Print gyro Z label */
        Serial.println(imu_data.gyro_z_dps, 3); /* Print gyro Z value */
    }
    else
    {
        Serial.println("Failed to read MPU6050"); /* Print error if reading failed */
    }

    delay(100);                           /* Wait 100 ms between readings */
}