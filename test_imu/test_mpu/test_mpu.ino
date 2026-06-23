/*
  Stage 2 — IDLE Dataset Collection
  TinyML Human Activity Recognition

  Calibration Rule:
    Keep the board FLAT, STILL, and in the official IDLE neutral position.
    Do not touch the sensor during calibration.

  CSV Output:
    timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,label
*/

#include <Wire.h>                         /* Include Wire library for I2C communication */

/* =========================
   User Configuration
   ========================= */

#define SDA_PIN 8                         /* ESP32-S3 SDA pin connected to MPU6050 SDA */
#define SCL_PIN 9                         /* ESP32-S3 SCL pin connected to MPU6050 SCL */


const char CURRENT_LABEL[] = "idle";      /* Fixed label for this dataset file */
const unsigned long SAMPLE_DELAY_MS = 50; /* 50 ms = about 20 samples per second */

/* =========================
   MPU6050 Registers
   ========================= */

#define MPU_ADDR 0x68                     /* MPU6050 I2C address when AD0 is connected to GND */
#define REG_WHO_AM_I 0x75                 /* Register used to check sensor identity */
#define REG_PWR_MGMT_1 0x6B               /* Register used to wake up the sensor */
#define REG_ACCEL_CONFIG 0x1C             /* Register used to set accelerometer range */
#define REG_GYRO_CONFIG 0x1B              /* Register used to set gyroscope range */
#define REG_ACCEL_XOUT_H 0x3B             /* First register of accelerometer data block */

/* =========================
   Scale Factors
   ========================= */

const float ACCEL_SCALE = 16384.0f;        /* +/-2g range: 16384 raw counts = 1g */
const float GYRO_SCALE = 131.0f;           /* +/-250 dps range: 131 raw counts = 1 deg/s */

/* =========================
   Calibration Bias
   ========================= */

float accelBiasX = 0.0f;                   /* Accelerometer X bias in g */
float accelBiasY = 0.0f;                   /* Accelerometer Y bias in g */
float accelBiasZ = 0.0f;                   /* Accelerometer Z bias in g */

float gyroBiasX = 0.0f;                    /* Gyroscope X bias in deg/s */
float gyroBiasY = 0.0f;                    /* Gyroscope Y bias in deg/s */
float gyroBiasZ = 0.0f;                    /* Gyroscope Z bias in deg/s */

/* =========================
   Filter Variables
   ========================= */

float filtAx = 0.0f;                       /* Filtered accelerometer X */
float filtAy = 0.0f;                       /* Filtered accelerometer Y */
float filtAz = 0.0f;                       /* Filtered accelerometer Z */

float filtGx = 0.0f;                       /* Filtered gyroscope X */
float filtGy = 0.0f;                       /* Filtered gyroscope Y */
float filtGz = 0.0f;                       /* Filtered gyroscope Z */

const float ALPHA = 0.2f;                  /* Low-pass filter factor */

/* =========================
   MPU6050 Write Function
   ========================= */

void mpuWriteByte(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU_ADDR);      /* Start I2C communication with MPU6050 */
    Wire.write(reg);                       /* Send register address */
    Wire.write(value);                     /* Send value to register */
    Wire.endTransmission();                /* End I2C communication */
}

/* =========================
   MPU6050 Read One Byte
   ========================= */

uint8_t mpuReadByte(uint8_t reg)
{
    Wire.beginTransmission(MPU_ADDR);      /* Start I2C communication with MPU6050 */
    Wire.write(reg);                       /* Send register address */
    Wire.endTransmission(false);           /* Keep bus active using repeated start */

    Wire.requestFrom(MPU_ADDR, 1, true);   /* Request one byte from MPU6050 */

    return Wire.read();                    /* Return received byte */
}

/* =========================
   Read Signed 16-bit Value
   ========================= */

int16_t readInt16()
{
    uint8_t highByte = Wire.read();        /* Read high byte */
    uint8_t lowByte = Wire.read();         /* Read low byte */

    return (int16_t)((highByte << 8) | lowByte); /* Combine two bytes into signed 16-bit value */
}

/* =========================
   Read Raw Sensor Data
   ========================= */

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz)
{
    Wire.beginTransmission(MPU_ADDR);      /* Start I2C communication */
    Wire.write(REG_ACCEL_XOUT_H);          /* Select first data register */

    if (Wire.endTransmission(false) != 0)  /* Check if sensor responded */
    {
        return false;                      /* Return false if I2C failed */
    }

    uint8_t bytesReceived = Wire.requestFrom(MPU_ADDR, 14, true); /* Request acc + temp + gyro */

    if (bytesReceived != 14)               /* Check if all bytes arrived */
    {
        return false;                      /* Return false if data is incomplete */
    }

    ax = readInt16();                      /* Read raw accelerometer X */
    ay = readInt16();                      /* Read raw accelerometer Y */
    az = readInt16();                      /* Read raw accelerometer Z */

    readInt16();                           /* Read and ignore temperature */

    gx = readInt16();                      /* Read raw gyroscope X */
    gy = readInt16();                      /* Read raw gyroscope Y */
    gz = readInt16();                      /* Read raw gyroscope Z */

    return true;                           /* Return true when reading succeeds */
}

/* =========================
   Calibration Function
   ========================= */

void calibrateSensor(int numSamples = 500)
{
    Serial.println("CALIBRATION_START");
    Serial.println("Keep board FLAT, STILL, and in IDLE neutral position");
    Serial.println("Do not touch the board");

    delay(2000);                           /* Give user time before calibration */

    double sumAx = 0.0;                    /* Sum accelerometer X */
    double sumAy = 0.0;                    /* Sum accelerometer Y */
    double sumAz = 0.0;                    /* Sum accelerometer Z */

    double sumGx = 0.0;                    /* Sum gyroscope X */
    double sumGy = 0.0;                    /* Sum gyroscope Y */
    double sumGz = 0.0;                    /* Sum gyroscope Z */

    int validSamples = 0;                  /* Count valid calibration samples */

    for (int i = 0; i < numSamples; i++)   /* Collect calibration samples */
    {
        int16_t axRaw, ayRaw, azRaw;       /* Store raw accelerometer values */
        int16_t gxRaw, gyRaw, gzRaw;       /* Store raw gyroscope values */

        if (mpuReadRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw) == true)
        {
            sumAx += axRaw / ACCEL_SCALE;  /* Convert raw acc X to g and add */
            sumAy += ayRaw / ACCEL_SCALE;  /* Convert raw acc Y to g and add */
            sumAz += azRaw / ACCEL_SCALE;  /* Convert raw acc Z to g and add */

            sumGx += gxRaw / GYRO_SCALE;   /* Convert raw gyro X to deg/s and add */
            sumGy += gyRaw / GYRO_SCALE;   /* Convert raw gyro Y to deg/s and add */
            sumGz += gzRaw / GYRO_SCALE;   /* Convert raw gyro Z to deg/s and add */

            validSamples++;                /* Increase valid samples count */
        }

        delay(2);                          /* Small delay between calibration samples */
    }

    if (validSamples == 0)                 /* Check if calibration failed */
    {
        Serial.println("CALIBRATION_FAILED");
        return;
    }

    accelBiasX = sumAx / validSamples;     /* X should be 0g in idle, so average is bias */
    accelBiasY = sumAy / validSamples;     /* Y should be 0g in idle, so average is bias */
    accelBiasZ = (sumAz / validSamples) - 1.0f; /* Z should be 1g in idle, keep gravity */

    gyroBiasX = sumGx / validSamples;      /* Gyro X should be 0 deg/s */
    gyroBiasY = sumGy / validSamples;      /* Gyro Y should be 0 deg/s */
    gyroBiasZ = sumGz / validSamples;      /* Gyro Z should be 0 deg/s */

    Serial.println("CALIBRATION_DONE");
    Serial.print("AccelBiasX=");
    Serial.println(accelBiasX, 4);
    Serial.print("AccelBiasY=");
    Serial.println(accelBiasY, 4);
    Serial.print("AccelBiasZ=");
    Serial.println(accelBiasZ, 4);
    Serial.print("GyroBiasX=");
    Serial.println(gyroBiasX, 4);
    Serial.print("GyroBiasY=");
    Serial.println(gyroBiasY, 4);
    Serial.print("GyroBiasZ=");
    Serial.println(gyroBiasZ, 4);
}

/* =========================
   Initialize Filter
   ========================= */

void initializeFilterWithFirstSample()
{
    int16_t axRaw, ayRaw, azRaw;           /* Store first raw accelerometer sample */
    int16_t gxRaw, gyRaw, gzRaw;           /* Store first raw gyroscope sample */

    if (mpuReadRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw) == false)
    {
        Serial.println("FILTER_INIT_FAILED");
        return;
    }

    filtAx = (axRaw / ACCEL_SCALE) - accelBiasX; /* Initialize filtered acc X */
    filtAy = (ayRaw / ACCEL_SCALE) - accelBiasY; /* Initialize filtered acc Y */
    filtAz = (azRaw / ACCEL_SCALE) - accelBiasZ; /* Initialize filtered acc Z */

    filtGx = (gxRaw / GYRO_SCALE) - gyroBiasX;   /* Initialize filtered gyro X */
    filtGy = (gyRaw / GYRO_SCALE) - gyroBiasY;   /* Initialize filtered gyro Y */
    filtGz = (gzRaw / GYRO_SCALE) - gyroBiasZ;   /* Initialize filtered gyro Z */

    Serial.println("FILTER_INIT_DONE");
}

/* =========================
   Arduino Setup
   ========================= */

void setup()
{
    Serial.begin(115200);                  /* Start serial communication */
    delay(1000);                           /* Wait for Serial Monitor */

    Wire.begin(SDA_PIN, SCL_PIN);          /* Start I2C with selected pins */
    Wire.setClock(400000);                 /* Set I2C speed to 400 kHz */

    mpuWriteByte(REG_PWR_MGMT_1, 0x00);    /* Wake up MPU6050 */
    delay(100);                            /* Wait for sensor startup */

    uint8_t whoAmI = mpuReadByte(REG_WHO_AM_I); /* Read sensor ID */

    Serial.print("WHO_AM_I=0x");
    Serial.println(whoAmI, HEX);

    if (whoAmI != 0x68)                    /* Check MPU6050 identity */
    {
        Serial.println("ERROR_MPU6050_NOT_FOUND");
        while (1)
        {
            delay(1000);
        }
    }

    mpuWriteByte(REG_ACCEL_CONFIG, 0x00);  /* Set accelerometer range to +/-2g */
    mpuWriteByte(REG_GYRO_CONFIG, 0x00);   /* Set gyroscope range to +/-250 deg/s */

    calibrateSensor(500);                  /* Calibrate in official idle position */

    initializeFilterWithFirstSample();     /* Start filter from first real sample */

    Serial.println("DATASET_COLLECTION_STARTED");
    Serial.print("Current label: ");
    Serial.println(CURRENT_LABEL);

    Serial.println("timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,label");
}

/* =========================
   Arduino Loop
   ========================= */

void loop()
{
    int16_t axRaw, ayRaw, azRaw;           /* Store raw accelerometer readings */
    int16_t gxRaw, gyRaw, gzRaw;           /* Store raw gyroscope readings */

    if (mpuReadRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw) == false)
    {
        Serial.println("SENSOR_READ_FAILED");
        delay(SAMPLE_DELAY_MS);
        return;
    }

    float ax = (axRaw / ACCEL_SCALE) - accelBiasX; /* Convert acc X to g and remove bias */
    float ay = (ayRaw / ACCEL_SCALE) - accelBiasY; /* Convert acc Y to g and remove bias */
    float az = (azRaw / ACCEL_SCALE) - accelBiasZ; /* Convert acc Z to g and remove bias */

    float gx = (gxRaw / GYRO_SCALE) - gyroBiasX;   /* Convert gyro X to dps and remove bias */
    float gy = (gyRaw / GYRO_SCALE) - gyroBiasY;   /* Convert gyro Y to dps and remove bias */
    float gz = (gzRaw / GYRO_SCALE) - gyroBiasZ;   /* Convert gyro Z to dps and remove bias */

    filtAx = (ALPHA * ax) + ((1.0f - ALPHA) * filtAx); /* Filter accelerometer X */
    filtAy = (ALPHA * ay) + ((1.0f - ALPHA) * filtAy); /* Filter accelerometer Y */
    filtAz = (ALPHA * az) + ((1.0f - ALPHA) * filtAz); /* Filter accelerometer Z */

    filtGx = (ALPHA * gx) + ((1.0f - ALPHA) * filtGx); /* Filter gyroscope X */
    filtGy = (ALPHA * gy) + ((1.0f - ALPHA) * filtGy); /* Filter gyroscope Y */
    filtGz = (ALPHA * gz) + ((1.0f - ALPHA) * filtGz); /* Filter gyroscope Z */

    unsigned long timestamp = millis();    /* Get time since ESP32 started */

    Serial.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%s\n",
                  timestamp,
                  filtAx,
                  filtAy,
                  filtAz,
                  filtGx,
                  filtGy,
                  filtGz,
                  CURRENT_LABEL);          /* Print CSV row */

    delay(SAMPLE_DELAY_MS);                /* Wait until next sample */
}