/*
  Stage 1 — Slow Sensor Reading
  TinyML HAR Project

  Goal:
    - Initialize I2C communication with MPU6050
    - Read raw accelerometer and gyroscope values
    - Convert raw values to real units: g and deg/s
    - Calibrate sensor bias at startup
    - Apply simple low-pass filter
    - Print slow and clear readings every 1 second

  Wiring:
    MPU6050 VCC -> ESP32-S3 3.3V
    MPU6050 GND -> ESP32-S3 GND
    MPU6050 SDA -> ESP32-S3 GPIO8
    MPU6050 SCL -> ESP32-S3 GPIO9

  Note:
    If your working pins are GPIO4 and GPIO5, change SDA_PIN and SCL_PIN only.
*/

#include <Wire.h>                         /* Include Wire library for I2C communication */
#include <math.h>                         /* Include math library for sqrt() function */

/* =========================
   I2C Pin Configuration
   ========================= */

#define SDA_PIN 8                         /* ESP32-S3 pin connected to MPU605#include <Wire.h>                         /* Include Wire library for I2C communication */
#include <math.h>                         /* Include math library for sqrt() function */

#define SCL_PIN 9                         /* ESP32-S3 pin connected to MPU6050 SCL */

/* =========================
   MPU6050 Register Addresses
   ========================= */

#define MPU_ADDR 0x68                     /* MPU6050 I2C address when AD0 is connected to GND */
#define REG_WHO_AM_I 0x75                 /* Register used to check sensor identity */
#define REG_PWR_MGMT_1 0x6B               /* Register used to wake up the MPU6050 */
#define REG_ACCEL_CONFIG 0x1C             /* Register used to set accelerometer range */
#define REG_GYRO_CONFIG 0x1B              /* Register used to set gyroscope range */
#define REG_ACCEL_XOUT_H 0x3B             /* First register of accelerometer and gyroscope data block */

/* =========================
   Scale Factors
   ========================= */

const float ACCEL_SCALE = 16384.0f;        /* Accelerometer scale factor for +/-2g: 16384 LSB = 1g */
const float GYRO_SCALE = 131.0f;           /* Gyroscope scale factor for +/-250 deg/s: 131 LSB = 1 deg/s */

/* =========================
   Reading Speed
   ========================= */

const unsigned long READ_DELAY_MS = 1000;  /* Print one reading every 1000 ms = 1 second */

/* =========================
   Calibration Variables
   ========================= */

float accelBiasX = 0.0f;                   /* Store accelerometer X bias in g */
float accelBiasY = 0.0f;                   /* Store accelerometer Y bias in g */
float accelBiasZ = 0.0f;                   /* Store accelerometer Z bias in g */

float gyroBiasX = 0.0f;                    /* Store gyroscope X bias in deg/s */
float gyroBiasY = 0.0f;                    /* Store gyroscope Y bias in deg/s */
float gyroBiasZ = 0.0f;                    /* Store gyroscope Z bias in deg/s */

/* =========================
   Low Pass Filter Variables
   ========================= */

float filtAx = 0.0f;                       /* Store filtered accelerometer X value */
float filtAy = 0.0f;                       /* Store filtered accelerometer Y value */
float filtAz = 0.0f;                       /* Store filtered accelerometer Z value */

float filtGx = 0.0f;                       /* Store filtered gyroscope X value */
float filtGy = 0.0f;                       /* Store filtered gyroscope Y value */
float filtGz = 0.0f;                       /* Store filtered gyroscope Z value */

const float ALPHA = 0.2f;                  /* Low-pass filter factor: 0.2 means smooth output */

/* =========================
   Write One Byte to MPU6050
   ========================= */

void mpuWriteByte(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU_ADDR);      /* Start I2C transmission to MPU6050 */
    Wire.write(reg);                       /* Send register address */
    Wire.write(value);                     /* Send value to register */
    Wire.endTransmission();                /* End I2C transmission */
}

/* =========================
   Read One Byte from MPU6050
   ========================= */

uint8_t mpuReadByte(uint8_t reg)
{
    Wire.beginTransmission(MPU_ADDR);      /* Start I2C transmission to MPU6050 */
    Wire.write(reg);                       /* Send register address */
    Wire.endTransmission(false);           /* Send repeated start without releasing I2C bus */

    Wire.requestFrom(MPU_ADDR, 1, true);   /* Request one byte from MPU6050 */

    return Wire.read();                    /* Return received byte */
}

/* =========================
   Read Signed 16-bit Value
   ========================= */

int16_t readInt16()
{
    uint8_t highByte = Wire.read();        /* Read high byte from I2C buffer */
    uint8_t lowByte = Wire.read();         /* Read low byte from I2C buffer */

    return (int16_t)((highByte << 8) | lowByte); /* Combine high and low bytes into signed 16-bit value */
}

/* =========================
   Read Raw MPU6050 Data
   ========================= */

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz)
{
    Wire.beginTransmission(MPU_ADDR);      /* Start I2C transmission to MPU6050 */
    Wire.write(REG_ACCEL_XOUT_H);          /* Select first data register: ACCEL_XOUT_H */

    if (Wire.endTransmission(false) != 0)  /* Check if MPU6050 responded correctly */
    {
        return false;                      /* Return false if I2C communication failed */
    }

    uint8_t bytesReceived = Wire.requestFrom(MPU_ADDR, 14, true); /* Request 14 bytes: acc + temp + gyro */

    if (bytesReceived != 14)               /* Check if all 14 bytes were received */
    {
        return false;                      /* Return false if reading is incomplete */
    }

    ax = readInt16();                      /* Read raw accelerometer X */
    ay = readInt16();                      /* Read raw accelerometer Y */
    az = readInt16();                      /* Read raw accelerometer Z */

    readInt16();                           /* Read and ignore temperature value */

    gx = readInt16();                      /* Read raw gyroscope X */
    gy = readInt16();                      /* Read raw gyroscope Y */
    gz = readInt16();                      /* Read raw gyroscope Z */

    return true;                           /* Return true if all data was read successfully */
}

/* =========================
   Sensor Calibration
   ========================= */

void calibrateSensor(int numSamples = 500)
{
    Serial.println("======================================");
    Serial.println("Calibration started");
    Serial.println("Keep the sensor FLAT and STILL");
    Serial.println("======================================");

    delay(1000);                           /* Wait one second before starting calibration */

    double sumAx = 0.0;                    /* Store sum of accelerometer X readings */
    double sumAy = 0.0;                    /* Store sum of accelerometer Y readings */
    double sumAz = 0.0;                    /* Store sum of accelerometer Z readings */

    double sumGx = 0.0;                    /* Store sum of gyroscope X readings */
    double sumGy = 0.0;                    /* Store sum of gyroscope Y readings */
    double sumGz = 0.0;                    /* Store sum of gyroscope Z readings */

    int validSamples = 0;                  /* Count valid sensor samples */

    for (int i = 0; i < numSamples; i++)   /* Collect many samples to calculate average bias */
    {
        int16_t axRaw, ayRaw, azRaw;       /* Store raw accelerometer readings */
        int16_t gxRaw, gyRaw, gzRaw;       /* Store raw gyroscope readings */

        if (mpuReadRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw) == true)
        {
            sumAx += axRaw / ACCEL_SCALE;  /* Convert raw acc X to g and add to sum */
            sumAy += ayRaw / ACCEL_SCALE;  /* Convert raw acc Y to g and add to sum */
            sumAz += azRaw / ACCEL_SCALE;  /* Convert raw acc Z to g and add to sum */

            sumGx += gxRaw / GYRO_SCALE;   /* Convert raw gyro X to deg/s and add to sum */
            sumGy += gyRaw / GYRO_SCALE;   /* Convert raw gyro Y to deg/s and add to sum */
            sumGz += gzRaw / GYRO_SCALE;   /* Convert raw gyro Z to deg/s and add to sum */

            validSamples++;                /* Increase valid samples counter */
        }

        delay(2);                          /* Small delay between calibration samples */
    }

    if (validSamples == 0)                 /* Check if calibration failed */
    {
        Serial.println("Calibration failed: no valid samples");
        return;
    }

    accelBiasX = sumAx / validSamples;     /* Calculate average accelerometer X bias */
    accelBiasY = sumAy / validSamples;     /* Calculate average accelerometer Y bias */
    accelBiasZ = (sumAz / validSamples) - 1.0f; /* Calculate Z bias while keeping gravity around 1g */

    gyroBiasX = sumGx / validSamples;      /* Calculate average gyroscope X bias */
    gyroBiasY = sumGy / validSamples;      /* Calculate average gyroscope Y bias */
    gyroBiasZ = sumGz / validSamples;      /* Calculate average gyroscope Z bias */

    Serial.println("Calibration complete");
    Serial.println("--------------------------------------");

    Serial.print("Accel Bias X [g] = ");
    Serial.println(accelBiasX, 4);

    Serial.print("Accel Bias Y [g] = ");
    Serial.println(accelBiasY, 4);

    Serial.print("Accel Bias Z [g] = ");
    Serial.println(accelBiasZ, 4);

    Serial.print("Gyro Bias X [deg/s] = ");
    Serial.println(gyroBiasX, 4);

    Serial.print("Gyro Bias Y [deg/s] = ");
    Serial.println(gyroBiasY, 4);

    Serial.print("Gyro Bias Z [deg/s] = ");
    Serial.println(gyroBiasZ, 4);

    Serial.println("--------------------------------------");
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
        Serial.println("Filter initialization failed"); /* Print error if first reading failed */
        return;                            /* Exit function */
    }

    float ax = (axRaw / ACCEL_SCALE) - accelBiasX; /* Convert and calibrate accelerometer X */
    float ay = (ayRaw / ACCEL_SCALE) - accelBiasY; /* Convert and calibrate accelerometer Y */
    float az = (azRaw / ACCEL_SCALE) - accelBiasZ; /* Convert and calibrate accelerometer Z */

    float gx = (gxRaw / GYRO_SCALE) - gyroBiasX;   /* Convert and calibrate gyroscope X */
    float gy = (gyRaw / GYRO_SCALE) - gyroBiasY;   /* Convert and calibrate gyroscope Y */
    float gz = (gzRaw / GYRO_SCALE) - gyroBiasZ;   /* Convert and calibrate gyroscope Z */

    filtAx = ax;                           /* Start filtered acc X from first real value */
    filtAy = ay;                           /* Start filtered acc Y from first real value */
    filtAz = az;                           /* Start filtered acc Z from first real value */

    filtGx = gx;                           /* Start filtered gyro X from first real value */
    filtGy = gy;                           /* Start filtered gyro Y from first real value */
    filtGz = gz;                           /* Start filtered gyro Z from first real value */

    Serial.println("Filter initialized with first real sample");
}

/* =========================
   Arduino setup()
   ========================= */

void setup()
{
    Serial.begin(115200);                  /* Start Serial Monitor at 115200 baud */
    delay(1000);                           /* Wait for Serial Monitor to become ready */

    Wire.begin(SDA_PIN, SCL_PIN);          /* Start I2C using selected SDA and SCL pins */
    Wire.setClock(400000);                 /* Set I2C speed to 400 kHz */

    mpuWriteByte(REG_PWR_MGMT_1, 0x00);    /* Wake up MPU6050 from sleep mode */
    delay(100);                            /* Wait for sensor startup */

    uint8_t whoAmI = mpuReadByte(REG_WHO_AM_I); /* Read WHO_AM_I register */

    Serial.println("======================================");
    Serial.println("MPU6050 Slow Reading Test");
    Serial.println("======================================");

    Serial.print("WHO_AM_I = 0x");
    Serial.println(whoAmI, HEX);

    if (whoAmI != 0x68)                    /* Check if correct MPU6050 is detected */
    {
        Serial.println("ERROR: MPU6050 not detected");
        Serial.println("Check VCC, GND, SDA, SCL, and AD0");

        while (1)                          /* Stop program if sensor is not detected */
        {
            delay(1000);
        }
    }

    mpuWriteByte(REG_ACCEL_CONFIG, 0x00);  /* Set accelerometer range to +/-2g */
    mpuWriteByte(REG_GYRO_CONFIG, 0x00);   /* Set gyroscope range to +/-250 deg/s */

    calibrateSensor(500);                  /* Calibrate sensor using 500 samples */

    initializeFilterWithFirstSample();     /* Start filter from first real sample instead of zero */

    Serial.println("Live readings started");
    Serial.println("Each row is printed every 1 second");
    Serial.println("--------------------------------------");

    Serial.println("timestamp_ms,acc_x_g,acc_y_g,acc_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,acc_magnitude_g");

    delay(1000);                           /* Wait before live readings */
}

/* =========================
   Arduino loop()
   ========================= */

void loop()
{
    int16_t axRaw, ayRaw, azRaw;           /* Store raw accelerometer values */
    int16_t gxRaw, gyRaw, gzRaw;           /* Store raw gyroscope values */

    if (mpuReadRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw) == false)
    {
        Serial.println("Sensor read failed"); /* Print error if reading failed */
        delay(READ_DELAY_MS);              /* Wait before retry */
        return;                            /* Exit current loop iteration */
    }

    float ax = (axRaw / ACCEL_SCALE) - accelBiasX; /* Convert raw acc X to g and remove bias */
    float ay = (ayRaw / ACCEL_SCALE) - accelBiasY; /* Convert raw acc Y to g and remove bias */
    float az = (azRaw / ACCEL_SCALE) - accelBiasZ; /* Convert raw acc Z to g and remove bias */

    float gx = (gxRaw / GYRO_SCALE) - gyroBiasX;   /* Convert raw gyro X to deg/s and remove bias */
    float gy = (gyRaw / GYRO_SCALE) - gyroBiasY;   /* Convert raw gyro Y to deg/s and remove bias */
    float gz = (gzRaw / GYRO_SCALE) - gyroBiasZ;   /* Convert raw gyro Z to deg/s and remove bias */

    filtAx = (ALPHA * ax) + ((1.0f - ALPHA) * filtAx); /* Apply low-pass filter to acc X */
    filtAy = (ALPHA * ay) + ((1.0f - ALPHA) * filtAy); /* Apply low-pass filter to acc Y */
    filtAz = (ALPHA * az) + ((1.0f - ALPHA) * filtAz); /* Apply low-pass filter to acc Z */

    filtGx = (ALPHA * gx) + ((1.0f - ALPHA) * filtGx); /* Apply low-pass filter to gyro X */
    filtGy = (ALPHA * gy) + ((1.0f - ALPHA) * filtGy); /* Apply low-pass filter to gyro Y */
    filtGz = (ALPHA * gz) + ((1.0f - ALPHA) * filtGz); /* Apply low-pass filter to gyro Z */

    float accMagnitude = sqrt((filtAx * filtAx) + (filtAy * filtAy) + (filtAz * filtAz)); /* Calculate total acceleration magnitude */

    unsigned long timestamp = millis();    /* Get timestamp in milliseconds since ESP32 started */

    Serial.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                  timestamp,
                  filtAx,
                  filtAy,
                  filtAz,
                  filtGx,
                  filtGy,
                  filtGz,
                  accMagnitude);           /* Print one CSV row */

    delay(READ_DELAY_MS);                  /* Wait before next reading */
}