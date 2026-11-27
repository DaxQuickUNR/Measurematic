/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lidar.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "test.h"
#include "ui_panels.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Channel order inside one sample of the flat record array (same as the MPU FIFO order).
   Sample i lives at rec[i * CH_COUNT + CH_AX] ... rec[i * CH_COUNT + CH_GZ]. */
enum { CH_AX, CH_AY, CH_AZ, CH_GX, CH_GY, CH_GZ, CH_COUNT };

/* Two-state Kalman filter for one angle: state = [angle (deg), gyro bias (deg/s)] */
typedef struct
{
    float angle;
    float bias;
    float P[2][2];
} kf_t;

/* Unit quaternion (w, x, y, z) */
typedef struct
{
    float w, x, y, z;
} Quaternion;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR          (0x68 << 1)   /* 7-bit address shifted left */
#define MPU6050_WHO_AM_I      0x75
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_USER_CTRL     0x6A
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_FIFO_EN       0x23
#define MPU6050_FIFO_COUNT_H  0x72
#define MPU6050_FIFO_R_W      0x74
#define MPU6050_ACCEL_XOUT_H  0x3B

/* Sample rate set by the MPU's own clock. Must divide 1000 (1000, 500, 250, 200, 100...).
   1000 Hz needs I2C at 400 kHz. At 100 kHz stay at 500 Hz or below. */
#define SAMPLE_RATE_HZ        1000
#if (1000 % SAMPLE_RATE_HZ) != 0
#error "SAMPLE_RATE_HZ must divide 1000"
#endif

#define MPU6050_DLPF_CFG      0x02          /* 0x01 = 184 Hz, 0x02 = 98 Hz, 0x03 = 42 Hz gyro bandwidth */
#define MPU6050_GYRO_FS       0x08          /* 0x00 = +-250, 0x08 = +-500, 0x10 = +-1000, 0x18 = +-2000 dps */
#define GYRO_LSB_PER_DPS      65.5f         /* 131 for +-250, 65.5 for +-500, 32.8 for +-1000, 16.4 for +-2000 */
#define ACCEL_LSB_PER_G       16384.0f      /* default +-2 g range */

#define FIFO_SAMPLE_BYTES     12            /* accel XYZ + gyro XYZ, 2 bytes each */
#define MAX_SAMPLES           3000          /* 3 s at 1 kHz = 36 KB of RAM */

#define RAD2DEG               57.2957795f

/* dt used for integration. 0 = 1 / SAMPLE_RATE_HZ (from the MPU clock, recommended for short holds).
   1 = press-to-release time divided by the sample count (only worthwhile for long holds, because
   the HAL tick is 1 ms and the window ends are only known to a few ms). */
#define USE_MEASURED_DT       0

#define Short_Press_Length	  250           //Length of User Interface Logic Press


/* Kalman filter tuning (all angles in degrees) */
#define KF_Q_ANGLE            0.002f        /* deg^2/s   : gyro angle noise. Larger = trust accel more */
#define KF_Q_BIAS             0.0001f       /* (deg/s)^2/s : how fast the gyro bias may wander */
#define KF_R_ANGLE            0.5f          /* deg^2     : accel angle noise. Larger = trust accel less */
#define KF_P0_ANGLE           0.1f          /* initial angle uncertainty, deg^2 */
#define KF_P0_BIAS            0.01f         /* initial bias uncertainty, (deg/s)^2 */
#define ACCEL_GATE_G          0.1f          /* skip accel correction when | |a| - 1 g | is larger than this */
#define INIT_SAMPLES          20            /* samples averaged to get the start orientation */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int16_t ax, ay, az, gx, gy, gz, temp_raw;
int32_t off_ax, off_ay, off_az, off_gx, off_gy, off_gz;

//button trigger
volatile uint32_t last_edge_tick = 0;
volatile uint8_t  edge_flag = 0;
static GPIO_PinState last_state;
//MPU6050
static uint8_t mpu_ok = 0;        /* 1 = MPU6050_Init succeeded */
static uint8_t calibrated = 0;    /* 1 = offsets measured, recording allowed */
//recording (flat array: ax, ay, az, gx, gy, gz, ax, ay, ...)
static int16_t  rec[MAX_SAMPLES * CH_COUNT];
static uint32_t rec_n = 0;          /* samples stored so far */
static uint32_t last_drain = 0;     /* tick of the last FIFO drain */
static uint8_t  recording = 0;
static uint8_t  rec_error = 0;      /* I2C failure or FIFO overflow: data has a gap */
static uint8_t  rec_full = 0;       /* RAM buffer filled up: data truncated */
//time between button press and release
uint32_t press_tick = 0;            /* HAL tick when recording started */
uint32_t release_tick = 0;          /* HAL tick when the release was confirmed */
uint32_t hold_ms = 0;               /* release_tick - press_tick */
//results (global so they can be watched in the debugger)
float res_roll0 = 0, res_roll1 = 0;
float res_pitch0 = 0, res_pitch1 = 0;
float res_yaw = 0;                  /* change in yaw, gyro only */
float res_total_deg = 0;            /* single angle between the start and end orientations */
float deg_total_rad = 0;
float distanceAB = 0;
//LiDAR distances (sensor units, see lidar.c), one per button event
uint16_t dist_press = 0, dist_release = 0;
static uint8_t dist_press_ok = 0, dist_release_ok = 0;   /* 1 = a frame was available */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef mpu_write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

/* Returns 0 on success, 1 = no I2C response, 2 = wake-up failed, 3 = filter write failed,
   4 = sample-rate write failed, 5 = gyro-range write failed, 6 = FIFO setup failed */
uint8_t MPU6050_Init(void)
{
    uint8_t who = 0;

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &who, 1, 100) != HAL_OK)
        return 1;

    /* Wake the sensor and clock it from the gyro PLL (more accurate than the internal RC) */
    if (mpu_write(MPU6050_PWR_MGMT_1, 0x01) != HAL_OK) return 2;
    HAL_Delay(100);   /* let the PLL and gyro settle */

    /* Digital low-pass filter. With it enabled the gyro output rate is 1 kHz */
    if (mpu_write(MPU6050_CONFIG, MPU6050_DLPF_CFG) != HAL_OK) return 3;

    /* Sample rate = 1000 / (1 + SMPLRT_DIV) */
    if (mpu_write(MPU6050_SMPLRT_DIV, (uint8_t)(1000 / SAMPLE_RATE_HZ - 1)) != HAL_OK) return 4;

    if (mpu_write(MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS) != HAL_OK) return 5;

    /* FIFO contents: accel XYZ + gyro XYZ (0x78). Order in the FIFO follows register order. */
    if (mpu_write(MPU6050_FIFO_EN, 0x78) != HAL_OK) return 6;

    return 0;
}

HAL_StatusTypeDef MPU6050_ReadAll(void)
{
    uint8_t buf[14];

    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
                                            MPU6050_ACCEL_XOUT_H,
                                            I2C_MEMADD_SIZE_8BIT, buf, 14, 100);
    if (st != HAL_OK) return st;

    ax       = (int16_t)(buf[0]  << 8 | buf[1]);
    ay       = (int16_t)(buf[2]  << 8 | buf[3]);
    az       = (int16_t)(buf[4]  << 8 | buf[5]);
    temp_raw = (int16_t)(buf[6]  << 8 | buf[7]);
    gx       = (int16_t)(buf[8]  << 8 | buf[9]);
    gy       = (int16_t)(buf[10] << 8 | buf[11]);
    gz       = (int16_t)(buf[12] << 8 | buf[13]);
    return HAL_OK;
}

void MPU6050_Calibrate(void)
{
    int32_t sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0;
    const int N = 200;

    UI_Data_t cal = {0};
    UI_ShowPanel(UI_PANEL_BOOTING, &cal);               // bar at 0%

    for (int i = 0; i < N; i++)
    {
        while (MPU6050_ReadAll() != HAL_OK) { }
        sax += ax; say += ay; saz += az;
        sgx += gx; sgy += gy; sgz += gz;
        HAL_Delay(5);

        if (i % 20 == 19)                               // 10 updates over the run
        {
            cal.progress_pct = (uint8_t)(((i + 1) * 100) / N);
            UI_ShowPanel(UI_PANEL_BOOTING, &cal);
        }
    }

    off_ax = sax / N;
    off_ay = say / N;
    off_az = saz / N - 16384;   // assumes the board is flat, so Z should read 1 g
    off_gx = sgx / N;
    off_gy = sgy / N;
    off_gz = sgz / N;
}

/* Clear the MPU FIFO and start collecting samples at SAMPLE_RATE_HZ */
static void fifo_start(void)
{
    mpu_write(MPU6050_USER_CTRL, 0x04);   /* FIFO_RESET (only works while FIFO is disabled) */
    mpu_write(MPU6050_USER_CTRL, 0x40);   /* FIFO_EN */
}

/* The FIFO cannot be read while disabled, so drain it first, then call this */
static void fifo_stop(void)
{
    mpu_write(MPU6050_USER_CTRL, 0x00);
}

/* Move everything currently in the MPU FIFO into the flat rec[] array.
   The MPU's own clock spaces the samples exactly 1/SAMPLE_RATE_HZ apart, no matter
   when this runs. It only has to run before the 1024-byte FIFO fills (~85 ms at 1 kHz). */
static void fifo_drain(void)
{
    uint8_t cnt[2];

    if (rec_error || rec_full) return;

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_FIFO_COUNT_H,
                         I2C_MEMADD_SIZE_8BIT, cnt, 2, 20) != HAL_OK)
    {
        rec_error = 1;
        return;
    }

    uint16_t bytes = ((uint16_t)cnt[0] << 8) | cnt[1];
    if (bytes >= 1024)          /* FIFO overflowed: old data was overwritten */
    {
        rec_error = 1;
        return;
    }

    uint16_t n = bytes / FIFO_SAMPLE_BYTES;   /* whole samples only, a partial one stays for next time */
    while (n > 0)
    {
        if (rec_n >= MAX_SAMPLES)
        {
            rec_full = 1;
            return;
        }

        uint8_t buf[FIFO_SAMPLE_BYTES * 8];
        uint16_t chunk = (n > 8) ? 8 : n;

        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_FIFO_R_W,
                             I2C_MEMADD_SIZE_8BIT, buf, chunk * FIFO_SAMPLE_BYTES, 50) != HAL_OK)
        {
            rec_error = 1;
            return;
        }

        for (uint16_t i = 0; i < chunk && rec_n < MAX_SAMPLES; i++)
        {
            const uint8_t *p = &buf[i * FIFO_SAMPLE_BYTES];
            int16_t *s = &rec[rec_n * CH_COUNT];
            for (uint8_t c = 0; c < CH_COUNT; c++)
            {
                s[c] = (int16_t)((p[2 * c] << 8) | p[2 * c + 1]);
            }
            rec_n++;
        }
        n -= chunk;
    }
}

/* ---- Kalman filter -------------------------------------------------------- */

static void kf_init(kf_t *k, float angle0)
{
    k->angle = angle0;
    k->bias = 0.0f;               /* gyro offsets are already removed by the calibration */
    k->P[0][0] = KF_P0_ANGLE;
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;
    k->P[1][1] = KF_P0_BIAS;
}

/* Predict step. The gyro rate is integrated with the trapezoidal rule:
   angle += dt * ( (rate_prev + rate_cur) / 2 - bias ) */
static void kf_predict(kf_t *k, float rate_prev, float rate_cur, float dt)
{
    float u = 0.5f * (rate_prev + rate_cur);
    k->angle += dt * (u - k->bias);

    /* P = F P F' + Q, with F = [1 -dt; 0 1] */
    float P00 = k->P[0][0], P01 = k->P[0][1], P10 = k->P[1][0], P11 = k->P[1][1];
    k->P[0][0] = P00 - dt * (P01 + P10) + dt * dt * P11 + KF_Q_ANGLE * dt;
    k->P[0][1] = P01 - dt * P11;
    k->P[1][0] = P10 - dt * P11;
    k->P[1][1] = P11 + KF_Q_BIAS * dt;
}

/* Update step with an angle measured from the accelerometer (H = [1 0]) */
static void kf_update(kf_t *k, float meas_angle)
{
    float y = meas_angle - k->angle;
    if (y > 180.0f)       y -= 360.0f;     /* handle the +-180 degree wrap of atan2 */
    else if (y < -180.0f) y += 360.0f;

    float S  = k->P[0][0] + KF_R_ANGLE;
    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;

    k->angle += K0 * y;
    k->bias  += K1 * y;

    float P00 = k->P[0][0], P01 = k->P[0][1];
    k->P[0][0] -= K0 * P00;
    k->P[0][1] -= K0 * P01;
    k->P[1][0] -= K1 * P00;
    k->P[1][1] -= K1 * P01;
}

/* ---- Orientation change as a single angle (quaternions) ------------------- */

/* Intrinsic ZYX Euler angles (yaw about Z, pitch about Y, roll about X) to a quaternion */
static Quaternion euler_to_quaternion(float yaw_deg, float pitch_deg, float roll_deg)
{
    float r = roll_deg  * (0.5f / RAD2DEG);   /* half angles, in radians */
    float p = pitch_deg * (0.5f / RAD2DEG);
    float y = yaw_deg   * (0.5f / RAD2DEG);

    float cr = cosf(r), sr = sinf(r);
    float cp = cosf(p), sp = sinf(p);
    float cy = cosf(y), sy = sinf(y);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

/* Conjugate (the inverse, for a unit quaternion) */
static Quaternion quaternion_conjugate(Quaternion q)
{
    Quaternion c = { q.w, -q.x, -q.y, -q.z };
    return c;
}

/* q = q1 * q2 */
static Quaternion quaternion_multiply(Quaternion q1, Quaternion q2)
{
    Quaternion q;
    q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    q.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return q;
}

/* Total rotation angle in degrees between two orientations: the angle of q1^-1 * q2 */
static float calculate_angle_change(float yaw1, float pitch1, float roll1,
                                    float yaw2, float pitch2, float roll2)
{
    Quaternion q1 = euler_to_quaternion(yaw1, pitch1, roll1);
    Quaternion q2 = euler_to_quaternion(yaw2, pitch2, roll2);
    Quaternion d  = quaternion_multiply(quaternion_conjugate(q1), q2);

    /* 2 * atan2(|vector part|, |w|) gives the same angle as 2 * acos(|w|), but acos loses
       precision for small angles in 32-bit float. No clamping of w is needed either. */
    float v = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
    return 2.0f * atan2f(v, fabsf(d.w)) * RAD2DEG;
}

/* Runs the filter over the whole recording and fills the res_* variables.
   Roll (about X) and pitch (about Y): gyro prediction + accelerometer correction.
   Yaw (about Z): gyro only, gravity gives no yaw reference. */
static void estimate_angles(void)
{
    float dt = 1.0f / SAMPLE_RATE_HZ;
#if USE_MEASURED_DT
    if (!rec_full && hold_ms > 0)
    {
        dt = ((float)hold_ms / 1000.0f) / (float)rec_n;
    }
#endif

    const float gk = 1.0f / GYRO_LSB_PER_DPS;     /* raw -> deg/s */
    const float ak = 1.0f / ACCEL_LSB_PER_G;      /* raw -> g */

    /* Start orientation: average the accelerometer over the first few samples */
    uint32_t n0 = (rec_n < INIT_SAMPLES) ? rec_n : INIT_SAMPLES;
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    for (uint32_t i = 0; i < n0; i++)
    {
        const int16_t *s = &rec[i * CH_COUNT];
        sx += (float)(s[CH_AX] - off_ax);
        sy += (float)(s[CH_AY] - off_ay);
        sz += (float)(s[CH_AZ] - off_az);
    }
    sx /= (float)n0; sy /= (float)n0; sz /= (float)n0;

    float roll0  = atan2f(sy, sz) * RAD2DEG;
    float pitch0 = atan2f(-sx, sqrtf(sy * sy + sz * sz)) * RAD2DEG;

    kf_t kr, kp;
    kf_init(&kr, roll0);
    kf_init(&kp, pitch0);
    float yaw = 0.0f;

    /* Rates of the previous sample (needed for the trapezoid). Sample 0 is the start state. */
    const int16_t *s0 = &rec[0];
    float gx_prev = (float)(s0[CH_GX] - off_gx) * gk;
    float gy_prev = (float)(s0[CH_GY] - off_gy) * gk;
    float gz_prev = (float)(s0[CH_GZ] - off_gz) * gk;

    for (uint32_t i = 1; i < rec_n; i++)
    {
        const int16_t *s = &rec[i * CH_COUNT];

        float gxr = (float)(s[CH_GX] - off_gx) * gk;
        float gyr = (float)(s[CH_GY] - off_gy) * gk;
        float gzr = (float)(s[CH_GZ] - off_gz) * gk;

        /* Predict: trapezoidal integration of the gyro */
        kf_predict(&kr, gx_prev, gxr, dt);
        kf_predict(&kp, gy_prev, gyr, dt);
        yaw += 0.5f * (gz_prev + gzr) * dt;

        /* Correct: only when the board is not accelerating (|a| close to 1 g) */
        float axg = (float)(s[CH_AX] - off_ax) * ak;
        float ayg = (float)(s[CH_AY] - off_ay) * ak;
        float azg = (float)(s[CH_AZ] - off_az) * ak;
        float amag = sqrtf(axg * axg + ayg * ayg + azg * azg);

        if (fabsf(amag - 1.0f) < ACCEL_GATE_G)
        {
            kf_update(&kr, atan2f(ayg, azg) * RAD2DEG);
            kf_update(&kp, atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * RAD2DEG);
        }

        gx_prev = gxr;
        gy_prev = gyr;
        gz_prev = gzr;
    }

    res_roll0  = roll0;
    res_roll1  = kr.angle;
    res_pitch0 = pitch0;
    res_pitch1 = kp.angle;
    res_yaw    = yaw;

    /* Start orientation: yaw 0 (yaw is relative), end orientation: filtered roll/pitch + yaw change */

    float yaw_end = yaw;
    res_total_deg = calculate_angle_change(0.0f, res_pitch0, res_roll0,
                                           yaw_end, res_pitch1, res_roll1);
}

/* Prints a float as degrees with 2 decimals (nano newlib has no %f) */
static void fmt_deg(char *out, size_t n, float deg)
{
    long c = (long)(deg * 100.0f);   // centi-degrees
    snprintf(out, n, "%s%ld.%02ld", (c < 0) ? "-" : "", labs(c) / 100, labs(c) % 100);
}

static void distanceAB_calc(unsigned dist_press, unsigned dist_release, float res_total_deg)
{
	deg_total_rad = res_total_deg * (3.141592653589793 / 180);
	distanceAB = sqrt(pow(dist_press, 2) + pow(dist_release, 2) - (2 * dist_press * dist_release * cos(deg_total_rad)));
//    long c = (long)(deg * 100.0f);   // centi-degrees
//    snprintf(out, n, "%s%ld.%02ld", (c < 0) ? "-" : "", labs(c) / 100, labs(c) % 100);
}

/* Prints the two LiDAR distances captured at press and release */
/* Prints LiDAR distance(s). press_type: 0 = short press, 1 = long press */
/* Prints LiDAR distance(s). press_type: 0 = short press, 1 = long press */
static void report_distance(int press_type)
{
    char buf[128];
    int len = 0;

    if (press_type == 0)   // Short press: press distance only
    {
        if (dist_press_ok)
        {
        	UI_Data_t ui = {0};
			ui.distance_cm = (int32_t)dist_press;
			UI_ShowPanel(UI_PANEL_DIST_TARGET, &ui);

			len = snprintf(buf, sizeof(buf), "Distance:%u cm\r\n", (unsigned)dist_press);
        }
        else
            len = snprintf(buf, sizeof(buf), "LiDAR: no frame\r\n");
    }
    else if (press_type == 1)   // Long press: both distances
    {
        if (dist_press_ok && dist_release_ok)
        {
            //
            char a[16]; //, b[16], c[16];

            if (rec_error)
            {
                len = snprintf(buf, sizeof(buf), "ERROR: FIFO overflow or I2C failure, recording invalid\r\n");
                HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
                return;
            }

            if (rec_n < 2)
            {
                len = snprintf(buf, sizeof(buf), "Recording too short (%lu samples)\r\n", (unsigned long)rec_n);
                HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
                return;
            }

            estimate_angles();

        //    len = snprintf(buf, sizeof(buf), "Hold:%lu ms  Samples:%lu (%lu ms at %d Hz)%s\r\n",
        //                   (unsigned long)hold_ms,
        //                   (unsigned long)rec_n,
        //                   (unsigned long)(rec_n * 1000UL / SAMPLE_RATE_HZ),
        //                   SAMPLE_RATE_HZ,
        //                   rec_full ? "  BUFFER FULL, truncated" : "");
        //    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
        //
        //    fmt_deg(a, sizeof(a), res_roll0);
        //    fmt_deg(b, sizeof(b), res_roll1);
        //    fmt_deg(c, sizeof(c), res_roll1 - res_roll0);
        //    len = snprintf(buf, sizeof(buf), "Roll  start:%s end:%s delta:%s deg\r\n", a, b, c);
        //    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
        //
        //    fmt_deg(a, sizeof(a), res_pitch0);
        //    fmt_deg(b, sizeof(b), res_pitch1);
        //    fmt_deg(c, sizeof(c), res_pitch1 - res_pitch0);
        //    len = snprintf(buf, sizeof(buf), "Pitch start:%s end:%s delta:%s deg\r\n", a, b, c);
        //    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
        //
        //    fmt_deg(a, sizeof(a), res_yaw);
        //    len = snprintf(buf, sizeof(buf), "Yaw   delta:%s deg (gyro only)\r\n", a);
        //    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);

            fmt_deg(a, sizeof(a), res_total_deg);
            distanceAB_calc(dist_press, dist_release, res_total_deg);

            UI_Data_t ui = {0};
            ui.target_a_cm = (int32_t)dist_press;
            ui.target_b_cm = (int32_t)dist_release;
            ui.between_cm  = (int32_t)(distanceAB + 0.5f);   // +0.5 rounds instead of cutting off
            UI_ShowPanel(UI_PANEL_DIST_BETWEEN, &ui);

			len = snprintf(buf, sizeof(buf), "Angle Change: %s deg\r\n", a);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);

            len = snprintf(buf, sizeof(buf), "Distance from A to B: %u cm\r\nDistance A: %u cm\r\nDistance B: %u cm\r\n",
                           (unsigned)distanceAB, (unsigned)dist_press, (unsigned)dist_release);
        }
        else
            len = snprintf(buf, sizeof(buf), "LiDAR: no frames received\r\n");
    }

    if (len > 0)
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  //UI_Test_Cycle();   /* TEMP: cycles all 9 UI panels forever; replace with the real state machine */
  UI_Init();
  //UI_Data_t = {0};
  UI_ShowPanel(UI_PANEL_AGAIN, NULL);
  HAL_Delay(5000);

  UI_ShowPanel(UI_PANEL_BOOTING, NULL);   // prompt text with an empty bar

  //Trigger Button
  last_state = HAL_GPIO_ReadPin(Trigger_Button_GPIO_Port, Trigger_Button_Pin);
  //LiDAR (USART1 receive interrupt)
  lidar_init();



  //MPU6050
  uint8_t status = MPU6050_Init();
  if (status != 0)
  {
      char err[32];
      int n = snprintf(err, sizeof(err), "MPU init failed: %d\r\n", status);
      HAL_UART_Transmit(&huart2, (uint8_t*)err, n, 100);
  }
  else
  {
      mpu_ok = 1;

      char rdy[] = "Press the button to calibrate (keep the board flat and still).\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)rdy, sizeof(rdy) - 1, 100);
  }
  /* USER CODE END 2 */

  /* Initialize leds */
  //BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  //BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_SPI_MspInit(&hspi1);   // BSP_LED_Init took PA5 (SCK) away from SPI1; give it back
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // While recording, empty the MPU FIFO every 5 ms (it holds ~85 ms at 1 kHz).
    if (recording && (HAL_GetTick() - last_drain) >= 5)
    {
        last_drain = HAL_GetTick();
        fifo_drain();
    }

    if (edge_flag && (HAL_GetTick() - last_edge_tick) >= 20)  // 20 ms settle time
    {
        edge_flag = 0;
        GPIO_PinState state = HAL_GPIO_ReadPin(Trigger_Button_GPIO_Port, Trigger_Button_Pin);

        if (state != last_state)   // real change, not just bounce
        {
            last_state = state;

            if (state == GPIO_PIN_RESET)
            {
            	if (!calibrated)
            	    {
            	        // FIRST press: calibrate instead of recording
            	        if (!mpu_ok)
            	        {
            	            HAL_UART_Transmit(&huart2, (uint8_t*)"MPU not ready\r\n", 15, 100);
            	        }
            	        else
            	        {
            	            HAL_UART_Transmit(&huart2, (uint8_t*)"Calibrating, keep still...\r\n", 28, 100);
            	            HAL_Delay(300);                 // let the board settle after the click
            	            MPU6050_Calibrate();            // ~1 s
            	            calibrated = 1;

            	            edge_flag = 0;                  // forget any bounce or release seen during calibration
            	            last_state = HAL_GPIO_ReadPin(Trigger_Button_GPIO_Port, Trigger_Button_Pin);

            	            UI_ShowPanel(UI_PANEL_READY, NULL);
            	            HAL_UART_Transmit(&huart2, (uint8_t*)"Calibrated. Hold the button to record.\r\n", 40, 100);
            	        }
            	    }
            	else
            	{
					// FALLING edge: button pressed (active low) -> start recording
					dist_press_ok = lidar_get_distance(&dist_press);   // LiDAR reading at press
					rec_n = 0;
					rec_error = 0;
					rec_full = 0;
					fifo_start();
					press_tick = HAL_GetTick();
					last_drain = press_tick;
					recording = 1;

					UI_ShowPanel(UI_PANEL_CROSSHAIR_TWO, NULL);

					char msg[] = "\n\n\n\nRecording...\r\n";
					HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg) - 1, 100);
            	}
            }
            else if (recording)
            {
                // RISING edge: button released get timing
                release_tick = HAL_GetTick();
                hold_ms = release_tick - press_tick;
                fifo_drain();
                fifo_stop();
                recording = 0;

                if (hold_ms <= Short_Press_Length)
                {
                    // SHORT: 1 target (press only)
                    dist_release_ok = 0;        // no release reading, don't print stale one
                    report_distance(0);
                }
                else
                {
                    // LONG: 2 targets + AB distance + angle change
                    dist_release_ok = lidar_get_distance(&dist_release);
                    //report_recording();
                    report_distance(1);
                }
            }
        }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OLED_RST_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Trigger_Button_Pin */
  GPIO_InitStruct.Pin = Trigger_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Trigger_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_RST_Pin OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_RST_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_CS_Pin */
  GPIO_InitStruct.Pin = OLED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Button Trigger
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Trigger_Button_Pin)
    {
        last_edge_tick = HAL_GetTick();
        edge_flag = 1;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
