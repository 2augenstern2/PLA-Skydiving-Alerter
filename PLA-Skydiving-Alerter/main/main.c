// ═══════════════════════════════════════════════════════════════
//  PLA v1 Final  —  Parachute Landing Assistant
//  ESP32-S3 + ICM-42688-P + TFmini Plus + SSD1680 + Buzzer
//
//  状态1 (地面冷启动)：IMU校准10秒 → 保存RTC → 检测ARM
//    → ARM关：Deep Sleep等待唤醒
//    → ARM开：直接进入ARMED
//  状态2 (空中唤醒)：读RTC跳过校准 → 激活传感器 → BB BB → ARMED
//
//  引脚：
//  SPI2=IMU(SCK=48,MOSI=38,MISO=47,CS=6)
//  SPI3=EPD(SCK=13,MOSI=14,CS=8,DC=9,RST=10,BUSY=7)
//  UART1=TFmini(TX=43,RX=44)
//  GPIO4=蜂鸣器  GPIO46=WiFi按键(长按6s)  GPIO17=ARM开关(D8)
// ═══════════════════════════════════════════════════════════════

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "PLA";

#define PIN_IMU_SCK     ((gpio_num_t)48)
#define PIN_IMU_MOSI    ((gpio_num_t)38)
#define PIN_IMU_MISO    ((gpio_num_t)47)
#define PIN_IMU_CS      ((gpio_num_t)6)
#define PIN_EPD_SCK     ((gpio_num_t)13)
#define PIN_EPD_MOSI    ((gpio_num_t)14)
#define PIN_EPD_CS      ((gpio_num_t)8)
#define PIN_EPD_DC      ((gpio_num_t)9)
#define PIN_EPD_RST     ((gpio_num_t)10)
#define PIN_EPD_BUSY    ((gpio_num_t)7)
#define PIN_TF_TX       ((gpio_num_t)43)  // TX1 (ESP TX -> TFmini RX, 绿线)
#define PIN_TF_RX       ((gpio_num_t)44)  // RX0 (ESP RX <- TFmini TX, 白线)
#define PIN_BUZZER      ((gpio_num_t)4)
#define PIN_WIFI_BTN    ((gpio_num_t)46)
#define PIN_ARM         ((gpio_num_t)17)

#define EPD_W     152
#define EPD_H     152
#define EPD_BYTES 2888

// ── RTC内存 ───────────────────────────────────────────────────
typedef struct {
    float q0,q1,q2,q3;
    float gx_bias,gy_bias,gz_bias;
    bool  valid;
} RtcCalib;

RTC_DATA_ATTR static RtcCalib  rtc_calib    = {.valid=false};
RTC_DATA_ATTR static uint32_t  rtc_boot_cnt = 0;

// ── NVS配置 ───────────────────────────────────────────────────
typedef struct {
    float wing_loading;
    uint16_t h_offset_cm;
    float stage1_h, stage2_h;
} PlaConfig;

static PlaConfig g_cfg = {
    .wing_loading=0.93f, .h_offset_cm=85,
    .stage1_h=4.0f, .stage2_h=1.2f
};

static void nvs_load(void) {
    nvs_flash_init();
    nvs_handle_t h;
    size_t sz = sizeof(PlaConfig);
    if (nvs_open("pla_cfg", NVS_READONLY, &h) != ESP_OK) return;
    nvs_get_blob(h, "cfg", &g_cfg, &sz);
    nvs_close(h);
}
static void nvs_save_cfg(void) {
    nvs_handle_t h;
    if (nvs_open("pla_cfg", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "cfg", &g_cfg, sizeof(PlaConfig));
    nvs_commit(h);
    nvs_close(h);
}

// ── 蜂鸣器 ───────────────────────────────────────────────────
typedef enum { BUZZ_OFF=0, BUZZ_STAGE1, BUZZ_STAGE2 } BuzzMode;
static volatile BuzzMode g_buzz_mode = BUZZ_OFF;
static volatile bool g_buzzer_suspend = false;

static void buzz_init(void) {
    gpio_set_direction(PIN_BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_BUZZER, 0);
}
static void buzz_beep(uint32_t ms) {
    g_buzzer_suspend = true;
    gpio_set_level(PIN_BUZZER, 1);
    vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_set_level(PIN_BUZZER, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    g_buzzer_suspend = false;
}

static void task_buzzer(void *a) {
    bool s1 = false;
    for (;;) {
        if (g_buzzer_suspend) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        switch (g_buzz_mode) {
        case BUZZ_OFF:
            gpio_set_level(PIN_BUZZER, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            break;
        case BUZZ_STAGE1:
            s1 = !s1;
            gpio_set_level(PIN_BUZZER, s1 ? 1 : 0);
            vTaskDelay(pdMS_TO_TICKS(80));
            break;
        case BUZZ_STAGE2:
            gpio_set_level(PIN_BUZZER, 1);
            vTaskDelay(pdMS_TO_TICKS(20));
            break;
        }
    }
}

// ── IMU SPI ───────────────────────────────────────────────────
static spi_device_handle_t imu_spi;

static uint8_t imu_read_reg(uint8_t r) {
    uint8_t tx[2] = {r|0x80, 0}, rx[2] = {0};
    spi_transaction_t t = {.length=16, .tx_buffer=tx, .rx_buffer=rx};
    spi_device_polling_transmit(imu_spi, &t);
    return rx[1];
}
static void imu_write_reg(uint8_t r, uint8_t v) {
    uint8_t tx[2] = {r&0x7F, v};
    spi_transaction_t t = {.length=16, .tx_buffer=tx};
    spi_device_polling_transmit(imu_spi, &t);
}
static void imu_read_regs(uint8_t r, uint8_t *buf, int len) {
    uint8_t tx[len+1], rx[len+1];
    memset(tx, 0, len+1);
    tx[0] = r|0x80;
    spi_transaction_t t = {.length=(len+1)*8, .tx_buffer=tx, .rx_buffer=rx};
    spi_device_polling_transmit(imu_spi, &t);
    memcpy(buf, &rx[1], len);
}
static bool imu_init(void) {
    imu_write_reg(0x11, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    if (imu_read_reg(0x75) != 0x47) return false;
    imu_write_reg(0x50, 0x26);
    imu_write_reg(0x4F, 0x06);
    imu_write_reg(0x4E, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(30));
    return true;
}
static void imu_sleep_mode(void) { imu_write_reg(0x4E, 0x00); }
static void imu_wakeup(void) {
    imu_write_reg(0x4E, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(30));
}

// ── Mahony ────────────────────────────────────────────────────
#define MKP 2.0f
#define MKI 0.005f
#define MDT 0.010f

typedef struct {
    float q0,q1,q2,q3;
    float ex_i,ey_i,ez_i;
    float gx_b,gy_b,gz_b;
    float R[3][3];
    float pitch, roll, err;
    bool conv;
} Mahony;

static Mahony g_mahony;

static void m_init_fresh(Mahony *f) {
    f->q0=1; f->q1=f->q2=f->q3=0;
    f->ex_i=f->ey_i=f->ez_i=0;
    f->gx_b=f->gy_b=f->gz_b=0;
    f->err=1; f->conv=false;
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            f->R[i][j] = (i==j) ? 1 : 0;
}
//static void m_init_rtc(Mahony *f, const RtcCalib *c) {
//    f->q0=c->q0; f->q1=c->q1; f->q2=c->q2; f->q3=c->q3;
//    f->gx_b=c->gx_bias; f->gy_b=c->gy_bias; f->gz_b=c->gz_bias;
//    f->ex_i=f->ey_i=f->ez_i=0;
//    f->err=1; f->conv=false;
//}

static void m_update(Mahony *f,
                     float gx, float gy, float gz,
                     float ax, float ay, float az) {
    gx -= f->gx_b; gy -= f->gy_b; gz -= f->gz_b;
    float q0=f->q0, q1=f->q1, q2=f->q2, q3=f->q3;
    float ex=0, ey=0, ez=0;
    float am = sqrtf(ax*ax + ay*ay + az*az);
    if (am > 0.3f) {
        float inv = 1.0f/am;
        ax*=inv; ay*=inv; az*=inv;
        float vx=2*(q1*q3-q0*q2), vy=2*(q0*q1+q2*q3);
        float vz=q0*q0-q1*q1-q2*q2+q3*q3;
        ex=ay*vz-az*vy; ey=az*vx-ax*vz; ez=ax*vy-ay*vx;
        f->err = sqrtf(ex*ex+ey*ey+ez*ez);
        f->conv = (f->err < 0.05f);
        f->ex_i += MKI*ex*MDT;
        f->ey_i += MKI*ey*MDT;
        f->ez_i += MKI*ez*MDT;
    }
    gx += MKP*ex+f->ex_i;
    gy += MKP*ey+f->ey_i;
    gz += MKP*ez+f->ez_i;
    q0 += 0.5f*(-q1*gx-q2*gy-q3*gz)*MDT;
    q1 += 0.5f*( q0*gx+q2*gz-q3*gy)*MDT;
    q2 += 0.5f*( q0*gy-q1*gz+q3*gx)*MDT;
    q3 += 0.5f*( q0*gz+q1*gy-q2*gx)*MDT;
    float n = 1.0f/sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
    q0*=n; q1*=n; q2*=n; q3*=n;
    f->q0=q0; f->q1=q1; f->q2=q2; f->q3=q3;
    f->R[0][0]=1-2*(q2*q2+q3*q3); f->R[0][1]=2*(q1*q2-q0*q3); f->R[0][2]=2*(q1*q3+q0*q2);
    f->R[1][0]=2*(q1*q2+q0*q3);   f->R[1][1]=1-2*(q1*q1+q3*q3); f->R[1][2]=2*(q2*q3-q0*q1);
    f->R[2][0]=2*(q1*q3-q0*q2);   f->R[2][1]=2*(q2*q3+q0*q1);   f->R[2][2]=1-2*(q1*q1+q2*q2);
    f->pitch = asinf(-f->R[2][0]) * 180.0f/3.14159f;
    f->roll  = atan2f(f->R[2][1], f->R[2][2]) * 180.0f/3.14159f;
}

static void imu_read_raw(float *ax, float *ay, float *az,
                         float *gx, float *gy, float *gz) {
    uint8_t raw[12];
    imu_read_regs(0x1F, raw, 12);
    *ax = (int16_t)((raw[0]<<8)|raw[1]) / 4096.0f;
    *ay = (int16_t)((raw[2]<<8)|raw[3]) / 4096.0f;
    *az = (int16_t)((raw[4]<<8)|raw[5]) / 4096.0f;
    *gx = (int16_t)((raw[6]<<8)|raw[7])  / 16.4f * (3.14159f/180.0f);
    *gy = (int16_t)((raw[8]<<8)|raw[9])  / 16.4f * (3.14159f/180.0f);
    *gz = (int16_t)((raw[10]<<8)|raw[11]) / 16.4f * (3.14159f/180.0f);
}

static void calibrate_gyro(Mahony *f) {
    ESP_LOGI(TAG, "Gyro calibration 10s...");
    const int N = 1000;
    double sx=0, sy=0, sz=0;
    for (int i=0; i<N; i++) {
        float ax,ay,az,gx,gy,gz;
        imu_read_raw(&ax,&ay,&az,&gx,&gy,&gz);
        sx+=gx; sy+=gy; sz+=gz;
        float bgx=sx/(i+1), bgy=sy/(i+1), bgz=sz/(i+1);
        m_update(f, gx-bgx, gy-bgy, gz-bgz, ax, ay, az);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    f->gx_b=(float)(sx/N);
    f->gy_b=(float)(sy/N);
    f->gz_b=(float)(sz/N);
    ESP_LOGI(TAG, "Bias: %.4f %.4f %.4f", f->gx_b, f->gy_b, f->gz_b);
}

// ── Kalman ────────────────────────────────────────────────────
typedef struct { float h,vz,P[2][2]; bool inited; } KF;
static KF g_kf;

static void kf_init(KF *k, float h0) {
    k->h=h0; k->vz=0;
    k->P[0][0]=1; k->P[0][1]=0;
    k->P[1][0]=0; k->P[1][1]=1;
    k->inited=true;
}
static void kf_predict(KF *k, float az) {
    float dt=MDT;
    float h  = k->h  + k->vz*dt + 0.5f*az*dt*dt;
    float vz = k->vz + az*dt;
    float p00 = k->P[0][0]+dt*k->P[1][0]+dt*(k->P[0][1]+dt*k->P[1][1])+0.001f;
    float p01 = k->P[0][1]+dt*k->P[1][1];
    float p10 = k->P[1][0]+dt*k->P[1][1];
    float p11 = k->P[1][1]+0.05f;
    k->h=(h<0)?0:h; k->vz=vz;
    k->P[0][0]=p00; k->P[0][1]=p01;
    k->P[1][0]=p10; k->P[1][1]=p11;
}
static void kf_update(KF *k, float z) {
    float S  = k->P[0][0]+0.0025f;
    float K0 = k->P[0][0]/S;
    float K1 = k->P[1][0]/S;
    float inn = z - k->h;
    k->h  += K0*inn;
    k->vz += K1*inn;
    float p00=(1-K0)*k->P[0][0], p01=(1-K0)*k->P[0][1];
    float p10=k->P[1][0]-K1*k->P[0][0], p11=k->P[1][1]-K1*k->P[0][1];
    k->P[0][0]=p00; k->P[0][1]=p01;
    k->P[1][0]=p10; k->P[1][1]=p11;
    if (k->h < 0) k->h=0;
}

// ── TFmini ────────────────────────────────────────────────────
typedef struct { uint16_t dist_cm, strength; bool valid; } TFFrame;
static volatile uint16_t g_dist_cm = 0;
static volatile bool     g_lidar_ok = false;
static volatile bool     g_lidar_initialized = false;

static void tfmini_init(void) {
    // 先卸载防止重复安装
    uart_driver_delete(UART_NUM_1);
    vTaskDelay(pdMS_TO_TICKS(10));

    uart_config_t cfg = {
        .baud_rate=115200, .data_bits=UART_DATA_8_BITS,
        .parity=UART_PARITY_DISABLE, .stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE, .source_clk=UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, PIN_TF_TX, PIN_TF_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // 清空缓冲区，丢弃深度休眠前残留数据
    uart_flush(UART_NUM_1);
    vTaskDelay(pdMS_TO_TICKS(500));  // 等TFmini完全稳定（重要）
    ESP_LOGI("TF","TFmini init done, flushed");
}

static bool tfmini_read_frame(TFFrame *f, uint32_t tms) {
    uint8_t b, buf[9];
    // 防呆：pdMS_TO_TICKS在100Hz tick下最小单位10ms，小于10ms的值变0
    TickType_t wait_ticks = pdMS_TO_TICKS(tms);
    if (wait_ticks == 0) wait_ticks = 1;
    TickType_t d = xTaskGetTickCount() + wait_ticks;

    while (xTaskGetTickCount() < d) {
        // 硬编码1 Tick(10ms)，防止变成0的非阻塞轮询
        if (uart_read_bytes(UART_NUM_1, &b, 1, 1) != 1) return false;
        if (b != 0x59) continue;
        buf[0] = b;

        if (uart_read_bytes(UART_NUM_1, &b, 1, 1) != 1) return false;
        if (b != 0x59) continue;
        buf[1] = b;

        // 后7字节给2个Tick裕量
        if (uart_read_bytes(UART_NUM_1, &buf[2], 7, 2) != 7) return false;

        uint8_t cs = 0;
        for (int i=0; i<8; i++) cs += buf[i];
        if (cs != buf[8]) return false;

        f->dist_cm  = buf[2] | (buf[3]<<8);
        f->strength = buf[4] | (buf[5]<<8);
        f->valid = (f->strength>100) && (f->strength!=65535)
                && (f->dist_cm>=10)  && (f->dist_cm<=1200);
        return true;
    }
    return false;
}

static void tfmini_sleep(void) {
    // 手册6.4: 低功耗模式使能 X=1, 5A 06 35 01 00 SU
    uint8_t cmd[] = {0x5A,0x06,0x35,0x01,0x00,0x96};
    uart_write_bytes(UART_NUM_1, cmd, 6);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI("TF","TFmini sleep");
}
static void tfmini_wake(void) {
    // 手册6.4: 关闭低功耗 X=0, 5A 06 35 00 00 SU
    uint8_t cmd_wake[] = {0x5A,0x06,0x35,0x00,0x00,0x95};
    uart_write_bytes(UART_NUM_1, cmd_wake, 6);
    vTaskDelay(pdMS_TO_TICKS(100));
    // 手册注: 退出低功耗后帧率保持低功耗值，需手动恢复100Hz
    uint8_t cmd_fps[] = {0x5A,0x06,0x03,0x64,0x00,0xC7};
    uart_write_bytes(UART_NUM_1, cmd_fps, 6);
    vTaskDelay(pdMS_TO_TICKS(500));  // 等稳定
    uart_flush(UART_NUM_1);
    ESP_LOGI("TF","TFmini wake");
}

static void task_lidar(void *a) {
    TFFrame f;
    for (;;) {
        // 每次读取超时50ms，超时后强制yield
        if (tfmini_read_frame(&f, 50)) {
            g_dist_cm  = f.dist_cm;
            g_lidar_ok = f.valid;
        } else {
            g_lidar_ok = false;
            // 没有数据时主动让出CPU，防止WDT
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        taskYIELD();  // 每帧结束都让出一次
    }
}

// ── 高度报警（纯阈值，实时判断） ─────────────────────────────
static volatile bool g_armed = false;

static void flare_update(float h, float vz) {
    if (!g_armed) { g_buzz_mode=BUZZ_OFF; return; }
    // 速度补偿：下落越快，报警越早（多留0.3秒反应时间）
    float d  = fabsf(vz);
    float t1 = g_cfg.stage1_h + d * 0.3f;
    float t2 = g_cfg.stage2_h + d * 0.3f;
    if (h <= t2) {
        g_buzz_mode = BUZZ_STAGE2;  // 连续长响
    } else if (h <= t1) {
        g_buzz_mode = BUZZ_STAGE1;  // 间歇滴滴
    } else {
        g_buzz_mode = BUZZ_OFF;     // 高于S1不响
    }
}
}

// ── 共享数据 ─────────────────────────────────────────────────
static volatile float g_kf_h=0, g_kf_vz=0, g_pitch=0, g_roll=0, g_agl=0;
static volatile bool  g_att_conv=false;

// ── ARMED主任务 ───────────────────────────────────────────────
static void task_armed(void *a) {
    uint32_t cnt = 0;
    for (;;) {
        float ax,ay,az,gx,gy,gz;
        imu_read_raw(&ax,&ay,&az,&gx,&gy,&gz);
        m_update(&g_mahony, gx,gy,gz, ax,ay,az);
        g_pitch    = g_mahony.pitch;
        g_roll     = g_mahony.roll;
        g_att_conv = g_mahony.conv;

        float hoff = g_cfg.h_offset_cm / 100.0f;
        float ct   = g_mahony.R[2][2];
        if (ct < 0.5f) ct = 0.5f;
        float agl  = g_dist_cm/100.0f * ct - hoff;
        if (agl < 0) agl = 0;
        g_agl = agl;

        float adown = -(g_mahony.R[2][0]*ax + g_mahony.R[2][1]*ay
                      + g_mahony.R[2][2]*az - 1.0f) * 9.81f;
        if (!g_kf.inited && g_lidar_ok && agl>0) kf_init(&g_kf, agl);
        if (g_kf.inited) {
            kf_predict(&g_kf, adown);
            if (g_lidar_ok && agl>0) kf_update(&g_kf, agl);
            g_kf_h  = g_kf.h;
            g_kf_vz = g_kf.vz;
        }

        flare_update(g_kf_h, g_kf_vz);

        if (++cnt % 10 == 0)
            ESP_LOGI(TAG,"P=%.1f R=%.1f h=%.2f vz=%.2f conv=%d",
                     g_mahony.pitch, g_mahony.roll,
                     (float)g_kf_h, (float)g_kf_vz, g_mahony.conv);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ── 水墨屏 ───────────────────────────────────────────────────
static spi_device_handle_t epd_spi;
static uint8_t epd_buf[EPD_BYTES];

#define EC()  gpio_set_level(PIN_EPD_DC,  0)
#define ED()  gpio_set_level(PIN_EPD_DC,  1)
#define EL()  gpio_set_level(PIN_EPD_CS,  0)
#define EH()  gpio_set_level(PIN_EPD_CS,  1)

static void epd_wait_busy(void) {
    int t=10000;
    while (gpio_get_level(PIN_EPD_BUSY)==1) {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (--t==0) return;
    }
}
static void epd_cmd(uint8_t c) {
    spi_transaction_t t={.length=8,.tx_buffer=&c};
    EC(); EL(); spi_device_polling_transmit(epd_spi,&t); EH();
}
static void epd_dat(uint8_t d) {
    spi_transaction_t t={.length=8,.tx_buffer=&d};
    ED(); EL(); spi_device_polling_transmit(epd_spi,&t); EH();
}
static void epd_cur(void) {
    epd_cmd(0x4E); epd_dat(0x00);
    epd_cmd(0x4F); epd_dat(0x00); epd_dat(0x00);
}
static void epd_init_seq(void) {
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_EPD_RST,0); vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_EPD_RST,1); vTaskDelay(pdMS_TO_TICKS(10));
    epd_wait_busy();
    epd_cmd(0x12); epd_wait_busy();
    epd_cmd(0x01); epd_dat(0x97); epd_dat(0x00); epd_dat(0x00);
    epd_cmd(0x11); epd_dat(0x03);
    epd_cmd(0x44); epd_dat(0x00); epd_dat(0x12);
    epd_cmd(0x45); epd_dat(0x00); epd_dat(0x00); epd_dat(0x97); epd_dat(0x00);
    epd_cur();
}
static void epd_flush(void) {
    epd_cmd(0x3C); epd_dat(0x05);
    epd_cur(); epd_cmd(0x24);
    ED(); EL();
    for (int i=0; i<EPD_BYTES; i++) {
        spi_transaction_t t={.length=8,.tx_buffer=&epd_buf[i]};
        spi_device_polling_transmit(epd_spi,&t);
    }
    EH();
    epd_cur(); epd_cmd(0x26);
    ED(); EL();
    for (int i=0; i<EPD_BYTES; i++) {
        uint8_t z=0;
        spi_transaction_t t={.length=8,.tx_buffer=&z};
        spi_device_polling_transmit(epd_spi,&t);
    }
    EH();
    epd_cmd(0x22); epd_dat(0xF4);
    epd_cmd(0x20); epd_wait_busy();
}
static void epd_sleep_mode(void) {
    epd_cmd(0x10); epd_dat(0x01);
    vTaskDelay(pdMS_TO_TICKS(200)); EH();
}
static void epd_show(void) { epd_init_seq(); epd_flush(); epd_sleep_mode(); }

// ── UI ───────────────────────────────────────────────────────
static const uint8_t F6[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00,0x00},{0x14,0x7F,0x14,0x7F,0x14,0x00},
    {0x24,0x2A,0x7F,0x2A,0x12,0x00},{0x23,0x13,0x08,0x64,0x62,0x00},
    {0x36,0x49,0x55,0x22,0x50,0x00},{0x00,0x05,0x03,0x00,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00,0x00},{0x00,0x41,0x22,0x1C,0x00,0x00},
    {0x08,0x2A,0x1C,0x2A,0x08,0x00},{0x08,0x08,0x3E,0x08,0x08,0x00},
    {0x00,0x50,0x30,0x00,0x00,0x00},{0x08,0x08,0x08,0x08,0x08,0x00},
    {0x00,0x60,0x60,0x00,0x00,0x00},{0x20,0x10,0x08,0x04,0x02,0x00},
    {0x3E,0x51,0x49,0x45,0x3E,0x00},{0x00,0x42,0x7F,0x40,0x00,0x00},
    {0x42,0x61,0x51,0x49,0x46,0x00},{0x21,0x41,0x45,0x4B,0x31,0x00},
    {0x18,0x14,0x12,0x7F,0x10,0x00},{0x27,0x45,0x45,0x45,0x39,0x00},
    {0x3C,0x4A,0x49,0x49,0x30,0x00},{0x01,0x71,0x09,0x05,0x03,0x00},
    {0x36,0x49,0x49,0x49,0x36,0x00},{0x06,0x49,0x49,0x29,0x1E,0x00},
    {0x00,0x36,0x36,0x00,0x00,0x00},{0x00,0x56,0x36,0x00,0x00,0x00},
    {0x00,0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14,0x00},
    {0x41,0x22,0x14,0x08,0x00,0x00},{0x02,0x01,0x51,0x09,0x06,0x00},
    {0x32,0x49,0x79,0x41,0x3E,0x00},{0x7E,0x11,0x11,0x11,0x7E,0x00},
    {0x7F,0x49,0x49,0x49,0x36,0x00},{0x3E,0x41,0x41,0x41,0x22,0x00},
    {0x7F,0x41,0x41,0x22,0x1C,0x00},{0x7F,0x49,0x49,0x49,0x41,0x00},
    {0x7F,0x09,0x09,0x01,0x01,0x00},{0x3E,0x41,0x41,0x51,0x32,0x00},
    {0x7F,0x08,0x08,0x08,0x7F,0x00},{0x00,0x41,0x7F,0x41,0x00,0x00},
    {0x20,0x40,0x41,0x3F,0x01,0x00},{0x7F,0x08,0x14,0x22,0x41,0x00},
    {0x7F,0x40,0x40,0x40,0x40,0x00},{0x7F,0x02,0x04,0x02,0x7F,0x00},
    {0x7F,0x04,0x08,0x10,0x7F,0x00},{0x3E,0x41,0x41,0x41,0x3E,0x00},
    {0x7F,0x09,0x09,0x09,0x06,0x00},{0x3E,0x41,0x51,0x21,0x5E,0x00},
    {0x7F,0x09,0x19,0x29,0x46,0x00},{0x26,0x49,0x49,0x49,0x32,0x00},
    {0x01,0x01,0x7F,0x01,0x01,0x00},{0x3F,0x40,0x40,0x40,0x3F,0x00},
    {0x1F,0x20,0x40,0x20,0x1F,0x00},{0x3F,0x40,0x38,0x40,0x3F,0x00},
    {0x63,0x14,0x08,0x14,0x63,0x00},{0x03,0x04,0x78,0x04,0x03,0x00},
    {0x61,0x51,0x49,0x45,0x43,0x00}
};

static void up(int x, int y, uint8_t b) {
    x = EPD_W-1-x;  // 修复水平镜像
    y = EPD_H-1-y;
    if (x<0||x>=EPD_W||y<0||y>=EPD_H) return;
    int idx=(y*EPD_W+x)/8, bit=7-(x%8);
    if (b) epd_buf[idx] &= ~(1<<bit);
    else   epd_buf[idx] |=  (1<<bit);
}
static void uc(int x, int y, char c) {
    if (c<0x20||c>0x5A) return;
    const uint8_t *g = F6[c-0x20];
    for (int col=0; col<6; col++) {
        uint8_t bits = g[col];
        for (int row=0; row<8; row++)
            up(x+col, y+row, (bits>>row)&1);
    }
}
static void us(int x, int y, const char *s) {
    while (*s) { uc(x,y,*s++); x+=6; }
}
static void ul(int x0, int x1, int y) {
    for (int x=x0; x<=x1; x++) up(x,y,1);
}

static void scr_standby(void) {
    memset(epd_buf, 0xFF, EPD_BYTES);
    us(4,4,"PLA V1"); ul(0,151,16);
    char b[32];
    snprintf(b,sizeof(b),"WING LOAD: %.2f",g_cfg.wing_loading);
    us(4,24,b);
    snprintf(b,sizeof(b),"H OFFSET: %dcm",g_cfg.h_offset_cm);
    us(4,40,b);
    snprintf(b,sizeof(b),"STAGE1: %.1fM",g_cfg.stage1_h);
    us(4,56,b);
    snprintf(b,sizeof(b),"STAGE2: %.1fM",g_cfg.stage2_h);
    us(4,72,b);
    ul(0,151,88);
    us(4,96,"B0 6S: WIFI CFG");
    us(4,112,"D8 GND: ARM");
    snprintf(b,sizeof(b),"BOOT: %lu",(unsigned long)rtc_boot_cnt);
    us(4,128,b);
}
#define WSSID "PLA-Config"
#define WPASS "QQ123456"
static void scr_wifi(void) {
    memset(epd_buf, 0xFF, EPD_BYTES);
    us(4,4,"WIFI CONFIG"); ul(0,151,16);
    us(4,26,"SSID:");
    us(4,40,WSSID);
    us(4,56,"PASS:");
    us(4,70,WPASS);
    ul(0,151,84);
    us(4,92,"192.168.4.1");
    ul(0,151,108);
    us(4,118,"SAVE TO EXIT");
}
static void scr_config(void) {
    memset(epd_buf, 0xFF, EPD_BYTES);
    us(4,4,"CONFIG SAVED"); ul(0,151,16);
    char b[32];
    snprintf(b,sizeof(b),"WING LOAD: %.2f",g_cfg.wing_loading);
    us(4,28,b);
    snprintf(b,sizeof(b),"H OFFSET: %dcm",g_cfg.h_offset_cm);
    us(4,44,b);
    snprintf(b,sizeof(b),"STAGE1: %.1fM",g_cfg.stage1_h);
    us(4,60,b);
    snprintf(b,sizeof(b),"STAGE2: %.1fM",g_cfg.stage2_h);
    us(4,76,b);
    ul(0,151,92);
    us(4,104,"WIFI CLOSED");
    us(4,120,"D8 GND: ARM");
}

// ── 水墨屏任务（已删除，不再持续刷新） ────────────────────────
static volatile bool g_cfg_saved = false;

// ── WiFi配置门户 ─────────────────────────────────────────────
static httpd_handle_t g_httpd    = NULL;
static bool           g_wifi_on  = false;
static bool           g_net_init = false;

static float uf(const char *b, const char *k, float d) {
    char s[32]; snprintf(s,sizeof(s),"%s=",k);
    const char *p = strstr(b,s);
    return p ? strtof(p+strlen(s),NULL) : d;
}
static int ui2(const char *b, const char *k, int d) {
    char s[32]; snprintf(s,sizeof(s),"%s=",k);
    const char *p = strstr(b,s);
    return p ? (int)strtol(p+strlen(s),NULL,10) : d;
}

static const char PH[] =
"<!DOCTYPE html><html><head>"
"<meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>PLA</title>"
"<style>"
"body{font-family:monospace;max-width:400px;margin:20px auto;padding:0 16px;"
"background:#111;color:#0f0;}"
"h1{color:#0ff;border-bottom:1px solid #0f0;padding-bottom:8px;}"
"label{display:block;margin:12px 0 4px;color:#aaa;font-size:.9em;}"
"input{width:100%%;box-sizing:border-box;background:#222;border:1px solid #0f0;"
"color:#0f0;padding:8px;font-size:1em;border-radius:4px;}"
"button{margin-top:20px;width:100%%;padding:12px;background:#0a0;border:none;"
"color:#fff;font-size:1.1em;border-radius:4px;}"
".ok{color:#0f0;margin-top:12px;display:none;}"
"</style></head><body>"
"<h1>PLA v1</h1>"
"<form id='f'>"
"<label>Wing Loading (lbs/ft2)</label>"
"<input type='number' name='wl' step='0.01' min='0.5' max='2.0' value='%.2f'>"
"<label>Height Offset cm</label>"
"<input type='number' name='hoff' step='1' min='30' max='150' value='%d'>"
"<label>Stage1 altitude (m)</label>"
"<input type='number' name='h1' step='0.1' min='2.0' max='8.0' value='%.1f'>"
"<label>Stage2 altitude (m)</label>"
"<input type='number' name='h2' step='0.1' min='0.5' max='3.0' value='%.1f'>"
"<button type='submit'>SAVE</button>"
"</form>"
"<p class='ok' id='ok'>Saved!</p>"
"<script>"
"document.getElementById('f').onsubmit=function(e){"
"e.preventDefault();"
"fetch('/save',{method:'POST',body:new URLSearchParams(new FormData(this))})"
".then(()=>{document.getElementById('ok').style.display='block';});"
"};"
"</script>"
"</body></html>";

static esp_err_t h_root(httpd_req_t *req) {
    char html[4096];
    snprintf(html, sizeof(html), PH,
             g_cfg.wing_loading, (int)g_cfg.h_offset_cm,
             g_cfg.stage1_h, g_cfg.stage2_h);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}
static esp_err_t h_save(httpd_req_t *req) {
    char body[256] = {0};
    int len = httpd_req_recv(req, body, 255);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "");
        return ESP_FAIL;
    }
    body[len] = 0;
    g_cfg.wing_loading = uf(body,"wl",  g_cfg.wing_loading);
    g_cfg.h_offset_cm  = ui2(body,"hoff",g_cfg.h_offset_cm);
    g_cfg.stage1_h     = uf(body,"h1",  g_cfg.stage1_h);
    g_cfg.stage2_h     = uf(body,"h2",  g_cfg.stage2_h);
    if (g_cfg.wing_loading<0.5f||g_cfg.wing_loading>2.0f) g_cfg.wing_loading=0.93f;
    if (g_cfg.h_offset_cm<30||g_cfg.h_offset_cm>150)      g_cfg.h_offset_cm=85;
    if (g_cfg.stage1_h<2||g_cfg.stage1_h>8)               g_cfg.stage1_h=4.0f;
    if (g_cfg.stage2_h<0.5f||g_cfg.stage2_h>3)            g_cfg.stage2_h=1.2f;
    if (g_cfg.stage2_h>=g_cfg.stage1_h)                   g_cfg.stage2_h=g_cfg.stage1_h*0.4f;
    nvs_save_cfg();
    g_cfg_saved = true;
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static void wifi_start(void) {
    gpio_set_level(PIN_BUZZER, 0);
    if (g_wifi_on) return;
    if (!g_net_init) {
        esp_netif_init();
        esp_event_loop_create_default();
        esp_netif_create_default_wifi_ap();
        wifi_init_config_t wic = WIFI_INIT_CONFIG_DEFAULT();
        esp_wifi_init(&wic);
        wifi_config_t wc = {};
        memcpy(wc.ap.ssid, WSSID, strlen(WSSID));
        wc.ap.ssid_len     = strlen(WSSID);
        memcpy(wc.ap.password, WPASS, strlen(WPASS));
        wc.ap.channel      = 6;
        wc.ap.max_connection = 2;
        wc.ap.authmode     = WIFI_AUTH_WPA2_PSK;
        esp_wifi_set_mode(WIFI_MODE_AP);
        esp_wifi_set_config(WIFI_IF_AP, &wc);
        g_net_init = true;
    }
    esp_wifi_start();
    vTaskDelay(pdMS_TO_TICKS(500));
    httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
    hc.lru_purge_enable = true;
    hc.stack_size = 8192;
    if (httpd_start(&g_httpd, &hc) == ESP_OK) {
        httpd_uri_t r = {.uri="/",.method=HTTP_GET,.handler=h_root};
        httpd_uri_t s = {.uri="/save",.method=HTTP_POST,.handler=h_save};
        httpd_register_uri_handler(g_httpd, &r);
        httpd_register_uri_handler(g_httpd, &s);
        g_wifi_on = true;
        ESP_LOGI(TAG,"WiFi: %s  192.168.4.1", WSSID);
    }
}
static void wifi_stop(void) {
    if (!g_wifi_on) return;
    if (g_httpd) { httpd_stop(g_httpd); g_httpd=NULL; }
    esp_wifi_stop();
    g_wifi_on = false;
}

// ── WiFi长按任务 ─────────────────────────────────────────────
static void task_wifi_btn(void *a) {
    gpio_config_t io = {
        .pin_bit_mask=(1ULL<<PIN_WIFI_BTN),
        .mode=GPIO_MODE_INPUT,
        .pull_up_en=GPIO_PULLUP_ENABLE
    };
    gpio_config(&io);
    for (;;) {
        // 检测配置是否已保存 → 关WiFi + 刷新屏幕显示新参数
        if (g_cfg_saved) {
            g_cfg_saved = false;
            vTaskDelay(pdMS_TO_TICKS(500));  // 让HTTP响应发出
            wifi_stop();
            scr_config(); epd_show();
            buzz_beep(200);
            ESP_LOGI(TAG,"Config saved, WiFi closed");
        }
        if (gpio_get_level(PIN_WIFI_BTN) == 0) {
            int h = 0;
            while (gpio_get_level(PIN_WIFI_BTN)==0 && h<600) {
                vTaskDelay(pdMS_TO_TICKS(10)); h++;
            }
            if (h >= 600) {
                ESP_LOGI(TAG,"WiFi BTN long press");
                buzz_beep(200);
                wifi_start();
                scr_wifi(); epd_show();  // 显示WiFi SSID和IP
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ── ARM开关任务 ───────────────────────────────────────────────
static void task_arm(void *a) {
    gpio_config_t io = {
        .pin_bit_mask=(1ULL<<PIN_ARM),
        .mode=GPIO_MODE_INPUT,
        .pull_up_en=GPIO_PULLUP_ENABLE
    };
    gpio_config(&io);
    bool last = false;
    for (;;) {
        bool armed = (gpio_get_level(PIN_ARM) == 0);
        if (armed != last) {
            vTaskDelay(pdMS_TO_TICKS(50));
            armed = (gpio_get_level(PIN_ARM) == 0);
            if (armed && !last) {
                ESP_LOGI(TAG,"ARM ON - waking sensors");
                wifi_stop();
                // IMU唤醒
                imu_wakeup();
                // TFmini唤醒（如果已初始化则唤醒，否则初始化）
                if (!g_lidar_initialized) {
                    tfmini_init();
                    g_lidar_initialized = true;
                } else {
                    tfmini_wake();
                }
                vTaskDelay(pdMS_TO_TICKS(500));
                bool ok = false;
                for (int i=0; i<1500; i++) {
                    if (g_mahony.conv && g_lidar_ok) { ok=true; break; }
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                if (ok) {
                    g_armed=true; g_buzz_mode=BUZZ_OFF;
                    buzz_beep(150); vTaskDelay(pdMS_TO_TICKS(100)); buzz_beep(150);
                    ESP_LOGI(TAG,"ARMED!");
                } else {
                    g_armed = false;
                    ESP_LOGW(TAG,"ARM FAIL silent");
                }
            } else if (!armed && last) {
                g_armed=false; g_buzz_mode=BUZZ_OFF;
                // 传感器进入低功耗
                imu_sleep_mode();
                tfmini_sleep();
                ESP_LOGI(TAG,"DISARMED - sensors sleeping");
            }
            last = armed;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ════════════════════════════════════════════════════════════════
//  app_main
// ════════════════════════════════════════════════════════════════
void app_main(void) {
    buzz_init();
    rtc_boot_cnt++;
    ESP_LOGI(TAG,"=== PLA Final boot #%lu ===", (unsigned long)rtc_boot_cnt);
    nvs_load();

    // SPI2：IMU
    spi_bus_config_t ib = {
        .mosi_io_num=PIN_IMU_MOSI, .miso_io_num=PIN_IMU_MISO,
        .sclk_io_num=PIN_IMU_SCK,  .quadwp_io_num=-1, .quadhd_io_num=-1,
        .max_transfer_sz=64
    };
    spi_bus_initialize(SPI2_HOST, &ib, SPI_DMA_CH_AUTO);
    spi_device_interface_config_t id = {
        .clock_speed_hz=8*1000*1000, .mode=0,
        .spics_io_num=PIN_IMU_CS, .queue_size=1
    };
    spi_bus_add_device(SPI2_HOST, &id, &imu_spi);

    // SPI3：水墨屏
    gpio_config_t eo = {
        .pin_bit_mask=(1ULL<<PIN_EPD_CS)|(1ULL<<PIN_EPD_DC)|(1ULL<<PIN_EPD_RST),
        .mode=GPIO_MODE_OUTPUT
    };
    gpio_config(&eo);
    gpio_config_t ei = {
        .pin_bit_mask=(1ULL<<PIN_EPD_BUSY),
        .mode=GPIO_MODE_INPUT, .pull_up_en=GPIO_PULLUP_ENABLE
    };
    gpio_config(&ei);
    EH(); gpio_set_level(PIN_EPD_RST, 1);
    spi_bus_config_t eb = {
        .mosi_io_num=PIN_EPD_MOSI, .miso_io_num=-1,
        .sclk_io_num=PIN_EPD_SCK,  .quadwp_io_num=-1, .quadhd_io_num=-1,
        .max_transfer_sz=EPD_BYTES+16
    };
    spi_bus_initialize(SPI3_HOST, &eb, SPI_DMA_CH_AUTO);
    spi_device_interface_config_t ed = {
        .clock_speed_hz=2*1000*1000, .mode=0,
        .spics_io_num=-1, .queue_size=1
    };
    spi_bus_add_device(SPI3_HOST, &ed, &epd_spi);

    // ARM引脚
    gpio_config_t ai = {
        .pin_bit_mask=(1ULL<<PIN_ARM),
        .mode=GPIO_MODE_INPUT, .pull_up_en=GPIO_PULLUP_ENABLE
    };
    gpio_config(&ai);

    // IMU初始化
    bool imu_ok = imu_init();
    if (!imu_ok) { ESP_LOGE(TAG,"IMU FAIL"); buzz_beep(1000); }

    // 冷启动 or 意外断电重启
    ESP_LOGI(TAG,"COLD START DETECTED");

    bool arm_on = (gpio_get_level(PIN_ARM) == 0);

    if (arm_on) {
        // 空中意外断电重启：ARM已开，跳过校准直接武装
        ESP_LOGE(TAG,"CRASH REBOOT IN AIR! Skip calibration!");
        buzz_beep(50); vTaskDelay(pdMS_TO_TICKS(50));
        buzz_beep(50); vTaskDelay(pdMS_TO_TICKS(50));
        buzz_beep(50);
        m_init_fresh(&g_mahony);
        g_armed = true;
        tfmini_init();
        g_lidar_initialized = true;

        } else {
            // 正常地面冷启动
            ESP_LOGI(TAG,"GROUND PREP - Normal Cold Start");
            buzz_beep(200);  // 自检通过：1声短鸣
            m_init_fresh(&g_mahony);
            calibrate_gyro(&g_mahony);  // 10秒校准

            // 保存零偏（直接保存在g_mahony，不需要RTC）
            rtc_calib.q0=g_mahony.q0; rtc_calib.q1=g_mahony.q1;
            rtc_calib.q2=g_mahony.q2; rtc_calib.q3=g_mahony.q3;
            rtc_calib.gx_bias=g_mahony.gx_b;
            rtc_calib.gy_bias=g_mahony.gy_b;
            rtc_calib.gz_bias=g_mahony.gz_b;
            rtc_calib.valid=true;
            ESP_LOGI(TAG,"Calibration done");

            buzz_beep(800);  // 校准完成：1声长鸣
            scr_standby(); epd_show();

            // ESP32不休眠，让IMU和TFmini进入低功耗等待ARM
            ESP_LOGI(TAG,"Entering low-power wait (IMU+TFmini sleep)");
            imu_sleep_mode();
            tfmini_init();              // 先装UART驱动
            g_lidar_initialized = true;
            tfmini_sleep();

            ESP_LOGI(TAG,"Waiting for ARM switch...");
        }

    // 启动运行任务
    xTaskCreatePinnedToCore(task_buzzer,  "buzz",  2048, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(task_lidar,   "lidar", 4096, NULL, 4, NULL, 1);  // Core1，避免阻塞Core0 IDLE
    xTaskCreatePinnedToCore(task_armed,   "armed", 8192, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(task_wifi_btn,"wbtn",  6144, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(task_arm,     "arm",   3072, NULL, 3, NULL, 1);
}
