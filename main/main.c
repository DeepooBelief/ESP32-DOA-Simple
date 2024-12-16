#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_dsp.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_check.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

#define FFT_SIZE 1024   //Since we are using FFT4, the size must be a power of 4, like 256, 1024, 4096
#define SAMPLE_RATE 100000 // 采样率
#define mic_distance 0.1 // Distance between microphones

#define GPIO_I2S_LRCK (GPIO_NUM_20)
#define GPIO_I2S_MCLK (GPIO_NUM_NC)
#define GPIO_I2S_SCLK (GPIO_NUM_2)
#define GPIO_I2S_SDIN (GPIO_NUM_21)
#define GPIO_I2S_DOUT (GPIO_NUM_NC)

#define INPUT_ARRAY_SIZE (FFT_SIZE << 1)
#define I2S_BUFF_SIZE (FFT_SIZE << 1)
#define RAD2DEG 57.295779513082320876798154814105

const float val = 343.0 / mic_distance;

__attribute__((aligned(16))) float x1[INPUT_ARRAY_SIZE];

__attribute__((aligned(16))) float x2[INPUT_ARRAY_SIZE];
static int32_t i2s_readraw_buff[I2S_BUFF_SIZE];
static i2s_chan_handle_t rx_chan; // I2S rx channel handler

void computeFFTandCrossPowerSpectrum(float *x1, float *x2, int N)
{
    // 对x1和x2进行FFT
    dsps_fft4r_fc32(x1, N >> 1);
    dsps_bit_rev4r_fc32(x1, N >> 1);
    dsps_fft4r_fc32(x2, N >> 1);
    dsps_bit_rev4r_fc32(x2, N >> 1);

    // 使用x2数组存储互功率谱的结果
    float *p1 = x1;
    float *p2 = x2;
    for (int i = 0; i < N; i += 2)
    {
        float real1 = *p1;
        float imag1 = *(p1 + 1);
        float real2 = *p2;
        float imag2 = *(p2 + 1);

        // G = x1_fft .* conj(x2_fft)
        *p2 = real1 * real2 + imag1 * imag2;       // 实部
        *(p2 + 1) = imag1 * real2 - real1 * imag2; // 虚部

        p1 += 2;
        p2 += 2;
    }
}

void applyWeightAndIFFT(float *G, float *temp, int N)
{
    // 加权后效果不好，暂时不使用
    // if (temp == NULL)
    // {
    //     float magnitude, weight;
    //     float *p = G;
    //     // 计算权重并应用
    //     for (int i = 0; i < N; i += 2)
    //     {
    //         magnitude = dsps_sqrtf_f32_ansi(*p * *p + *(p + 1) * *(p + 1));

    //         weight = 1.0 / magnitude;
    //         *p *= weight;
    //         *(p + 1) *= weight;
    //         p += 2;
    //     }
    // }
    // else
    // {
    //     dsps_mul_f32(G, G, temp, N, 1, 1, 1);
    //     dsps_add_f32(temp, temp + 1, temp, N >> 1, 2, 2, 2);
    //     for (int i = 0; i < N; i += 2)
    //     {
    //         temp[i] = dsps_sqrtf_f32(temp[i]);
    //     }
    //     dsps_mul_f32(G, temp, G, N >> 1, 2, 2, 2);
    //     dsps_mul_f32(G + 1, temp, G + 1, N >> 1, 2, 2, 2);
    // }

    // 进行IFFT
    // 参考：https://www.embedded.com/dsp-tricks-computing-inverse-ffts-using-the-forward-fft/
    for (int i = 1; i < N; i += 2)
    {
        G[i] = -G[i];
    }
    dsps_fft4r_fc32(G, N >> 1);
    dsps_bit_rev4r_fc32(G, N >> 1);
    dsps_mulc_f32(G, G, N, 1.0 / (N >> 1), 1, 1);
    for (int i = 1; i < N; i += 2)
    {
        G[i] = -G[i];
    }
}

float FindDelay(float *R, float *temp, int N, float fs)
{
    float maxVal = 0;
    int idx = 0;
    if (temp == NULL)
    {
        float *p = R;
        for (int i = 0; i < N; i += 2)
        {
            float absVal = *p * *p + *(p + 1) * *(p + 1);
            if (absVal > maxVal)
            {
                maxVal = absVal;
                idx = i >> 1;
            }
            p += 2;
        }
    }
    else
    {
        dsps_mul_f32(R, R, temp, N, 1, 1, 1);
        dsps_add_f32(temp, temp + 1, temp, N >> 1, 2, 2, 2);
        for (int i = 0; i < N; i += 2)
        {
            if (temp[i] > maxVal)
            {
                maxVal = temp[i];
                idx = i >> 1;
            }
        }
    }
    // 计算时延
    int sIndex = (idx >= (N >> 2)) ? (idx - (N >> 1)) : idx;
    return -sIndex / fs;
}

float angleEstimation(float *x1, float *x2, int N, float fs)
{
    // 计算互功率谱
    computeFFTandCrossPowerSpectrum(x1, x2, N);

    // 计算权重并应用
    applyWeightAndIFFT(x2, x1, N); // 这个使用临时数组更快，x1内容会被覆盖

    // 计算时延
    float delay = FindDelay(x2, NULL, N, fs); // 这个不使用临时数组更快, 不知道为什么
    return acos(delay * val) * RAD2DEG;
}

static void i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = GPIO_I2S_MCLK,
            .bclk = GPIO_I2S_SCLK,
            .ws = GPIO_I2S_LRCK,
            .dout = GPIO_I2S_DOUT,
            .din = GPIO_I2S_SDIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    /* Initialize the channels */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
}

static void DOA_task(void *args)
{
    /* Enable the RX channel */
    while (1)
    {
        /* Read i2s data */
        if (i2s_channel_read(rx_chan, i2s_readraw_buff, sizeof(i2s_readraw_buff), NULL, 1000) == ESP_OK)
        {
            for (int i = 0; i < I2S_BUFF_SIZE; i += 2)
            {
                x1[i] = i2s_readraw_buff[i];
                x1[i + 1] = 0.0;
                x2[i] = i2s_readraw_buff[i + 1];
                x2[i + 1] = 0.0;
            }
            float angle = angleEstimation(x1, x2, INPUT_ARRAY_SIZE, SAMPLE_RATE);
            printf("angle: %f\n", angle);
        }
        else
        {
            printf("Read Task: i2s read failed\n");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

void app_main()
{
    i2s_init();

    esp_err_t ret;
    ret = dsps_fft4r_init_fc32(NULL, FFT_SIZE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Not possible to initialize FFT4R. Error = %i", ret);
        return;
    }

    xTaskCreate(DOA_task, "DOA_task", 4096, NULL, 5, NULL);
}