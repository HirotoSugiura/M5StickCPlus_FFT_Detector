#include <driver/i2s.h>
#include "M5StickCPlus.h"
#include "fft.h"

#define SAMPLE_RATE 5000         // サンプリングレートの定義
#define WINDOW_SIZE 1024         // ウインドウサイズの定義（バッファ長制限に合わせて調整）
#define FFT_SIZE (WINDOW_SIZE / 2)
#define DISPLAY_HEIGHT 128       // ディスプレイの高さ（縦方向のピクセル数）

#define FREQ_THRESHOLD 500      // 周波数検出の閾値（調整が必要な場合あり）
#define FREQ_1_LOW 630           // 検出したい周波数帯1の下限
#define FREQ_1_HIGH 670          // 検出したい周波数帯1の上限
#define FREQ_2_LOW 480           // 検出したい周波数帯2の下限
#define FREQ_2_HIGH 560          // 検出したい周波数帯2の上限

SemaphoreHandle_t start_fft;
SemaphoreHandle_t start_dis;
int8_t i2s_readraw_buff[WINDOW_SIZE * 2];  // バッファサイズをウインドウサイズに合わせる
uint8_t fft_dis_buff[240][DISPLAY_HEIGHT] = {0};  // ウインドウサイズに合わせたディスプレイバッファ
uint16_t posData = 0;

TFT_eSprite Disbuff = TFT_eSprite(&M5.Lcd);

bool detectFrequency(float *fft_output, int fft_size, int freq_low, int freq_high) {
    int bin_low = freq_low * fft_size / SAMPLE_RATE;
    int bin_high = freq_high * fft_size / SAMPLE_RATE;
    float max_val = 0;
    
    for (int i = bin_low; i <= bin_high; i++) {
        float val = sqrt(fft_output[2 * i] * fft_output[2 * i] + fft_output[2 * i + 1] * fft_output[2 * i + 1]);
        if (val > max_val) {
            max_val = val;
        }
    }
    return max_val > FREQ_THRESHOLD;
}

void MicRecordfft(void *arg) {
    int16_t *buffptr;
    size_t bytesread;
    uint16_t count_n = 0;
    float adc_data;
    double data = 0;
    uint16_t ydata;

    while (1) {
        xSemaphoreTake(start_fft, portMAX_DELAY);
        fft_config_t *real_fft_plan = fft_init(WINDOW_SIZE, FFT_REAL, FFT_FORWARD, NULL, NULL);  // ウインドウサイズを使用
        i2s_read(I2S_NUM_0, (char *)i2s_readraw_buff, sizeof(i2s_readraw_buff), &bytesread, (100 / portTICK_RATE_MS));
        buffptr = (int16_t *)i2s_readraw_buff;

        for (count_n = 0; count_n < real_fft_plan->size; count_n++) {
            adc_data = (float)map(buffptr[count_n], INT16_MIN, INT16_MAX, -2000, 2000);
            real_fft_plan->input[count_n] = adc_data;
        }
        fft_execute(real_fft_plan);

        // 周波数検出
        bool detected_1 = detectFrequency(real_fft_plan->output, real_fft_plan->size, FREQ_1_LOW, FREQ_1_HIGH);
        bool detected_2 = detectFrequency(real_fft_plan->output, real_fft_plan->size, FREQ_2_LOW, FREQ_2_HIGH);

        if (detected_1) {
            digitalWrite(10,0);
            Serial.println("1 detect!");
        } else if (detected_2) {
            digitalWrite(10,0);
            Serial.println("2 detect!");
        } else {
            digitalWrite(10,1);
        }

        for (count_n = 1; count_n < FFT_SIZE / 2; count_n++) {
            data = sqrt(real_fft_plan->output[2 * count_n] * real_fft_plan->output[2 * count_n] +
                        real_fft_plan->output[2 * count_n + 1] * real_fft_plan->output[2 * count_n + 1]);
            if ((count_n - 1) < DISPLAY_HEIGHT) {
                data  = (data > 2000) ? 2000 : data;
                ydata = map(data, 0, 2000, 0, 255);
                fft_dis_buff[posData][DISPLAY_HEIGHT - count_n] = ydata;  // ディスプレイの高さに合わせて変更
            }
        }

        posData++;
        if (posData >= 240) {
            posData = 0;
        }
        fft_destroy(real_fft_plan);
        xSemaphoreGive(start_fft);
        vTaskDelay(10 / portTICK_RATE_MS);  // タスクウォッチドッグタイマーをリセット
    }
}

void Drawdisplay(void *arg) {
    uint16_t count_x = 0, count_y = 0;
    uint16_t colorPos;
    while (1) {
        xSemaphoreTake(start_dis, portMAX_DELAY);
        for (count_y = 0; count_y < DISPLAY_HEIGHT; count_y++) {  // ディスプレイの高さに合わせて変更
            for (count_x = 0; count_x < 240; count_x++) {
                if ((count_x + posData) >= 240) {
                    colorPos = fft_dis_buff[count_x + posData - 240][count_y];
                } else {
                    colorPos = fft_dis_buff[count_x + posData][count_y];
                }

                Disbuff.drawPixel(count_x, count_y, Disbuff.color565(colorPos, colorPos, colorPos));
            }
        }
        Disbuff.pushSprite(0, 0);
        xSemaphoreGive(start_dis);
        vTaskDelay(10 / portTICK_RATE_MS);  // タスクウォッチドッグタイマーをリセット
    }
}

#define PIN_CLK  0
#define PIN_DATA 34

bool InitI2SMicroPhone() {
    esp_err_t err = ESP_OK;
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,  // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 1, 0)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,  // Set the format of the communication.
#else
        .communication_format = I2S_COMM_FORMAT_I2S,
#endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count    = 2,
        .dma_buf_len      = 1024,  // バッファ長を1024に設定
    };

    i2s_pin_config_t pin_config;
#if (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0))
    pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
#endif
    pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
    pin_config.ws_io_num    = PIN_CLK;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num  = PIN_DATA;

    err += i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    err += i2s_set_pin(I2S_NUM_0, &pin_config);
    err += i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

    if (err != ESP_OK) {
        return false;
    } else {
        return true;
    }
}

void setup() {
    M5.begin();
    M5.Lcd.setRotation(1);
    M5.Axp.ScreenBreath(80);
    Disbuff.createSprite(240, 135);
    Disbuff.fillRect(0, 0, 240, 135, TFT_BLACK);
    Disbuff.pushSprite(0, 0);
    if (!InitI2SMicroPhone()) {
        Serial.println("I2S initialization failed!");
        while (1); // エラーの場合は無限ループ
    }
    start_fft = xSemaphoreCreateBinary();  // MutexからBinary Semaphoreに変更
    start_dis = xSemaphoreCreateBinary();  // MutexからBinary Semaphoreに変更
    xSemaphoreGive(start_fft);  // 初期状態でセマフォを与える
    xSemaphoreGive(start_dis);  // 初期状態でセマフォを与える
    xTaskCreate(MicRecordfft, "MicRecordfft", 1024 * 2, (void *)0, 5, NULL);
    xTaskCreate(Drawdisplay, "Drawdisplay", 1024 * 2, (void *)0, 4, NULL);

    pinMode(10, OUTPUT);
}

void loop() {
    // loop内でのセマフォ操作を削除
    delay(100);
}
