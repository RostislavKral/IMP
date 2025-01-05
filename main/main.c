/**
* @author Rostislav Kral (xkralr06)
* @date 2024-12-01
* @file main.c 
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ssd1306.h"
#include "driver/i2c.h"


#define I2C_MASTER_NUM            I2C_NUM_0
#define I2C_MASTER_SCL_IO         32
#define I2C_MASTER_SDA_IO         33
#define MAX30102_ADDRESS          0x57

// Registers of the sensor MAX30102
#define REG_INTR_ENABLE_1 0x02
#define REG_FIFO_DATA 0x07
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D



#define FSS 25 // Sampling rate
#define NUMBER_OF_SAMPLES 128 // Batch


static const char *TAG = "MAX30102";



/**
* @brief Custom i2c initialization function
* @return 
*/
void i2c_master_initt() {
    ESP_LOGI(TAG, "Initializing I2C");
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 200000,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, config.mode, 0, 0, 0));
}

/**
* @brief Function for writing into sensor's registers
* @param reg for register's name
* @param value to be set to the register
* @return esp_err_t
*/
esp_err_t max30102_write_register(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
* @brief Function for reading FIFO(First in, first out) communication
* @return esp_err_t
*/
esp_err_t max30102_read_fifo(uint32_t *ir, uint32_t *red) {
    uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, REG_FIFO_DATA, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *red = (data[0] << 16) | (data[1] << 8) | data[2];
        *ir = (data[3] << 16) | (data[4] << 8) | data[5];
    }
    return ret;
}

/**
* @brief Sensor initialization function
* @return
*/
void max30102_init() {


   	max30102_write_register(REG_INTR_ENABLE_1, 0xC0); // Enable interrupts
    max30102_write_register(REG_MODE_CONFIG, 0x03);   // Heart rate mode
    max30102_write_register(REG_SPO2_CONFIG, 0x23);   // 25Hz sampling, 18-bit resolution
    max30102_write_register(REG_LED1_PA, 0x24);       // LED1 current
    max30102_write_register(REG_LED2_PA, 0x24);       // LED2 current


}

bool in_array(int *arr, int needle)
{
    for(int i=0; i<NUMBER_OF_SAMPLES; i++){
        //printf("%d: %d(%d)\n",i,arr[i],needle);
        if(arr[i] == needle) return true;
    }
    return false;
}


/**
* @brief Function realizing lowpass filter
* @param input input signal
* @param output filtered ouput signal
* @param alpha smoothing constant
* @return
*/
void lowpass_filter(long unsigned *input, long int *filtered_data, float alpha) {
    filtered_data[0] = input[0];  

    for (size_t i = 1; i < NUMBER_OF_SAMPLES; i++) {
        filtered_data[i] = (alpha * input[i]) + ((1 - alpha) * filtered_data[i - 1]);
    }
}

/**
* @brief Function for computing autocorrelation coefficients
* @param input signal
* @param output signal
* @return
*/
void compute_autocorrelation(const int32_t *signal, int32_t *output) {
    for (int lag = 0; lag < NUMBER_OF_SAMPLES; lag++) {
        long long int sum = 0;
        for (int i = 0; i < NUMBER_OF_SAMPLES - lag; i++) {
            sum += signal[i] * signal[i + lag];
        }
       output[lag] = sum;
    }
}


/**
* @brief Helper function for getting the max of array
* @param arr array of integers
* @return max value of the arr
*/
long long int get_max(const int32_t *arr)
{
    long long int max = arr[0];
    for(int i=0; i < NUMBER_OF_SAMPLES; i++)
    {
        if(arr[i] > max) max = arr[i];
    }

    return max;
}


/**
* @brief Function for finding peaks in autocorrelated signal
* @param autocorr array of autocorrelated coefficients
* @param peaks array of measured peak indicies used as output
* @return number of peaks
*/
int find_peaks(const int32_t *autocorr, int *peaks) {
    int peak_count = 0;

    long long int max = get_max(autocorr);

    double threshold = max * 0.3;

    for (int i = 1; i < NUMBER_OF_SAMPLES - 1; i++) {
        if (autocorr[i] > autocorr[i - 1] && autocorr[i] > autocorr[i + 1] && autocorr[i] > threshold) {
                peaks[peak_count] = i;
                peak_count++;
        }
    }

    return peak_count;
}


/**
* @brief Function will take array of indices and calculate final heart rate based on averaging intervals
* @param peaks array of on which index in singal peak had occured
* @param n_of_peaks number of peaks
* @return double heartrate
*/
double calculate_heart_rate(const int *peaks, int n_of_peaks) {

        double T = (double)peaks[0] / FSS; 
        printf("PERIOD: %f", T);
        if(T == 0) return 0;

        double bpm = 60.0f / T;

        return bpm;
}

/**
* @brief Filter for smoothing or shifting signal values by mean
* @param input signal
* @param output filtered signal
*/
void mean_filter(unsigned long *signal, long int *filtered_data)
{
    long long int acc = 0; //accumulator
    for(int i=0; i < NUMBER_OF_SAMPLES; i++){
        acc += signal[i]; 
    }

    long int mean = acc/NUMBER_OF_SAMPLES;

    for(int i=0; i<NUMBER_OF_SAMPLES;i++){
        filtered_data[i] = signal[i] - mean;
    }

}

void app_main(void)
{
	SSD1306_t dev;
	char buffer[20];

    i2c_master_initt();
    max30102_init();
#if CONFIG_SPI_INTERFACE
	ESP_LOGI(TAG, "INTERFACE is SPI");
	ESP_LOGI(TAG, "CONFIG_MOSI_GPIO=%d",CONFIG_MOSI_GPIO);
	ESP_LOGI(TAG, "CONFIG_SCLK_GPIO=%d",CONFIG_SCLK_GPIO);
	ESP_LOGI(TAG, "CONFIG_CS_GPIO=%d",CONFIG_CS_GPIO);
	ESP_LOGI(TAG, "CONFIG_DC_GPIO=%d",CONFIG_DC_GPIO);
	ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
	spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO);
#endif // CONFIG_SPI_INTERFACE


#if CONFIG_SSD1306_128x64
	ESP_LOGI(TAG, "Panel is 128x64");
	ssd1306_init(&dev, 128, 64);
#endif // CONFIG_SSD1306_128x64
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text_x3(&dev, 0, "HELLO", 5, false);
for(;;) {

    long unsigned int input[NUMBER_OF_SAMPLES];
    int32_t filtered_data[NUMBER_OF_SAMPLES];
    long int autocorr_output[NUMBER_OF_SAMPLES];
    int peaks[NUMBER_OF_SAMPLES];
    

    // Just to be sure
    memset(peaks, 0, NUMBER_OF_SAMPLES*sizeof(int));
	memset(filtered_data,0, NUMBER_OF_SAMPLES*sizeof(int));
    memset(autocorr_output,0, NUMBER_OF_SAMPLES*sizeof(int));
    memset(input, 0, NUMBER_OF_SAMPLES*sizeof(unsigned int));
    
    printf("MEASURED FROM SENSOR: ");
    for (size_t i = 0; i < NUMBER_OF_SAMPLES; i++) {
            long unsigned int ir, red;
            if (max30102_read_fifo(&ir, &red) == ESP_OK) {
                input[i] = ir;
               printf("%ld,", ir);
            }
            vTaskDelay(pdMS_TO_TICKS(40)); // 25Hz sampling
    }
        printf("\n------------------------------------------------------------\n");

        lowpass_filter(input,filtered_data, 0.3); //Just in case, the last param is for adjusting the filter's smoothness 
   		mean_filter(input,filtered_data); 


            printf("FILTERED DATA(MEAN + LOWPASS): ");
			
    for (size_t i = 0; i < NUMBER_OF_SAMPLES; i++) {
          printf("%ld,", filtered_data[i]);
        }
        printf("\n------------------------------------------------------------\n");



    compute_autocorrelation(filtered_data, autocorr_output);


    printf("AUTOCORRELATION RESULTS: ");

    for(int i=0; i<NUMBER_OF_SAMPLES; i++)
        {
            printf("%ld,", autocorr_output[i]);
        }
    
    printf("\n------------------------------------------------------------\n");


    int n_of_peaks = find_peaks(autocorr_output, peaks);

    printf("NON-ZERO PEAKS: ");
    for(int i=0; i<NUMBER_OF_SAMPLES; i++)
        {
            if(peaks[i] >0 && peaks[i] <= 127)
                printf("PEAK[%d]: %d\n", i, peaks[i]);
        }
    printf("\n------------------------------------------------------------\n\n\n");

    int heart_rate = calculate_heart_rate(peaks, n_of_peaks);
    if(heart_rate >= 40 && heart_rate <= 135){
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	sprintf(buffer, "%dBPM", heart_rate);
	ssd1306_display_text_x3(&dev, 0, buffer, 5, false);
    }
    printf("BPM: %d\n", heart_rate);
    printf("\n***********************************************************************************************\n\n\n");

    }
}
