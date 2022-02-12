
#include "bt_hid_task.h"

#include "esp_hidd.h"
#include "esp_hid_gap.h"

#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "soc/rtc_periph.h"
#include "soc/sens_periph.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"


// BEGIN touch pad inputs

#define TOUCH_THRESH_NO_USE   (0)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)

#define TOUCH_IO_PIN_NB 6
static uint8_t TOUCH_IO_PINS[TOUCH_IO_PIN_NB] = {
    CONFIG_LEFT_CLICK_TOUCH_IO,
    CONFIG_RIGHT_CLICK_TOUCH_IO,
    CONFIG_LEFT_KEY_TOUCH_IO,
    CONFIG_RIGHT_KEY_TOUCH_IO,
    CONFIG_UP_KEY_TOUCH_IO,
    CONFIG_DOWN_KEY_TOUCH_IO
};

static bool s_pad_activated[TOUCH_PAD_MAX];
static uint32_t s_pad_init_val[TOUCH_PAD_MAX];

/*
  Read values sensed at all available touch pads.
  Use 2 / 3 of read value as the threshold
  to trigger interrupt when the pad is touched.
  Note: this routine demonstrates a simple way
  to configure activation threshold for the touch pads.
  Do not touch any pads when this routine
  is running (on application start).
 */
static void init_touch_pad_thresholds(void)
{
    uint16_t touch_value;
    for (int i = 0; i < TOUCH_IO_PIN_NB; i++) {
        //read filtered value
        touch_pad_read_filtered(TOUCH_IO_PINS[i], &touch_value);
        s_pad_init_val[TOUCH_IO_PINS[i]] = touch_value;
        //set interrupt threshold.
        ESP_ERROR_CHECK(touch_pad_set_thresh(TOUCH_IO_PINS[i], touch_value * 2 / 3));
    }
}
/*
  Handle an interrupt triggered when a pad is touched.
  Recognize what pad has been touched and save it in a table.
 */
static void rtc_intr(void *arg)
{
    uint32_t pad_intr = touch_pad_get_status();
    //clear interrupt
    touch_pad_clear_status();
    for (int i = 0; i < TOUCH_IO_PIN_NB; i++)
        if ((pad_intr >> TOUCH_IO_PINS[i]) & 0x01)
            s_pad_activated[TOUCH_IO_PINS[i]] = true;
}

static void init_touch_pad_inputs(void)
{
    // Initialize touch pad peripheral, it will start a timer to run a filter
    ESP_ERROR_CHECK(touch_pad_init());
    // If use interrupt trigger mode, should set touch sensor FSM mode at 'TOUCH_FSM_MODE_TIMER'.
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    // Set reference voltage for charging/discharging
    // For most usage scenarios, we recommend using the following combination:
    // the high reference valtage will be 2.7V - 1V = 1.7V, The low reference voltage will be 0.5V.
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);

    // Init touch pad IO
    for (int i = 0; i < TOUCH_IO_PIN_NB; i++) {
        //init RTC IO and mode for touch pad.
        touch_pad_config(TOUCH_IO_PINS[i], TOUCH_THRESH_NO_USE);
    }

    // Initialize and start a software filter to detect slight change of capacitance.
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
    // Init threshholds
    init_touch_pad_thresholds();

    // Register touch interrupt ISR
    touch_pad_isr_register(rtc_intr, NULL);
    
    // Enabling interupt 
    touch_pad_intr_enable();
}

// END touch pad inputs


// BEGIN adc inputs

#define ADC_DEFAULT_VREF    1100    //Use adc2_vref_to_gpio() to obtain a better estimate
#define ADC_SAMPLES_NB      64      //Multisampling

static const adc_channel_t precision_channel= ADC_CHANNEL_6;

static void init_adc_input(adc_channel_t channel)
{
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);

    // Characterize ADC
    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, ADC_DEFAULT_VREF, adc_chars);
}

static void init_adc_inputs(void)
{
    init_adc_input(precision_channel);
}


uint32_t get_adc_reading(adc_channel_t channel) {
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < ADC_SAMPLES_NB; i++)
        adc_reading += adc1_get_raw((adc1_channel_t)channel);

    return adc_reading / ADC_SAMPLES_NB;
}


// END adc inputs

#define INPUT_MOVE_DELTA_RANGE CONFIG_INPUT_MOVE_DELTA_MAX - CONFIG_INPUT_MOVE_DELTA_MIN
static void init_inputs(void)
{
    init_touch_pad_inputs();
    init_adc_inputs();
}



#define RESET_PAD_ACTIVATED for (int i = 0; i < TOUCH_IO_PIN_NB; i++) s_pad_activated[TOUCH_IO_PINS[i]] = false;
void bt_hid_task(void *pvParameters)
{
    // first thing done is initialising touch_pad
    init_inputs();
    ESP_LOGI("INPUTS", "Ready to take inputs");

    esp_hidd_dev_t *hid_dev = pvParameters;

    bool was_last_command_empty = true;

    while (1) {
        uint8_t buffer[4] = {0};

        /* buffer array description: see 
         *  Device Class Definition for Human Interface Devices (HID)
         *  Firmware Specification — 6/27/01
         *  Version 1.11 (appendix B.2 is a good starting point)
         *
         * buffer[0] is for clicks:
         *  first bit = 1 -> left click, second bit = 1 -> right click
         * buffer[1] is for horizontal cursor movement
         * buffer[2] is for vertical cursor movement
         * buffer[3] is for the wheel (unsupported yet)
         */
        uint32_t move_delta = (get_adc_reading(precision_channel) * INPUT_MOVE_DELTA_RANGE) / 4095 + CONFIG_INPUT_MOVE_DELTA_MIN;
        if (s_pad_activated[CONFIG_LEFT_CLICK_TOUCH_IO])
            buffer[0] = 0b1;
        if (s_pad_activated[CONFIG_RIGHT_CLICK_TOUCH_IO])
            buffer[0] = 0b10;
        if (s_pad_activated[CONFIG_LEFT_KEY_TOUCH_IO])
            buffer[1] -= move_delta;
        if (s_pad_activated[CONFIG_RIGHT_KEY_TOUCH_IO])
            buffer[1] += move_delta;
        if (s_pad_activated[CONFIG_UP_KEY_TOUCH_IO])
            buffer[2] -= move_delta;
        if (s_pad_activated[CONFIG_DOWN_KEY_TOUCH_IO])
            buffer[2] += move_delta;

        bool is_current_command_empty = buffer[0] == 0 && buffer[1] == 0 && buffer[2] == 0 && buffer[3] == 0;
        // If last command wasn't empty, we need to send even empty commands to say the command is over
        if (!is_current_command_empty || !was_last_command_empty)
            esp_hidd_dev_input_set(hid_dev, 0, 0, buffer, 4);
        was_last_command_empty = is_current_command_empty;

        RESET_PAD_ACTIVATED

        vTaskDelay(pdMS_TO_TICKS(CONFIG_INPUT_REFRESH_DELTA));
    }
}
    
