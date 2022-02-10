
#include "bt_hid_task.h"

#include "esp_hidd.h"
#include "esp_hid_gap.h"


#define RESET_BUFFER for (int i = 0; i < 4; i++) buffer[i] = 0;
void bt_hid_task(void *pvParameters)
{
    esp_hidd_dev_t *hid_dev = pvParameters;

    printf("wasdqe, type something\n");

    static uint8_t buffer[4] = {0};
    RESET_BUFFER

    char c;
    bool is_input_set = true;
    while (1) {
        c = fgetc(stdin);
        switch (c) {
            case 'q':   // left click
                buffer[0] = 1;
                break;
            case 'e':   // right click
                buffer[0] = 2;
                break;
            case 'w':   // cursor goes up
                buffer[2] = -10;
                break;
            case 'a':   // cursor goes left
                buffer[1] = -10;
                break;
            case 's':   // cursor goes down
                buffer[2] = 10;
                break;
            case 'd':   // cursor goes right
                buffer[1] = 10;
                break;
            default:
                is_input_set = false;
                break;
        }

        if (is_input_set) {
            esp_hidd_dev_input_set(hid_dev, 0, 0, buffer, 4);
            RESET_BUFFER
        }
        else
            is_input_set = true;

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
    
