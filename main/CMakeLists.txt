set(srcs "esp_hid_device_main.c"
         "esp_hid_gap.c"
         "bt_hid_task.c")
set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid)
