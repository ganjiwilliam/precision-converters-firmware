# All the .c and .cpp and .h Files in SRC_DIRS are used in Build (Recursive)
SRC_DIRS += $(PROJECT_APP_PATH)
SRC_DIRS += $(LIBRARIES_PATH)/no-OS/drivers/adc/ad717x
SRC_DIRS += $(LIBRARIES_PATH)/no-OS/drivers/api
SRC_DIRS += $(LIBRARIES_PATH)/no-OS/drivers/platform/mbed
SRC_DIRS += $(LIBRARIES_PATH)/no-OS/iio
SRC_DIRS += $(LIBRARIES_PATH)/no-OS/util
SRC_DIRS += $(LIBRARIES_PATH)/no-OS/include
SRC_DIRS += $(LIBRARIES_PATH)/precision-converters-library/adi_console_menu

# Extra Macros
override NEW_CFLAGS += -DACTIVE_PLATFORM=MBED_PLATFORM
