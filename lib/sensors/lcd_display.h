#ifndef __LCD_DISPLAY_H__
#define __LCD_DISPLAY_H__

#include "project_config.h"
#include "def_consts.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "reLCD.h"

// Размер стека LCD-задачи — отдельно от задачи сенсоров
#ifndef CONFIG_LCD_TASK_STACK_SIZE
#define CONFIG_LCD_TASK_STACK_SIZE (3 * 1024)
#endif

// Объект дисплея — создан в sensors.cpp, используется здесь для вывода
extern reLCD lcd;

// Structure to hold sensor data for display
typedef struct {
    float indoor_temp;
    float outdoor_temp;
    float boiler_temp;
    bool indoor_valid;
    bool outdoor_valid;
    bool boiler_valid;
    bool boiler_state;
    float thermostat_temp;
    char mode_char;
} lcd_display_data_t;

// Вызывается из sensorsTaskExec после каждого опроса датчиков —
// передаёт актуальные данные в LCD-задачу без прямого доступа к объектам сенсоров
void lcdUpdateData(
    float indoor_temp, bool indoor_valid,
    float outdoor_temp, bool outdoor_valid,
    float boiler_temp, bool boiler_valid,
    bool boiler_state,
    float thermostat_temp,
    char mode_char);

// Function declarations
bool lcdDisplayInit();
bool lcdDisplayStart();
bool lcdDisplayStop();
void lcdDisplayTask(void* arg);
void lcdDisplayFormatData(const lcd_display_data_t& data, char* line1, char* line2);

#endif // __LCD_DISPLAY_H__ 