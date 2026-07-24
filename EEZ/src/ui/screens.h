#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// Screens

enum ScreensEnum {
    _SCREEN_ID_FIRST = 1,
    SCREEN_ID_MAIN = 1,
    _SCREEN_ID_LAST = 1
};

typedef struct _objects_t {
    lv_obj_t *main;
    lv_obj_t *pg_main_lv_bat_v;
    lv_obj_t *pg_main_hv_soc_est;
    lv_obj_t *pg_main_sx_status_bar;
    lv_obj_t *pg_main_dx_status_bar;
    lv_obj_t *pg_main_rot_1;
    lv_obj_t *pg_main_rot_2;
    lv_obj_t *pg_main_rot_3;
    lv_obj_t *mission_name;
    lv_obj_t *pg_main_motor_temp_max;
    lv_obj_t *pg_main_inv_temp_max;
    lv_obj_t *pg_main_cell_temp_max;
    lv_obj_t *pg_main_tc;
    lv_obj_t *pg_main_tv;
    lv_obj_t *pg_main_lc;
    lv_obj_t *pg_main_reg;
    lv_obj_t *lap_time_actual;
    lv_obj_t *lap_time_last;
    lv_obj_t *lap_number;
} objects_t;

extern objects_t objects;

void create_screen_main();
void tick_screen_main();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/