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

typedef struct {
    lv_obj_t *main;
    lv_obj_t *pg_main_lv_bat_v;
    lv_obj_t *pg_main_hv_soc_est;
    lv_obj_t *pg_main_sx_status_bar;
    lv_obj_t *pg_main_dx_status_bar;
    lv_obj_t *pg_main_sx_map;
    lv_obj_t *pg_main_dx_map;
    lv_obj_t *pg_main_lc;
    lv_obj_t *pg_main_tc;
    lv_obj_t *pg_main_tv;
    lv_obj_t *mission_name;
    lv_obj_t *inv_label;
    lv_obj_t *pg_main_inv_temp_fl;
    lv_obj_t *pg_main_inv_temp_fr;
    lv_obj_t *pg_main_cool_press_l;
    lv_obj_t *pg_main_cool_press_r;
    lv_obj_t *pg_main_inv_temp_rl;
    lv_obj_t *pg_main_inv_temp_rr;
    lv_obj_t *mot_label;
    lv_obj_t *pg_main_motor_temp_fl;
    lv_obj_t *pg_main_motor_temp_fr;
    lv_obj_t *pg_main_motor_temp_rl;
    lv_obj_t *pg_main_motor_temp_rr;
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