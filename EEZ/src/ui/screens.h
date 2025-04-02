#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _objects_t {
    lv_obj_t *tires;
    lv_obj_t *main;
    lv_obj_t *inverters;
    lv_obj_t *extra;
    lv_obj_t *pg_tires_fl_temp;
    lv_obj_t *pg_tires_fl_bar;
    lv_obj_t *obj0;
    lv_obj_t *pg_tires_fr_temp;
    lv_obj_t *pg_tires_fr_bar;
    lv_obj_t *obj1;
    lv_obj_t *pg_tires_rl_temp;
    lv_obj_t *pg_tires_rl_bar;
    lv_obj_t *obj2;
    lv_obj_t *pg_tires_rr_temp;
    lv_obj_t *pg_tires_rr_bar;
    lv_obj_t *obj3;
    lv_obj_t *pg_tires_lv_bat_v;
    lv_obj_t *pg_tires_hv_soc_est;
    lv_obj_t *pg_tires_sx_status_bar;
    lv_obj_t *pg_tires_dx_status_bar;
    lv_obj_t *pg_tires_sx_map;
    lv_obj_t *pg_tires_lc;
    lv_obj_t *pg_tires_tc;
    lv_obj_t *pg_tires_tv;
    lv_obj_t *pg_tires_dx_map;
    lv_obj_t *pg_main_current_time;
    lv_obj_t *pg_main_last_time;
    lv_obj_t *pg_main_target_time;
    lv_obj_t *pg_main_lv_bat_v;
    lv_obj_t *pg_main_hv_soc_est;
    lv_obj_t *pg_main_sx_status_bar;
    lv_obj_t *pg_main_dx_status_bar;
    lv_obj_t *pg_main_sx_map;
    lv_obj_t *pg_main_lc;
    lv_obj_t *pg_main_tc;
    lv_obj_t *pg_main_tv;
    lv_obj_t *pg_main_dx_map;
    lv_obj_t *pg_inverters_fl_temp_inv;
    lv_obj_t *pg_inverters_fl_inv_state;
    lv_obj_t *pg_inverters_fr_temp_inv;
    lv_obj_t *pg_inverters_fr_inv_state;
    lv_obj_t *pg_inverters_rl_temp_inv;
    lv_obj_t *pg_inverters_rl_inv_state;
    lv_obj_t *pg_inverters_rr_temp_inv;
    lv_obj_t *pg_inverters_rr_inv_state;
    lv_obj_t *pg_inverters_lv_bat_v;
    lv_obj_t *pg_inverters_hv_soc_est;
    lv_obj_t *pg_inverters_sx_status_bar;
    lv_obj_t *pg_inverters_dx_status_bar;
    lv_obj_t *pg_inverters_sx_map;
    lv_obj_t *pg_inverters_lc;
    lv_obj_t *pg_inverters_tc;
    lv_obj_t *pg_inverters_tv;
    lv_obj_t *pg_inverters_dx_map;
    lv_obj_t *pg_extra_cooling_sys_bar;
    lv_obj_t *pg_extra_cooling_sys_temp;
    lv_obj_t *pg_extra_throttle_perc;
    lv_obj_t *pg_extra_rear_brake_bar;
    lv_obj_t *pg_extra_front_brake_bar;
    lv_obj_t *pg_extra_lv_bat_v;
    lv_obj_t *pg_extra_hv_soc_est;
    lv_obj_t *pg_extra_sx_status_bar;
    lv_obj_t *pg_extra_dx_status_bar;
    lv_obj_t *pg_extra_sx_map;
    lv_obj_t *pg_extra_lc;
    lv_obj_t *pg_extra_tc;
    lv_obj_t *pg_extra_tv;
    lv_obj_t *pg_extra_dx_map;
} objects_t;

extern objects_t objects;

enum ScreensEnum {
    SCREEN_ID_TIRES = 1,
    SCREEN_ID_MAIN = 2,
    SCREEN_ID_INVERTERS = 3,
    SCREEN_ID_EXTRA = 4,
};

void create_screen_tires();
void tick_screen_tires();

void create_screen_main();
void tick_screen_main();

void create_screen_inverters();
void tick_screen_inverters();

void create_screen_extra();
void tick_screen_extra();

void create_screens();
void tick_screen(int screen_index);


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/