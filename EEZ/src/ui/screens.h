#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _objects_t {
    lv_obj_t *tires;
    lv_obj_t *pg_tires_lv_bat_v;
    lv_obj_t *pg_tires_hv_soc_est;
    lv_obj_t *pg_tires_sx_status_bar;
    lv_obj_t *pg_tires_dx_status_bar;
    lv_obj_t *pg_tires_sx_map;
    lv_obj_t *pg_tires_dx_map;
    lv_obj_t *pg_tires_lc;
    lv_obj_t *pg_tires_tc;
    lv_obj_t *pg_tires_tv;
    lv_obj_t *pg_tires_fl_temp;
    lv_obj_t *pg_tires_fl_bar;
    lv_obj_t *pg_tires_fr_temp;
    lv_obj_t *pg_tires_fr_bar;
    lv_obj_t *pg_tires_rl_temp;
    lv_obj_t *pg_tires_rl_bar;
    lv_obj_t *pg_tires_rr_temp;
    lv_obj_t *pg_tires_rr_bar;
} objects_t;

extern objects_t objects;

enum ScreensEnum {
    SCREEN_ID_TIRES = 1,
};

void create_screen_tires();
void tick_screen_tires();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/