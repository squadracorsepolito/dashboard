#include "screens.h"
#include "styles.h"
#include "ui.h"

#include <string.h>

objects_t objects;
lv_obj_t *tick_value_change_obj;
uint32_t active_theme_index = 0;

void create_screen_tires() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.tires = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    {
        lv_obj_t *parent_obj = obj;
        {
            lv_obj_t *obj = lv_obj_create(parent_obj);
            lv_obj_set_pos(obj, 0, 0);
            lv_obj_set_size(obj, LV_PCT(100), LV_PCT(100));
            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            add_style_container_basic_style(obj);
            {
                lv_obj_t *parent_obj = obj;
                {
                    // pg_tires_lv_bat_v
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_lv_bat_v = obj;
                    lv_obj_set_pos(obj, -193, -135);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f32_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -136, -135);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f20_style(obj);
                    lv_label_set_text(obj, "V");
                }
                {
                    // pg_tires_hv_soc_est
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_hv_soc_est = obj;
                    lv_obj_set_pos(obj, 198, -135);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f32_style(obj);
                    lv_label_set_text(obj, "0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 147, -135);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f20_style(obj);
                    lv_label_set_text(obj, "%");
                }
                {
                    // pg_tires_sx_status_bar
                    lv_obj_t *obj = lv_obj_create(parent_obj);
                    objects.pg_tires_sx_status_bar = obj;
                    lv_obj_set_pos(obj, LV_PCT(5), LV_PCT(25));
                    lv_obj_set_size(obj, 36, 140);
                    lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                }
                {
                    // pg_tires_dx_status_bar
                    lv_obj_t *obj = lv_obj_create(parent_obj);
                    objects.pg_tires_dx_status_bar = obj;
                    lv_obj_set_pos(obj, LV_PCT(85), LV_PCT(25));
                    lv_obj_set_size(obj, 36, 140);
                    lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                }
                {
                    // pg_tires_sx_map
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_sx_map = obj;
                    lv_obj_set_pos(obj, -198, 110);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_label_set_text(obj, "9");
                }
                {
                    // pg_tires_dx_map
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_dx_map = obj;
                    lv_obj_set_pos(obj, 186, 112);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_label_set_text(obj, "9");
                }
                {
                    // pg_tires_lc
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_lc = obj;
                    lv_obj_set_pos(obj, -101, 114);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "LC");
                }
                {
                    // pg_tires_tc
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_tc = obj;
                    lv_obj_set_pos(obj, -5, 115);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "TC");
                }
                {
                    // pg_tires_tv
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_tv = obj;
                    lv_obj_set_pos(obj, 91, 117);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "TV");
                }
            }
        }
        {
            lv_obj_t *obj = lv_obj_create(parent_obj);
            lv_obj_set_pos(obj, 0, 0);
            lv_obj_set_size(obj, 313, 188);
            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            add_style_container_basic_style(obj);
            {
                lv_obj_t *parent_obj = obj;
                {
                    // pg_tires_fl_temp
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_fl_temp = obj;
                    lv_obj_set_pos(obj, -113, -74);
                    lv_obj_set_size(obj, 100, LV_SIZE_CONTENT);
                    add_style_label_basic_f32_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -54, -75);
                    lv_obj_set_size(obj, 55, LV_SIZE_CONTENT);
                    add_style_label_basic_f20_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    // pg_tires_fl_bar
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_fl_bar = obj;
                    lv_obj_set_pos(obj, -114, -34);
                    lv_obj_set_size(obj, 100, LV_SIZE_CONTENT);
                    add_style_label_basic_f32_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -55, -34);
                    lv_obj_set_size(obj, 55, LV_SIZE_CONTENT);
                    add_style_label_basic_f20_style(obj);
                    lv_label_set_text(obj, "Bar");
                }
                {
                    // pg_tires_fr_temp
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_fr_temp = obj;
                    lv_obj_set_pos(obj, 50, -75);
                    lv_obj_set_size(obj, 100, LV_SIZE_CONTENT);
                    add_style_label_basic_f32_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 109, -75);
                    lv_obj_set_size(obj, 55, LV_SIZE_CONTENT);
                    add_style_label_basic_f20_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    // pg_tires_fr_bar
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_fr_bar = obj;
                    lv_obj_set_pos(obj, 50, -33);
                    lv_obj_set_size(obj, 100, LV_SIZE_CONTENT);
                    add_style_label_basic_f32_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 109, -34);
                    lv_obj_set_size(obj, 55, LV_SIZE_CONTENT);
                    add_style_label_basic_f20_style(obj);
                    lv_label_set_text(obj, "Bar");
                }
                {
                    // pg_tires_rl_temp
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_rl_temp = obj;
                    lv_obj_set_pos(obj, -114, 64);
                    lv_obj_set_size(obj, 100, LV_SIZE_CONTENT);
                    add_style_label_basic_f32_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -56, 25);
                    lv_obj_set_size(obj, 55, LV_SIZE_CONTENT);
                    add_style_label_basic_f20_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    // pg_tires_rl_bar
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_rl_bar = obj;
                    lv_obj_set_pos(obj, -115, 24);
                    lv_obj_set_size(obj, 100, LV_SIZE_CONTENT);
                    add_style_label_basic_f32_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -55, 66);
                    lv_obj_set_size(obj, 55, LV_SIZE_CONTENT);
                    add_style_label_basic_f20_style(obj);
                    lv_label_set_text(obj, "Bar");
                }
                {
                    // pg_tires_rr_temp
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_rr_temp = obj;
                    lv_obj_set_pos(obj, 50, 25);
                    lv_obj_set_size(obj, 100, LV_SIZE_CONTENT);
                    add_style_label_basic_f32_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 109, 26);
                    lv_obj_set_size(obj, 55, LV_SIZE_CONTENT);
                    add_style_label_basic_f20_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    // pg_tires_rr_bar
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_tires_rr_bar = obj;
                    lv_obj_set_pos(obj, 50, 67);
                    lv_obj_set_size(obj, 100, LV_SIZE_CONTENT);
                    add_style_label_basic_f32_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 109, 67);
                    lv_obj_set_size(obj, 55, LV_SIZE_CONTENT);
                    add_style_label_basic_f20_style(obj);
                    lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Bar");
                }
            }
        }
    }
    
    tick_screen_tires();
}

void tick_screen_tires() {
}



typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_tires,
};
void tick_screen(int screen_index) {
    tick_screen_funcs[screen_index]();
}
void tick_screen_by_id(enum ScreensEnum screenId) {
    tick_screen_funcs[screenId - 1]();
}

void create_screens() {
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    
    create_screen_tires();
}
