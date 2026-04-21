#include "screens.h"
#include "styles.h"
#include "images.h"
#include "ui.h"

#include <string.h>

objects_t objects;

//
// Event handlers
//

lv_obj_t *tick_value_change_obj;

//
// Screens
//

void create_screen_main() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(obj, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_opa(obj, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
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
                    // pg_main_lv_bat_v
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_lv_bat_v = obj;
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
                    // pg_main_hv_soc_est
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_hv_soc_est = obj;
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
                    // pg_main_sx_status_bar
                    lv_obj_t *obj = lv_obj_create(parent_obj);
                    objects.pg_main_sx_status_bar = obj;
                    lv_obj_set_pos(obj, LV_PCT(5), LV_PCT(25));
                    lv_obj_set_size(obj, 36, 140);
                    lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                }
                {
                    // pg_main_dx_status_bar
                    lv_obj_t *obj = lv_obj_create(parent_obj);
                    objects.pg_main_dx_status_bar = obj;
                    lv_obj_set_pos(obj, LV_PCT(85), LV_PCT(25));
                    lv_obj_set_size(obj, 36, 140);
                    lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                }
                {
                    // pg_main_sx_map
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_sx_map = obj;
                    lv_obj_set_pos(obj, -198, 110);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_label_set_text(obj, "9");
                }
                {
                    // pg_main_dx_map
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_dx_map = obj;
                    lv_obj_set_pos(obj, 186, 112);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_label_set_text(obj, "9");
                }
                {
                    // pg_main_lc
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_lc = obj;
                    lv_obj_set_pos(obj, -117, 117);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "LC");
                }
                {
                    // pg_main_tc
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_tc = obj;
                    lv_obj_set_pos(obj, 0, 117);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "TC");
                }
                {
                    // pg_main_tv
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_tv = obj;
                    lv_obj_set_pos(obj, 117, 117);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "TV");
                }
                {
                    // mission_name
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.mission_name = obj;
                    lv_obj_set_pos(obj, 0, -135);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f32_style(obj);
                    lv_label_set_text(obj, "AS Off");
                }
            }
        }
        {
            lv_obj_t *obj = lv_obj_create(parent_obj);
            lv_obj_set_pos(obj, -12, -12);
            lv_obj_set_size(obj, 313, 209);
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
                    // inv_label
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.inv_label = obj;
                    lv_obj_set_pos(obj, -112, -81);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Inverters");
                }
                {
                    // pg_main_inv_temp_fl
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_inv_temp_fl = obj;
                    lv_obj_set_pos(obj, -138, -51);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    // pg_main_inv_temp_fr
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_inv_temp_fr = obj;
                    lv_obj_set_pos(obj, -56, -51);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -90, 37);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Cool press L");
                }
                {
                    // pg_main_cool_press_l
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_cool_press_l = obj;
                    lv_obj_set_pos(obj, -12, 37);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -89, 72);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Cool press R");
                }
                {
                    // pg_main_cool_press_r
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_cool_press_r = obj;
                    lv_obj_set_pos(obj, -12, 72);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -102, -51);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -102, -12);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    // pg_main_inv_temp_rl
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_inv_temp_rl = obj;
                    lv_obj_set_pos(obj, -138, -12);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    // pg_main_inv_temp_rr
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_inv_temp_rr = obj;
                    lv_obj_set_pos(obj, -56, -12);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -20, -51);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -20, -12);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 26, 37);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Bar");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 26, 72);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Bar");
                }
                {
                    // mot_label
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.mot_label = obj;
                    lv_obj_set_pos(obj, 45, -81);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Motors");
                }
                {
                    // pg_main_motor_temp_fl
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_motor_temp_fl = obj;
                    lv_obj_set_pos(obj, 28, -51);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    // pg_main_motor_temp_fr
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_motor_temp_fr = obj;
                    lv_obj_set_pos(obj, 112, -51);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 65, -51);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 65, -12);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    // pg_main_motor_temp_rl
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_motor_temp_rl = obj;
                    lv_obj_set_pos(obj, 28, -12);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    // pg_main_motor_temp_rr
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_motor_temp_rr = obj;
                    lv_obj_set_pos(obj, 112, -12);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 149, -51);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 149, -12);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
            }
        }
    }
    
    tick_screen_main();
}

void tick_screen_main() {
}

typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_main,
};
void tick_screen(int screen_index) {
    tick_screen_funcs[screen_index]();
}
void tick_screen_by_id(enum ScreensEnum screenId) {
    tick_screen_funcs[screenId - 1]();
}

void create_screens() {

// Set default LVGL theme
    lv_display_t *dispp = lv_display_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
    lv_display_set_theme(dispp, theme);
    
    // Initialize screens
    // Create screens
    create_screen_main();
}