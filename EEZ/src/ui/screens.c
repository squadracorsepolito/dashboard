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
                    lv_obj_set_pos(obj, -147, -134);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f20_style(obj);
                    lv_label_set_text(obj, "V");
                }
                {
                    // pg_main_hv_soc_est
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_hv_soc_est = obj;
                    lv_obj_set_pos(obj, 170, -135);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f32_style(obj);
                    lv_label_set_text(obj, "0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 211, -134);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f20_style(obj);
                    lv_label_set_text(obj, "%");
                }
                {
                    // pg_main_sx_status_bar
                    lv_obj_t *obj = lv_obj_create(parent_obj);
                    objects.pg_main_sx_status_bar = obj;
                    lv_obj_set_pos(obj, LV_PCT(4), LV_PCT(17));
                    lv_obj_set_size(obj, 30, 180);
                    lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                }
                {
                    // pg_main_dx_status_bar
                    lv_obj_t *obj = lv_obj_create(parent_obj);
                    objects.pg_main_dx_status_bar = obj;
                    lv_obj_set_pos(obj, LV_PCT(90), LV_PCT(17));
                    lv_obj_set_size(obj, 30, 180);
                    lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                }
                {
                    // pg_main_rot_1
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_rot_1 = obj;
                    lv_obj_set_pos(obj, -55, -134);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_label_set_text(obj, "9");
                }
                {
                    // pg_main_rot_2
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_rot_2 = obj;
                    lv_obj_set_pos(obj, 0, -134);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_label_set_text(obj, "9");
                }
                {
                    // pg_main_rot_3
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_rot_3 = obj;
                    lv_obj_set_pos(obj, 54, -134);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_label_set_text(obj, "9");
                }
                {
                    // mission_name
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.mission_name = obj;
                    lv_obj_set_pos(obj, -102, 124);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f38_style(obj);
                    lv_label_set_text(obj, "AS Off");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 78, 84);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Mot:");
                }
                {
                    // pg_main_motor_temp_max
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_motor_temp_max = obj;
                    lv_obj_set_pos(obj, 124, 84);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 158, 84);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 82, 109);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Inv:");
                }
                {
                    // pg_main_inv_temp_max
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_inv_temp_max = obj;
                    lv_obj_set_pos(obj, 124, 109);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 158, 109);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 79, 134);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "Cell:");
                }
                {
                    // pg_main_cell_temp_max
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_cell_temp_max = obj;
                    lv_obj_set_pos(obj, 124, 134);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "0.0");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 158, 134);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f18_style(obj);
                    lv_label_set_text(obj, "°C");
                }
            }
        }
        {
            lv_obj_t *obj = lv_obj_create(parent_obj);
            lv_obj_set_pos(obj, 0, -11);
            lv_obj_set_size(obj, 367, 209);
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
                    // pg_main_tc
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_tc = obj;
                    lv_obj_set_pos(obj, -103, -68);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f30_style(obj);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "TC");
                }
                {
                    // pg_main_tv
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_tv = obj;
                    lv_obj_set_pos(obj, -34, -68);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f30_style(obj);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "TV");
                }
                {
                    // pg_main_lc
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_lc = obj;
                    lv_obj_set_pos(obj, 32, -68);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f30_style(obj);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "LC");
                }
                {
                    // pg_main_reg
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.pg_main_reg = obj;
                    lv_obj_set_pos(obj, 99, -68);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f30_style(obj);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "RG");
                }
                {
                    // lap_time_actual
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.lap_time_actual = obj;
                    lv_obj_set_pos(obj, 87, 11);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f40_style(obj);
                    lv_label_set_text(obj, "0:00");
                }
                {
                    // lap_time_last
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.lap_time_last = obj;
                    lv_obj_set_pos(obj, -81, 29);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f30_style(obj);
                    lv_label_set_text(obj, "0:00");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -108, -8);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f30_style(obj);
                    lv_label_set_text(obj, "Lap");
                }
                {
                    // lap_number
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.lap_number = obj;
                    lv_obj_set_pos(obj, -46, -8);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    add_style_label_basic_f30_style(obj);
                    lv_label_set_text(obj, "00");
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