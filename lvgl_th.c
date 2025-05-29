
#include "./lvgl/lvgl.h"
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include "./lvgl/demos/lv_demos.h"

uint16_t disp_buf[2][320*160];

void *lvgl_ticker(void *arg)
{
    while(1)
    {
        usleep(10000);
        lv_tick_inc(10);   //unrecommended -- It's a rough ticker
        lv_timer_handler();
    }
}

extern void lvgl_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map);

void * lvgl_start()
{
    pthread_t tid;

    lv_init();
    lv_display_t * disp = lv_display_create(160, 80);
    lv_display_set_buffers(disp, disp_buf[0], disp_buf[1], 160*80*2, LV_DISPLAY_RENDER_MODE_DIRECT);
    lv_display_set_flush_cb(disp, lvgl_flush_cb);
    //sss
    if (pthread_create(&tid, NULL, lvgl_ticker, NULL) != 0) {
        printf("ticker thread create fail\n");
        return 1;
    }

    printf("h = %d, w = %d\n", lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));

    /*Change the active screen's background color*/
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x000000), LV_PART_MAIN);

    /*Create a white label, set its text and align it to the center*/
    /*Create a LED and switch it OFF*/
    // lv_obj_t * led1  = lv_led_create(lv_screen_active());
    // lv_obj_align(led1, LV_ALIGN_TOP_LEFT, 0, 0);
    // lv_obj_set_size(led1, 10, 80);

    /*Copy the previous LED and set a brightness*/
    lv_obj_t * led2  = lv_led_create(lv_screen_active());
    lv_obj_align(led2, LV_ALIGN_CENTER, 0, 0);
    lv_led_set_brightness(led2, 150);
    lv_led_set_color(led2, lv_palette_main(LV_PALETTE_RED));
    lv_led_on(led2);
    lv_obj_set_size(led2, 10, 10);

    // /*Copy the previous LED and switch it ON*/
    // lv_obj_t * led3  = lv_led_create(lv_screen_active());
    // // lv_obj_align(led3, LV_ALIGN_CENTER, +20, 0);
    // lv_led_on(led3);

    
    // lv_obj_set_size(led2, 10, 10);
    // lv_obj_set_size(led3, 10, 10);

    // lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    uint32_t i = 0xf;
    while(1) {
        // lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(i), LV_PART_MAIN);
        if (i<<1 <i) {
            i = i<<1 | 1;   
        } else {
            i = i<<1;
        }
        lv_led_toggle(led2);
        
        printf("i = %d\n", i);
        usleep(1000000);

    }
    // lv_demos_create("stress", 1);

    return;
}