/*
 * qinheng usb screen demo
 * Copyright © 2025 Champion. All rights reserved.
 * Contact: xcdanswer@gmail.com
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

 #include <stdio.h>
 #include "stdlib.h"
 #include "string.h"
 #include "fcntl.h"
 #include "unistd.h"
 #include "pthread.h"
 #include <libusb-1.0/libusb.h>
 #include "./lvgl/lvgl.h"

#define EP_OUT 0x02
#define EP_IN  0x83

#define SWAP_16(var) var = ((var&0xff)<<8) | ((var&0xff00)>>8)

struct ulcd_device {
    int (*write)(struct ulcd_device *dev, uint8_t *buf, int len);
    int (*read)(struct ulcd_device *dev, uint8_t *buf, int len);
    libusb_device_handle *h;
    int width;
    int height;  
    int intf;
};

#pragma pack(push, 1)
struct ulcd_ins {
    uint8_t typ;
    uint8_t cmd;
    uint32_t val;
};
#pragma pack(pop)

struct ulcd_device g_ulcd;


void dump_img(char *file_name, uint8_t *src, uint32_t size)
{
    int fd = open("./dump.bin", O_WRONLY | O_CREAT);
    // int fd = open("./dump.bin", O_WRONLY | O_APPEND);
    if (fd == -1) {
        printf("dump fail\n");
        return;
    }

    write(fd, src, size);

    close(fd);
    return;
}

int ulcd_write(struct ulcd_device *dev, uint8_t *buf, int len)
{
    int done;
    libusb_bulk_transfer(dev->h, EP_OUT, buf, len, &done, 1000);
    return done;
}

int ulcd_read(struct ulcd_device *dev, uint8_t *buf, int len)
{
    int done;
    libusb_bulk_transfer(dev->h, EP_IN, buf, len, &done, 1000);
    // libusb_interrupt_transfer(dev->h, EP_IN, buf, len, &done, 1000);
    return done;
}

void ulcd_ins_init2(struct ulcd_ins *ins, uint8_t cmd, uint32_t val)
{
    ins->typ = 2;  //multi write
    ins->cmd = cmd;
    ins->val = val;
}

void ulcd_ins_init4(struct ulcd_ins *ins, uint8_t cmd, uint32_t val)
{
    ins->typ = 4;  //multi write
    ins->cmd = cmd;
    ins->val = val;
}

uint32_t find_common_hex(uint32_t *hex)
{
    //i: idx of element input(*hex)
    //j: idx of element in set 
    //k: cnt of element in set
    int i, j, k = 0;
    uint32_t buf[64] = {0};
    uint8_t times[64] = {0};
    uint32_t *hex2 = hex;
    uint32_t ret;
    
    for (i = 0;i<64;i++)
    {  
        for (j = 0;j<k;j++)
        {
            if (hex2[i] == buf[j]) {
                times[j]++;
                break;
            }
        }
        if (j == k) {
            //un match
            buf[j] = hex2[i];
            times[j]++;
            k++;
        }
    }

    //find max
    ret = buf[0];
    for (i = 0;i<k;i++)
    {
        if (times[i] > times[i+1]) {
            times[i+1] = times[i];
        } else {
            ret = buf[i+1];
        }
    }
    return ret;
}

int rgb16_to_ins(uint16_t *rgb16, struct ulcd_ins *ins)
{
    uint8_t *p8;
    int idx = 0;
    int ret = 0;
    uint32_t rgb16x2[64];
    for (int i = 0;i<64;i++) {
        rgb16x2[i] = (rgb16[i*2+0]<<16) | rgb16[i*2+1];
    }
    uint32_t common = find_common_hex(rgb16x2);
    // printf("c is %x\n", common);
    ulcd_ins_init2(ins++, 4, common);
    for (int i = 0;i<64;i++)
    {
        if (rgb16x2[i] != common) {
            // printf("i = %d, 0x%x, 0x%x\n", i, rgb16x2[i], common);
            ulcd_ins_init4(ins++, i, rgb16x2[i]);
            ret++;
        }
    }
    ulcd_ins_init2(ins++, 3, 0x00000108);
    // printf("cnt of ins: %d\n", ret);
    return ret + 2;
}

struct ulcd_img_rgb16 {
    uint32_t x;
    uint32_t y;
    uint16_t *src;
    struct ulcd_ins *ins;
};

int img_to_ins(struct ulcd_img_rgb16 *img)
{
    struct ulcd_ins *ins;
    int cnt = 0;
    int cur;
    img->ins = malloc((img->x * img->y * 3 / 2) * sizeof(struct ulcd_ins));
    if (!img->ins) {
        return 1;
    }
    ins = img->ins;

    for (int i = 0;i<img->x*img->y/128;i++) 
    {
        cur = rgb16_to_ins(&img->src[i*128], ins);
        ins += cur;
        cnt += cur;
    }
    return cnt;
}


volatile uint16_t flush_buf[160*80];
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER; // 静态初始化
void lvgl_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map)
{
    /* The most simple case (also the slowest) to send all rendered pixels to the
     * screen one-by-one.  `put_px` is just an example.  It needs to be implemented by you. */
    static int i = 0;
    uint16_t * rgb = (uint16_t *)px_map; /* Let's say it's a 16 bit (RGB565) display */
    // memcpy(flush_buf, rgb, 160*80*2);
    pthread_mutex_lock(&mutex); // 加锁
    for (int i = 0;i<160*80;i++)
    {
        flush_buf[i] = ((rgb[i]>>8) & 0xff)| ((rgb[i]<<8) & 0xff00);
        // flush_buf[i] = rgb[i];
    }
    pthread_mutex_unlock(&mutex); // 解锁


    dump_img(NULL, rgb, 160*80*2);
    
    printf("cb %d\n", i++);

    /* IMPORTANT!!!
     * Inform LVGL that flushing is complete so buffer can be modified again. */
    lv_display_flush_ready(display);
    printf("end\n");
}

int ulcd_init(struct ulcd_device *dev)
{
    int r;
    ssize_t cnt;
    int done;
 
    r = libusb_init(NULL);
    if (r < 0)
        return r;
 
    dev->h = libusb_open_device_with_vid_pid(NULL, (uint16_t)0x1a86, (uint16_t)0xfe0c);
    if (dev->h == NULL) {
        printf("libusb open fail\n");
    }

    libusb_set_auto_detach_kernel_driver(dev->h, 1);
    r = libusb_claim_interface(dev->h, 1);
    if (r != LIBUSB_SUCCESS) {
        libusb_close(dev->h);
        printf("claim interface fail\n");
        return 1;
    }

    dev->write = ulcd_write;
    dev->read  = ulcd_read;
    dev->width = 160;
    dev->height = 80;
    dev->intf = 1;

    uint8_t buf[32];
    uint8_t connect_sig[] = "\0MSNCN";
    uint8_t listen_sig[] = "\0MSN01";
    libusb_control_transfer(dev->h, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE, // bmRequestType
        0x21, /* SET_CONTROL_LINE_STATE */
        0, /* wValue: 控制线状态(DTR和RTS) */
        dev->intf, /* wIndex: 接口号 */
        buf, /* data buffer: data state buf */
        7, /* wLength: data state buf len */
        1000); /* timeout (milliseconds) */
    
    printf("conecting");
    // exit(0); 
    while (1)
    {
        printf(".");
        r = ulcd_read(dev, buf, 16);
        if (!memcmp(buf, listen_sig, 6)) {
            ulcd_write(dev, connect_sig, 6);
            usleep(200*1000);
            
            ulcd_read(dev, buf, 16);
            if (!memcmp(buf, listen_sig, 6)) {
                break;
            }
        }
        usleep(150*1000);
    }
    
    printf("\nconnect done\n");
       
    return 0;
}

void ulcd_exit(struct ulcd_device *dev)
{
    libusb_release_interface(dev->h, dev->intf);
    libusb_close(dev->h); 
    libusb_exit(NULL);
}

int lcd_add(struct ulcd_device *dev)
{
    uint8_t set_xy_pos_to_00[] = {2,0,0,0,0,0};
    uint8_t set_xy_size_to_160_80[] = {2,1,0,160,0,80};
    dev->write(dev, set_xy_pos_to_00, 6);
    dev->write(dev, set_xy_size_to_160_80, 6);
    return 0;
}

extern void * lvgl_start();

void flush_period()
{
    int r;
    struct ulcd_img_rgb16 img = {
        .x = 160,
        .y = 80,
        .src = flush_buf,
    };

    while(1)
    {
        img.src = flush_buf;
        pthread_mutex_lock(&mutex); // 加锁
        r = img_to_ins(&img);
        pthread_mutex_unlock(&mutex); // 解锁
        
        g_ulcd.write(&g_ulcd, (void *)img.ins, r * sizeof(struct ulcd_ins));
        free(img.ins);
        usleep(500000);
        printf(".");
    }
}

int main(void)
{
    int ret;
    pthread_t tid;
    struct ulcd_device *ulcd = &g_ulcd;
    pthread_mutex_init(&mutex, NULL);
    ret = ulcd_init(ulcd);
    if (ret) {
        printf("ulcd init fail\n");
    }

    if (pthread_create(&tid, NULL, lvgl_start, NULL) != 0) {
        printf("ticker thread create fail\n");
        return 1;
    }
    flush_period();
    ulcd_exit(ulcd);
    return 0;
}

int main1(void)
{
    int ret;
    pthread_t tid;
    struct ulcd_device *ulcd = &g_ulcd;

    printf("ssss\n");

    if (pthread_create(&tid, NULL, lvgl_start, NULL) != 0) {
        printf("ticker thread create fail\n");
        return 1;
    }
    while(1);
    return 0;
}








