/*
    Xbox controller
    Bluetooth
    MAC: 44:16:22:6c:c0:65
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
#include "hog_host.h"
#include "xbox_controller.h"
#include "hardware/pwm.h"


typedef struct
{
    uint8_t lx;
    uint8_t ly;
    uint8_t rx;
    uint8_t ry;
    uint8_t lt;
    uint8_t rt;
    uint8_t buttons;
    uint8_t hat;
    bool button_a;
    bool button_b;
    bool button_x;
    bool button_y;

} ControllerData;

ControllerData controller_data;

static void xbox_controller_event_handler(const uint8_t* data, uint16_t size)
{
    // Print the data
    controller_data.lx      = data[1];
    controller_data.ly      = 255 - data[3]; // inverted
    controller_data.rx      = data[5];
    controller_data.ry      = 255 - data[7]; // inverted
    controller_data.lt      = data[8];
    controller_data.rt      = data[10];
    controller_data.hat     = data[12];
    controller_data.buttons = data[13];

    printf("Left Stick: X: %03u, Y: %03u, T: %03u - Right Stick: X: %03u, Y: %03u, T: %03u - Hat: %03u - Buttons: %03u\n", 
        controller_data.lx, 
        controller_data.ly, 
        controller_data.lt,
        controller_data.rx, 
        controller_data.ry, 
        controller_data.rt,
        controller_data.hat,
        controller_data.buttons);

    //printf("Size: %u, 0: %03u, 1: %03u, 2: %03u, 3: %03u, 4: %03u, 5: %03u, 6: %03u, 7: %03u, 8: %03u, 9: %03u, 10: %03u, 11: %03u, 12: %03u, 13: %03u, 14: %03u\n",
    //    size,
    //    data[0],
    //    data[1],
    //    data[2],
    //    data[3],
    //    data[4],
    //    data[5],
    //    data[6],
    //    data[7],
    //    data[8],
    //    data[9],
    //    data[10],
    //    data[11],
    //    data[12],
    //    data[13],
    //    data[14]
    //);
}

int main() {
    
    stdio_init_all();

    // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    sleep_ms(2000);
    
    // The BT stack runs async with callbacks... Could also be called with RTOS
    btstack_main(xbox_controller_event_handler);

    // PWM outputs
    const uint left_motor_pin = 0;
    const uint right_motor_pin = 0;
    gpio_set_function(left_motor_pin, GPIO_FUNC_PWM);
    gpio_set_function(right_motor_pin, GPIO_FUNC_PWM);

    // There are 8 pwm slices each with two channels A/B
    uint left_slice_num = pwm_gpio_to_slice_num(left_motor_pin);
    uint right_slice_num = pwm_gpio_to_slice_num(right_motor_pin);
    uint left_chan_num = pwm_gpio_to_channel(left_motor_pin);
    uint right_chan_num = pwm_gpio_to_channel(left_motor_pin);

    const uint pwm_wrap = 255; // PWM wrap => resolution = wrap + 1
    pwm_set_wrap(left_slice_num, pwm_wrap);
    pwm_set_wrap(right_slice_num, pwm_wrap);

    // F_pwm = 125Mhz / (125 + frac/16) / 256 = 3,9kHz
    const uint8_t clock_div_int = 125; // 0 <= i <= 255 (0 is max division of 256)
    const uint8_t clock_div_frac = 0;  // 0 <= f <= 15 (4bits range so 0-15, ie. 8 = 0.5)
    pwm_set_clkdiv_int_frac(left_slice_num, clock_div_int, clock_div_frac);
    pwm_set_clkdiv_int_frac(right_slice_num, clock_div_int, clock_div_frac);    

    pwm_set_enabled(left_slice_num, true);
    pwm_set_enabled(right_slice_num, true);


    // Program loop
    while (true)
    {
        update_led();

        pwm_set_chan_level(left_slice_num, left_chan_num, controller_data.lt);
        pwm_set_chan_level(right_slice_num, right_chan_num, controller_data.rt);
        
        sleep_ms(10);
    }
}

void update_led()
{
    if (bt_state == READY)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    else
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);    
}

