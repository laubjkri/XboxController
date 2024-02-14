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

#define ANALOG_MAX 255
#define ANALOG_RANGE ANALOG_MAX + 1
#define ANALOG_HALF (ANALOG_RANGE) / 2 - 1
#define HALF_DEAD_ZONE 5
#define PWM_WRAP 1280 - 1
#define PIN_L_PWM 0
#define PIN_R_PWM 1
#define PIN_L_FWD 2
#define PIN_L_BWD 3
#define PIN_R_FWD 4
#define PIN_R_BWD 5

ControllerData controller;

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
    gpio_set_function(PIN_L_PWM, GPIO_FUNC_PWM);
    gpio_set_function(PIN_R_PWM, GPIO_FUNC_PWM);

    // Direction outputs
    gpio_init(PIN_L_FWD);
    gpio_init(PIN_L_BWD);
    gpio_init(PIN_R_FWD);
    gpio_init(PIN_R_BWD);
    gpio_set_dir(PIN_L_FWD, GPIO_OUT);
    gpio_set_dir(PIN_L_BWD, GPIO_OUT);
    gpio_set_dir(PIN_R_FWD, GPIO_OUT);
    gpio_set_dir(PIN_R_BWD, GPIO_OUT);    

    // There are 8 pwm slices each with two channels A/B
    uint left_slice_num  = pwm_gpio_to_slice_num(PIN_L_PWM);
    uint right_slice_num = pwm_gpio_to_slice_num(PIN_R_PWM);
    uint left_chan_num   = pwm_gpio_to_channel(PIN_L_PWM);
    uint right_chan_num  = pwm_gpio_to_channel(PIN_R_PWM);

     
    pwm_set_wrap(left_slice_num, PWM_WRAP);
    pwm_set_wrap(right_slice_num, PWM_WRAP);

    // PWM_res = 1280 (128 * 10)
    // F_pwm = 125Mhz / (80 + frac/16) / 1280 = 1220Hz
    const uint8_t clock_div_int = 80; // 0 <= i <= 255 (0 is max division of 256)
    const uint8_t clock_div_frac = 0;  // 0 <= f <= 15 (4bits range so 0-15, ie. 8 = 0.5)
    pwm_set_clkdiv_int_frac(left_slice_num, clock_div_int, clock_div_frac);
    pwm_set_clkdiv_int_frac(right_slice_num, clock_div_int, clock_div_frac);    

    pwm_set_enabled(left_slice_num, true);
    pwm_set_enabled(right_slice_num, true);

    int cal_ly = ANALOG_HALF;
    int cal_ry = ANALOG_HALF;

    // Halt program until bluetooth and other conditions are true
    while (true)
    {
        bool bt_ok = bt_state == READY;

        if (bt_ok)
        {
            // Would be cool to flash led here
        }


        bool calibrate = controller.button_a && controller.button_x;

        if (calibrate)
        {
            //cal_ly = controller.ly;
            //cal_ry = controller.ry;
            break;
        }

        sleep_ms(10);
    }



    // Program loop
    while (true)
    {
        update_led();
        
        // Convert 0-255 to -128 - 127        
        int ly_s = controller.ly - cal_ly;
        int ry_s = controller.ry - cal_ry;

        // Convert to absolute PWM res
        int ly_abs = abs(ly_s) * 10;
        int ry_abs = abs(ry_s) * 10;

        // Set PWM
        pwm_set_chan_level(left_slice_num, left_chan_num, ly_abs);
        pwm_set_chan_level(right_slice_num, right_chan_num, ry_abs);


        bool left_fwd = false;
        bool left_bwd = false;
        bool right_fwd = false;
        bool right_bwd = false;

        // Set direction left
        if (ly_s > 0 + HALF_DEAD_ZONE)
        {
            left_fwd = true;
        }
        else if (ly_s < 0 - HALF_DEAD_ZONE)
        {
            left_bwd = true;
        }
        gpio_put(PIN_L_FWD, left_fwd);
        gpio_put(PIN_L_BWD, left_bwd);


        // Set direction right
        if (ry_s > 0 + HALF_DEAD_ZONE)
        {
            right_fwd = true;
        }
        else if (ry_s < 0 - HALF_DEAD_ZONE)
        {
            right_bwd = true;
        }
        gpio_put(PIN_R_FWD, right_fwd);
        gpio_put(PIN_R_BWD, right_bwd);
        

        // Should be timered
        //DEBUG_LOG("PWM L,R: %04d,%04d, L: %01u,%01u, R: %01u,%01u\n",
        //    ly_abs, ry_abs, left_fwd, left_bwd, right_fwd, right_bwd);
        //sleep_ms(100);
    }
}

static inline void update_led()
{
    if (bt_state == READY)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    else
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);    
}

static inline bool get_bit_status(uint val, uint bit_no)
{
    /* Right shift num, n times and perform bitwise AND with 1 */
    return (val >> bit_no) & 1;
}

static inline void xbox_controller_event_handler(const uint8_t* data, uint16_t size)
{
    // Print the data
    controller.lx = data[1];
    controller.ly = ANALOG_MAX - data[3]; // inverted
    controller.rx = data[5];
    controller.ry = ANALOG_MAX - data[7]; // inverted
    controller.lt = data[8];
    controller.rt = data[10];
    controller.hat = data[12];
    controller.buttons = data[13];
    controller.button_a = get_bit_status(controller.buttons, 0); // 1
    controller.button_b = get_bit_status(controller.buttons, 1); // 2
    controller.button_x = get_bit_status(controller.buttons, 3); // 8
    controller.button_y = get_bit_status(controller.buttons, 4); // 16

    VERBOSE_LOG("Left Stick: X: %03u, Y: %03u, T: %03u - Right Stick: X: %03u, Y: %03u, T: %03u - Hat: %03u - Buttons: %03u\n",
        controller.lx,
        controller.ly,
        controller.lt,
        controller.rx,
        controller.ry,
        controller.rt,
        controller.hat,
        controller.buttons);
}
