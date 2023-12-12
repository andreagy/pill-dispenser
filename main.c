#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>


#define OPTOFORK_PIN 28
#define PIEZO_PIN 27

#define MOTOR_PIN_1 2
#define MOTOR_PIN_2 3
#define MOTOR_PIN_3 6
#define MOTOR_PIN_4 13

#define LED1_PIN 22
#define LED2_PIN 21
#define LED3_PIN 20

#define SW0_PIN 9
#define SW1_PIN 8
#define SW2_PIN 7

#define PWM_CLKDIV 125
#define PWM_WRAP 999

volatile bool ledTimerFired = false;
bool timer_callback(repeating_timer_t *rt) {
    ledTimerFired = true;
    return true; // keep repeating
}

const uint MOTOR_PINS[] = {MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4};

// Half-step sequence
const int HALF_STEP_SEQUENCE[8][4] = {
        {1, 0, 0, 0},
        {1, 1, 0, 0},
        {0, 1, 0, 0},
        {0, 1, 1, 0},
        {0, 0, 1, 0},
        {0, 0, 1, 1},
        {0, 0, 0, 1},
        {1, 0, 0, 1}
};

void stepMotor(int step) {
    for (int pin = 0; pin < 4; ++pin) {
        gpio_put(MOTOR_PINS[pin], HALF_STEP_SEQUENCE[step][pin]);
    }
}

static int global_step_count = 0;
void runMotor(int fraction, int averageSteps) {
    int stepsToRun = (averageSteps / 8)*fraction;
    int target_step_count = global_step_count + stepsToRun;

    for (; global_step_count < target_step_count; global_step_count++) {
        stepMotor(global_step_count % 8);
        sleep_ms(2);
    }
}


// Interrupt handler for the opto fork and piezo sensor
volatile bool optoforkTriggered = false;
volatile bool pillDispensed = false;
void generic_irq_callback(uint gpio, uint32_t event_mask){
    if (gpio == OPTOFORK_PIN) {
        optoforkTriggered = true;
    }
    if (gpio == PIEZO_PIN) {
        pillDispensed = true;
    }
}


int main() {
    gpio_init(OPTOFORK_PIN);
    gpio_set_dir(OPTOFORK_PIN, GPIO_IN);
    gpio_pull_up(OPTOFORK_PIN);

    gpio_init(PIEZO_PIN);
    gpio_set_dir(PIEZO_PIN, GPIO_IN);
    gpio_pull_up(PIEZO_PIN);

    //Initializing stepper motor pins
    gpio_init(MOTOR_PIN_1);
    gpio_set_dir(MOTOR_PIN_1, GPIO_OUT);
    gpio_init(MOTOR_PIN_2);
    gpio_set_dir(MOTOR_PIN_2, GPIO_OUT);
    gpio_init(MOTOR_PIN_3);
    gpio_set_dir(MOTOR_PIN_3, GPIO_OUT);
    gpio_init(MOTOR_PIN_4);
    gpio_set_dir(MOTOR_PIN_4, GPIO_OUT);

    //Initializing LED1 pin
    gpio_init(LED1_PIN);
    gpio_set_function(LED1_PIN, GPIO_FUNC_PWM);
    gpio_set_dir(LED1_PIN, GPIO_OUT);

    //Initializing LED2 pin
    gpio_init(LED2_PIN);
    gpio_set_function(LED2_PIN, GPIO_FUNC_PWM);
    gpio_set_dir(LED2_PIN, GPIO_OUT);

    //Initializing LED3 pin
    gpio_init(LED3_PIN);
    gpio_set_function(LED3_PIN, GPIO_FUNC_PWM);
    gpio_set_dir(LED3_PIN, GPIO_OUT);

    //Initializing button pins
    gpio_init(SW0_PIN);
    gpio_set_dir(SW0_PIN, GPIO_IN);
    gpio_init(SW1_PIN);
    gpio_set_dir(SW1_PIN, GPIO_IN);
    gpio_init(SW2_PIN);
    gpio_set_dir(SW2_PIN, GPIO_IN);
    gpio_pull_up(SW0_PIN);
    gpio_pull_up(SW1_PIN);
    gpio_pull_up(SW2_PIN);


    // Interrupts
    gpio_set_irq_enabled_with_callback(OPTOFORK_PIN, GPIO_IRQ_EDGE_FALL, true, &generic_irq_callback);
    gpio_set_irq_enabled(OPTOFORK_PIN, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIEZO_PIN, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_callback(&generic_irq_callback);

    stdio_init_all();

    // Create the timer
    repeating_timer_t timer;

    // Negative timeout means exact delay (rather than delay between callbacks)
    if (!add_repeating_timer_us(-500000, timer_callback, NULL, &timer)) {
        printf("Failed to add timer\n");
        return 1;
    }

    //Configuring PWM for LED1 (D1)
    uint led1_slice_num = pwm_gpio_to_slice_num(LED1_PIN);
    uint led1_channel_num = pwm_gpio_to_channel(LED1_PIN);
    pwm_set_enabled(led1_slice_num, false);
    pwm_config config_led1 = pwm_get_default_config();
    pwm_config_set_clkdiv(&config_led1, PWM_CLKDIV);
    pwm_config_set_wrap(&config_led1, PWM_WRAP);
    pwm_init(led1_slice_num, &config_led1, false);
    pwm_set_chan_level(led1_slice_num, led1_channel_num, 0);
    gpio_set_function(LED1_PIN, GPIO_FUNC_PWM);
    pwm_set_enabled(led1_slice_num, true);

    //Configuring PWM for LED2 (D2)
    uint led2_slice_num = pwm_gpio_to_slice_num(LED2_PIN);
    uint led2_channel_num = pwm_gpio_to_channel(LED2_PIN);
    pwm_set_enabled(led2_slice_num, false);
    pwm_config config_led2 = pwm_get_default_config();
    pwm_config_set_clkdiv(&config_led2, PWM_CLKDIV);
    pwm_config_set_wrap(&config_led2, PWM_WRAP);
    pwm_init(led2_slice_num, &config_led2, false);
    pwm_set_chan_level(led2_slice_num, led2_channel_num, 0);
    gpio_set_function(LED2_PIN, GPIO_FUNC_PWM);
    pwm_set_enabled(led2_slice_num, true);

    //Configuring PWM for LED3 (D3)
    uint led3_slice_num = pwm_gpio_to_slice_num(LED3_PIN);
    uint led3_channel_num = pwm_gpio_to_channel(LED3_PIN);
    pwm_set_enabled(led3_slice_num, false);
    pwm_config config_led3 = pwm_get_default_config();
    pwm_config_set_clkdiv(&config_led3, PWM_CLKDIV);
    pwm_config_set_wrap(&config_led3, PWM_WRAP);
    pwm_init(led3_slice_num, &config_led3, false);
    pwm_set_chan_level(led3_slice_num, led3_channel_num, 0);
    gpio_set_function(LED3_PIN, GPIO_FUNC_PWM);
    pwm_set_enabled(led3_slice_num, true);


    bool isCalibrated = false;
    int pill_dispense_count = 0;
    int turn_count = 0;
    int step = 0;
    int step_count = 0;
    float ave_steps = 0;
    int measurements[3];
    int ledState = 0;

    while (1){

        // Blink LED 1 while waiting
        if (ledTimerFired) {
            if (isCalibrated == false) {
                pwm_set_chan_level(led1_slice_num, led1_channel_num, ledState);
                if (ledState) {
                    ledState = 0;
                } else {
                    ledState = 100;
                }
            }
            ledTimerFired = false;
        }


        // Calibrating
        // Press SW0 button to calibrate
        if (!gpio_get(SW0_PIN)) {
            while (!gpio_get(SW0_PIN)) {
                sleep_ms(50);
            }
            printf("Calibration started\n");
            pwm_set_chan_level(led1_slice_num, led1_channel_num, 0);
            while (isCalibrated == false) {
                while (optoforkTriggered == false) {
                    stepMotor(step % 8);
                    step++;
                    sleep_ms(2);
                }
                printf("Optofork found\n");

                optoforkTriggered = false;

                // Turn 3 times to calibrate
                for (int i = 0; i < 3; i++) {
                    optoforkTriggered = false;
                    step_count = 0;
                    while (optoforkTriggered == false) {
                        stepMotor(step % 8);
                        step++;
                        step_count++;
                        sleep_ms(2);
                    }
                    measurements[i] = step_count;
                }

                // Calculate average steps per revolution
                ave_steps = (measurements[0] + measurements[1] + measurements[2]) / 3;
                isCalibrated = true;

                printf("Average steps: %.2f\n", ave_steps);
                printf("round1 steps: %d\n", measurements[0]);
                printf("round2 steps: %d\n", measurements[1]);
                printf("round3 steps: %d\n", measurements[2]);

                // Turn LED 1 on when calibration is finished
                if (ledTimerFired) {
                    pwm_set_chan_level(led1_slice_num, led1_channel_num, 100);
                }
            }
        }

        // Dispensing pills
        // Press SW1 button to start
        if (!gpio_get(SW1_PIN)) {
            while (!gpio_get(SW1_PIN)) {
                sleep_ms(50);
            }
            // Turn LED 1 off
            if (ledTimerFired) {
                pwm_set_chan_level(led1_slice_num, led1_channel_num, 0);
            }

            if(isCalibrated != true) {
                printf("System is not calibrated.\n");
                continue;
            }
            pillDispensed = false;
            while (turn_count < 7){
                runMotor(1, ave_steps);
                sleep_ms(80);
                if (pillDispensed == true) {
                    pill_dispense_count++;
                    pillDispensed = false;
                    printf("Pill dispensed\n");
                } else if (pillDispensed == false) {
                    // Blink LED 1 five times if no dispense detected
                    for (int i = 0; i <= 5; i++) {
                        pwm_set_chan_level(led1_slice_num, led1_channel_num, 100);
                        sleep_ms(100);
                        pwm_set_chan_level(led1_slice_num, led1_channel_num, 0);
                        sleep_ms(100);
                    }
                    printf("No dispense detected\n");
                }
                turn_count++;
            }
            printf("Number of pills dispensed: %d\n", pill_dispense_count);
            turn_count = 0;
            pill_dispense_count = 0;

            isCalibrated = false;


        }



    }
    cancel_repeating_timer(&timer);


}
