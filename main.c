#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "uart.h"
#include "hardware/i2c.h"

// Pin definitions
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

// I2C - EEPROM
#define I2C_BAUDRATE 100000
#define DEVICE_ADDR 0x50
#define EEPROM_SIZE 0x8000
#define I2C1_SDA_PIN 14
#define I2C1_SCL_PIN 15

// Device state memory addresses
uint16_t pillDispensedCount_addr = EEPROM_SIZE - 11;
uint16_t averageSteps_addr = EEPROM_SIZE - 10; // this is a larger, 16 bit value
uint16_t turnCount_addr = EEPROM_SIZE - 5;
uint16_t isCalibrated_addr = EEPROM_SIZE - 4;
uint16_t isTurning_addr = EEPROM_SIZE - 3;
uint16_t isCalibrated_addr_inverted = EEPROM_SIZE - 2;
uint16_t isTurning_addr_inverted = EEPROM_SIZE - 1;

// Using inverse states for safety check
typedef struct deviceState {
    uint8_t state;
    uint8_t not_state;
} deviceState;


// Global variables
volatile bool ledTimerFired = false;
volatile bool optoforkTriggered = false;
volatile bool pillDispensed = false;
deviceState isTurning;
deviceState isCalibrated;
static int global_step_count;
static uint16_t ave_steps;
static int turn_count;
static int pill_dispense_count;
const uint MOTOR_PINS[] = {MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4};
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


// Function declarations
void create_led(uint led);
void adjust_bright(uint led, int bright);
void create_button(uint button);
void generic_irq_callback(uint gpio, uint32_t events);
void stepMotor(int step);
void runMotor(int fraction, int average_steps);
bool turn_motor_backwards();
void configure_motor_pins();
void configure_piezo();
void configure_opto_fork();
bool timer_callback(repeating_timer_t *rt);
void write_memory(uint16_t memory_address, uint8_t data);
uint8_t read_memory(uint16_t memory_address);
void uint16_write_memory(uint16_t addr, uint16_t data);
uint16_t uint16_read_memory(uint16_t memory_address);

int main() {
    configure_opto_fork(); //Initializing Optical Fork pins
    configure_piezo(); //Initializing piezoelectric sensor
    configure_motor_pins(); //Initializing stepper motor pins

    create_led(LED1_PIN);
    create_led(LED2_PIN);
    create_led(LED3_PIN);

    create_button(SW0_PIN);
    create_button(SW1_PIN);
    create_button(SW2_PIN);


    // Interrupts
    gpio_set_irq_enabled_with_callback(OPTOFORK_PIN, GPIO_IRQ_EDGE_FALL, true, &generic_irq_callback);
    gpio_set_irq_enabled_with_callback(PIEZO_PIN, GPIO_IRQ_EDGE_FALL, true, &generic_irq_callback);


    stdio_init_all();

    // Create the timer
    repeating_timer_t timer;

    // Negative timeout means exact delay (rather than delay between callbacks)
    if (!add_repeating_timer_us(-500000, timer_callback, NULL, &timer)) {
        printf("Failed to add timer\n");
        return 1;
    }

    // Initializing I2C HW block and buffer
    i2c_init(i2c1, I2C_BAUDRATE);
    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    sleep_ms(5000);
    
    printf("Booting\n");

    // Reading device states from memory
    isCalibrated.state = read_memory(isCalibrated_addr);
    isCalibrated.not_state = read_memory(isCalibrated_addr_inverted);
    isTurning.state = read_memory(isTurning_addr);
    isTurning.not_state = read_memory(isTurning_addr_inverted);
    turn_count = read_memory(turnCount_addr);
    ave_steps = uint16_read_memory((averageSteps_addr));
    pill_dispense_count = read_memory(pillDispensedCount_addr);
    printf("Turning stat is %d, negated is %d\n", isTurning.state, isTurning.not_state);
    printf("Calibrated stat is %d, negated is %d\n", isCalibrated.state, isCalibrated.not_state);
    if(isCalibrated.state == isCalibrated.not_state) {
        printf("Stored calibrated state is invalid. Setting it for recalibrating.\n");
        isCalibrated.state = 0;
        isCalibrated.not_state = 1;
        write_memory(isCalibrated_addr, 0);
        write_memory(isCalibrated_addr_inverted, 1);
    }
    if(isTurning.state == isTurning.not_state) {
        printf("Stored turning state is invalid. Setting it to default.\n");
        isTurning.state = 0;
        isTurning.not_state = 1;
        write_memory(isTurning_addr_inverted, 1);
        write_memory(isTurning_addr, 0);
    }


    bool dispenserEmpty;
    int step = 0;
    int step_count = 0;
    int measurements[3];
    int ledState = 0;


    while (1){

        // Check if device was powered off while turning
        if (isTurning.state == 1) {
            printf("Device powered off while turning. Turning back to starting point.\n");
            if (turn_motor_backwards()) {
                printf("Dispenser turned back to starting point.\n");
                turn_count = 0;
                global_step_count = 0;
                step = 0;
                step_count = 0;
                optoforkTriggered = false;
                write_memory(turnCount_addr, turn_count);
            }
            if (ave_steps < 4090) {
                printf("Average step count is invalid. Need to recalibrate.\n");
                isCalibrated.state = 0;
                isCalibrated.not_state = 1;
                write_memory(isCalibrated_addr, 0);
                write_memory(isCalibrated_addr_inverted, 1);
            }
            isTurning.state = 0;
            isTurning.not_state = 1;
            write_memory(isTurning_addr, 0);
            write_memory(isTurning_addr_inverted, 1);
        }
        // Turn on LED 1 for standby
        if (ledTimerFired) {
            adjust_bright(LED1_PIN, 100);
        }

        // Blink LED 1 while waiting
        if (ledTimerFired) {
            if (isCalibrated.state == 0) {
                adjust_bright(LED1_PIN, ledState);
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
            if(isCalibrated.state == 1 && ave_steps > 4090){
                printf("System is already calibrated.\n");
            } else {
                printf("Calibration started\n");
                isCalibrated.state = 0;
                isCalibrated.not_state = 1;
                write_memory(isCalibrated_addr, 0);
                write_memory(isCalibrated_addr_inverted, 1);
            }
            adjust_bright(LED1_PIN, 0);
            while (isCalibrated.state == 0) {
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
                for (int i = 0; i < 168; ++i) { // align the holes
                    stepMotor(i % 8);
                    sleep_ms(5); // just to make it move different
                }

                // Calculate average steps per revolution
                ave_steps = (measurements[0] + measurements[1] + measurements[2]) / 3;
                printf("Average steps: %d\n", ave_steps);
                uint16_write_memory(averageSteps_addr, ave_steps);

                isCalibrated.state = 1;
                isCalibrated.not_state = 0;
                write_memory(isCalibrated_addr, 1);
                write_memory(isCalibrated_addr_inverted, 0);

                // Turn LED 1 on when calibration is finished
                if (ledTimerFired) {
                    adjust_bright(LED1_PIN, 100);
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
                adjust_bright(LED1_PIN, 0);
            }

            if(isCalibrated.state == 0) {
                printf("System is not calibrated.\n");
                continue;
            }

            pillDispensed = false;
            dispenserEmpty = false;
            turn_count = 0;
            write_memory(turnCount_addr, turn_count);
            while (turn_count < 8){
                pillDispensed = false;
                runMotor(1, ave_steps);
                isTurning.state = 0;
                isTurning.not_state = 1;
                sleep_ms(80);
                if (pillDispensed == true) {
                    pill_dispense_count++;
                    write_memory(pillDispensedCount_addr, pill_dispense_count);
                    pillDispensed = false;
                    printf("Pill dispensed\n");
                } else if (pillDispensed == false) {
                    // Blink LED 1 five times if no dispense detected
                    for (int i = 0; i <= 5; i++) {
                        adjust_bright(LED1_PIN, 100);
                        sleep_ms(100);
                        adjust_bright(LED1_PIN, 0);
                        sleep_ms(100);
                    }
                    printf("No dispense detected\n");
                }
                turn_count++;
                write_memory(turnCount_addr, turn_count);
                //sleep_ms(30000); // dispense every 30 seconds
                sleep_ms(3000); // dispense every 3 seconds
            }
            if (pill_dispense_count == 7) {
                dispenserEmpty = true;
                printf("Dispenser is empty.\n");
            }
            printf("Number of pills dispensed: %d\n", pill_dispense_count);

            pill_dispense_count = 0;
            write_memory(pillDispensedCount_addr, pill_dispense_count);
        }

        // Resetting memory and global variables
        // Press SW2 button to reset
        if (!gpio_get(SW2_PIN)) {
            while (!gpio_get(SW2_PIN)) {
                sleep_ms(50);
            }

            isCalibrated.state = 0;
            isCalibrated.not_state = 1;
            isTurning.state = 0;
            isTurning.not_state = 1;
            write_memory(isCalibrated_addr, 0);
            write_memory(isCalibrated_addr_inverted, 1);
            write_memory(isTurning_addr, 0);
            write_memory(isTurning_addr_inverted, 1);

            turn_count = 0;
            global_step_count = 0;
            step = 0;
            step_count = 0;
            optoforkTriggered = false;
            pillDispensed = false;
            printf("Memory and globals are cleared!\n");

        }
    }
    cancel_repeating_timer(&timer);


}

// Function definitions

// Function to write int to memory
void uint16_write_memory(uint16_t memory_address, uint16_t data) {
    uint8_t buffer[4];
    buffer[0] = memory_address >> 8;
    buffer[1] = memory_address & 0xFF;
    buffer[2] = data >> 8;
    buffer[3] = data & 0xFF;
    i2c_write_blocking(i2c1, DEVICE_ADDR, buffer, 4, false);
    sleep_ms(5);
}

// Timer for slow LED blink
bool timer_callback(repeating_timer_t *rt) {
    ledTimerFired = true;
    return true; // keep repeating
}

void create_led(uint led){
    uint period, freq;
    period = 1000;
    freq = 125; //Division of 125MHz / 1MHz
    uint slice_num = pwm_gpio_to_slice_num(led);
    pwm_config config_led = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config_led,freq);
    pwm_config_set_wrap(&config_led,period-1); //Set the PWM period
    pwm_set_enabled(slice_num, led);
    pwm_init(slice_num,&config_led,true);
    gpio_set_function(led,GPIO_FUNC_PWM);
}

void create_button(uint button){
    gpio_init(button);
    gpio_set_dir(button, GPIO_IN);
    gpio_set_pulls(button, true, false);
}

void adjust_bright(uint led, int bright){
    uint slice, channel;
    slice = pwm_gpio_to_slice_num(led);
    channel = pwm_gpio_to_channel(led);
    pwm_set_chan_level(slice,channel,bright);
}

void configure_motor_pins() {
    gpio_init(MOTOR_PIN_1);
    gpio_init(MOTOR_PIN_2);
    gpio_init(MOTOR_PIN_3);
    gpio_init(MOTOR_PIN_4);

    gpio_set_dir(MOTOR_PIN_1, GPIO_OUT);
    gpio_set_dir(MOTOR_PIN_2, GPIO_OUT);
    gpio_set_dir(MOTOR_PIN_3, GPIO_OUT);
    gpio_set_dir(MOTOR_PIN_4, GPIO_OUT);
}

void configure_opto_fork() {
    gpio_init(OPTOFORK_PIN);
    gpio_set_dir(OPTOFORK_PIN, GPIO_IN);
    gpio_pull_up(OPTOFORK_PIN);
}

void configure_piezo() {
    gpio_init(PIEZO_PIN);
    gpio_set_dir(PIEZO_PIN, GPIO_IN);
    gpio_pull_up(PIEZO_PIN);
}

void stepMotor(int step) {
    for (int pin = 0; pin < 4; ++pin) {
        gpio_put(MOTOR_PINS[pin], HALF_STEP_SEQUENCE[step][pin]);
    }
}

void runMotor(int fraction, int averageSteps) {
    isTurning.state = 1;
    isTurning.not_state = 0;
    write_memory(isTurning_addr, 1);
    write_memory(isTurning_addr_inverted, 0);
    int stepsToRun = (averageSteps / 8)*fraction;
    int target_step_count = global_step_count + stepsToRun;

    for (; global_step_count < target_step_count; global_step_count++) {
        stepMotor(global_step_count % 8);
        sleep_ms(2);
    }
}

bool turn_motor_backwards() {
    int step = 5000;
    optoforkTriggered = false;
    int step_count = 0;
    while (optoforkTriggered == false) {
        if (step_count > 4100) {
            return 0;
        }
        stepMotor(step % 8);
        step--;
        step_count++;
        sleep_ms(2);
    }
    for (int i = 100; i > 0; --i) { // align the holes
        stepMotor(i % 8);
        sleep_ms(4); // just to make it move different
    }
    isTurning.state = 0;
    isTurning.not_state = 1;
    write_memory(isTurning_addr, 0);
    write_memory(isTurning_addr_inverted, 1);

    return 1;

}

// Interrupt handler for the opto fork and piezo sensor
void generic_irq_callback(uint gpio, uint32_t event_mask){
    if (gpio == OPTOFORK_PIN && (event_mask & GPIO_IRQ_EDGE_FALL)) {
        optoforkTriggered = true;
    }
    if (gpio == PIEZO_PIN && (event_mask & GPIO_IRQ_EDGE_FALL)) {
        pillDispensed = true;
    }
}

void write_memory(uint16_t memory_address, uint8_t data) {
    uint8_t buffer[3];
    buffer[0] = memory_address >> 8;
    buffer[1] = memory_address & 0xFF;
    buffer[2] = data;
    i2c_write_blocking(i2c1, DEVICE_ADDR, buffer, 3, false);
    sleep_ms(5);
}

uint8_t read_memory(uint16_t memory_address) {
    uint8_t value;
    uint8_t buffer[2];
    buffer[0] = memory_address >> 8;
    buffer[1] = memory_address & 0xFF;
    i2c_write_blocking(i2c1, DEVICE_ADDR, buffer, 2, false);
    sleep_ms(5);
    i2c_read_blocking(i2c1, DEVICE_ADDR, &value, 1, false);
    return value;
}

uint16_t uint16_read_memory(uint16_t memory_address) {
    uint8_t value[2];
    uint16_t returnValue;
    uint8_t buffer[2];
    buffer[0] = memory_address >> 8;
    buffer[1] = memory_address & 0xFF;
    i2c_write_blocking(i2c1, DEVICE_ADDR, buffer, 2, false);
    sleep_ms(5);
    i2c_read_blocking(i2c1, DEVICE_ADDR, value, 2, false);
    returnValue = value[0] << 8;
    returnValue += value[1];
    return returnValue;
}

