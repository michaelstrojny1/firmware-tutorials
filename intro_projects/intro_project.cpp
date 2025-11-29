#include <arduino_freertos.h>
#include <cstddef>
#include <stdint.h>

/*
The car has three possible functional states:
1. Low Voltage (LV)
2. Tractive System (TS)
3. Ready to Drive (RTD)

When in LV, the car cannot drive. When in TS, the car is energized, but still
cannot drive. When in RTD, the car is energized and able to drive.

There are two Accumulator Isolation Relays (AIRs) that connect the high voltage
(HV) battery to the rest of the car. One is on the positive terminal, and the
other on the negative terminal (AIR+ and AIR-). That is to say, the battery is
disconnected unless we energize both relays, which is important for safety.
Whenever we have a critical error, we can open these relays / de-energize them
to make sure there is no HV outside the accumulator.

When we go from LV -> TS, we cannot simply energize the relays (ask an
electrical lead why!). Instead we first close AIR-, then another relay called
the "Precharge Relay". After ~5 seconds we close AIR+ and open the precharge
relay.

There is a TS On button and an RTD button on the dash that move the cars into
those states. ie. When you press TS On the car goes TS, and when you press RTD,
it goes to RTD. They are both active low, meaning when the button's are pressed,
the voltage goes low.

We also have a few sensors that we need to be able to calculate how much torque
to command:

1. Current sensor (How much current there is in the HV path)
2. Wheelspeeds
3. Steering Angle Sensor

The current sensor is a Hall Effect sensor read by an ADC through GPIO pins. The
conversion is: 1 amp per 10mV

Wheelspeeds are a bit tricky. The sensor is routed to a GPIO pin, and is
normally high. However, the wheels have a gear looking thing with 17 teeth --
everytime one of the teeth passes the sensor, the voltage at the GPIO pin goes
low. You can derive the RPM of the wheel from how often the voltage goes low.

The steering angle sensor comes to us via CAN.

We also have to monitor our battery to make sure nothing explodes or catches on
fire... Thankfully we have a Battery Management System (BMS) that monitors cell
voltages and temperatures and can send them to us. It sends this info via SPI.
If any cell voltage goes below 2.8V or above 4.3V, de-energize the car. Also, if
any temperature exceeds 60 degrees, de-energize.

Finally, we have an LCD on the dash which is useful to display information. You
should print all relevant info to the screen so we know what's going on with the
car. This also operates via SPI.

Mock library functions have been provided where necessary (marked with extern).

NOTE - EXTREMELY IMPORTANT: the CAN methods `can_send` and `can_receive` are NOT
thread safe.

Also think about the implications of having two peripherals on one SPI bus.

----------------------------

Requirements:

1. Command each motor with an appropriate torque every 1ms
2. Print sensor values and torque values to the LCD every 100ms
3. Monitor the highest/lowest voltage and highest temperature from the BMS and
shutdown the car if there is any unsafe condition

----------------------------

Pins:

TS On:          12
RTD:            13
Current Sensor: 19
CAN RX:         2
CAN TX:         3
MOSI:           5
MISO:           6
SCK:            7
LCD_CS:         8
BMS_CS:         9
AIR+:           22
Precharge:      23
AIR-:           10

----------------------------

"Submission" instructions. I want you all to get familiar with git, so we will
do this project with git. The repo is
https://github.com/UTFR/firmware-tutorials.

If you still haven't joined the github organization, let me know and i'll add
you. Clone the repository, and make a branch called `<your name>/intro_project`.
Copy this file into the directory `intro_projects/<your name>` and make all your
changes there.

Whenever you add a feature, `git add` the files, and `git commit -m "..."` with
a useful/descriptive message. Whenver you want your changes to be public, do
`git push origin <your name>/intro_project`.

Also, I won't enforce it for this project, but for the actual firmware repo we
have a code formatter (clang-format). It means that everyone's code will look
exactly the same, in terms of how much whitespace there is and other aesthetics
like that. It's not there for aesthetics, but moreso so that you when people
make changes you can see exactly what they changed, whereas without a formatter,
you end up with lots of useless formatting changes. You should install
clang-format as soon as possible and set it up :)
*/

/*
  Initialize the CAN peripheral with given RX and TX pins at a given baudrate.
*/
extern void can_init(uint8_t rx, uint8_t tx, uint32_t baudrate);

/*
  Send a CAN message with a given id.
  The 8 byte can_msg_trq is encoded as a uint64_t
*/
extern void can_send(uint8_t id, uint64_t can_msg_trq);
/*
  Receive a CAN message with a given id into a uint64_t
*/
extern void can_receive(uint64_t *can_msg_trq, uint8_t id);

/*
  Calculates four torques, in order, for the Front Left, Front Right, Rear Left,
  and Rear Right motors given current, wheelspeeds (in the same order), and a
  steering angle.
*/
extern void calculate_torque_cmd(
  float *torques, float current, float *wheelspeeds, float steering_angle
);

/*
  Initialize the LCD peripheral
*/
extern void lcd_init(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t lcs_cs);

/*
  Print something to the LCD
*/
extern void lcd_printf(const char *fmt, ...);

/*
  Initialize the BMS
*/
extern void bms_init(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t lcs_cs);

/*
  Get voltage of the nth cell in the battery
*/
extern float bms_get_voltage(uint8_t n);

/*
  Get temperature of the nth cell in the battery
*/
extern float bms_get_temperature(uint8_t n);

#define TS_ON     12
#define RTD_BTN   13
#define CURRENT   19
#define CAN_RX    2
#define CAN_TX    3
#define MOSI      5
#define MISO      6
#define SCK       7
#define LCD_CS    8
#define BMS_CS    9
#define AIR_POS   22
#define PRECHARGE 23
#define AIR_NEG   10

// wheel speed sensors . Wheel 1,2,3,4
#define W1       34
#define W2       35
#define W3       32
#define W4       33

#define NUM_CELLS 10
#define CAN_ID_STEER  1
#define CAN_ID_TORQUE 2

enum states {lv, precharge, ts, rtd};

// value not cached might change
volatile enum states state = lv;

// globals
float curr;
float rpms[4];
float steer;
float trqs[4];

// limits
float v_min, v_max, t_max;

volatile uint32_t wheel_cnts[4] = {0, 0, 0, 0};
. 
// disables interrupts
portMUX_TYPE m = portMUX_INITIALIZER_UNLOCKED;

// For instance, the ++ wheel counter can interupt while we read the current number of ticks & store in cpu reg & calc torque (i.e torq task)
// but, the read opperation read the old count value (n) and will resume with that old value
// so it will ignore the n+1. portmux fixes this.

// only one task can hold the mutex key and others wait. we need 2 of these. we cant both read and write can/spi and we also cant spi/can to multiple places at once
// this is actually a data type so we must define
SemaphoreHandle_t spi_mutex;
SemaphoreHandle_t can_mutex;

// RAM is fast so we use RAM since these are interupts (keeps them short in duration) and also why degrade flash
void IRAM_ATTR w1() { 
    wheel_cnts[0]++; }
void IRAM_ATTR w2() { 
    wheel_cnts[1]++; }
void IRAM_ATTR w3() {
    wheel_cnts[2]++; }
void IRAM_ATTR w4() {
    wheel_cnts[3]++; }

void shutdown() {
    // the safety relays are active high
    digitalWrite(AIR_POS, LOW);
    digitalWrite(AIR_NEG, LOW);
    digitalWrite(PRECHARGE, LOW);
    state = lv;
   
    if(xSemaphoreTake(spi_mutex, 0) == pdTRUE) {  // just grab mutex (wait 0 seconds) if being used by LCD and use for shutdown
        lcd_printf("Shutdown");
        xSemaphoreGive(spi_mutex);  // give it back. now, whicheevr process runs next (with highest priority or waiting longest if priorities equal) gets the key
    }
}

// tasks
void trq(void *p);
void fsm(void *p);
void lcd(void *p);
void bms(void *p);
void can_rx(void *p);

void setup(void) {
    // INPUT_PULLUP activates a resistor connecting the pin to 3.3V so its active low
    pinMode(TS_ON, INPUT_PULLUP);
    pinMode(RTD_BTN, INPUT_PULLUP);

    pinMode(AIR_POS, OUTPUT);
    pinMode(AIR_NEG, OUTPUT);
    pinMode(PRECHARGE, OUTPUT);
    
    // off (default)
    digitalWrite(AIR_POS, LOW);
    digitalWrite(AIR_NEG, LOW);
    digitalWrite(PRECHARGE, LOW);

    // wheel speed. input is normally high and goes low when one of the teeth pass sensor
    pinMode(W1, INPUT_PULLUP);
    pinMode(W2, INPUT_PULLUP);
    pinMode(W3, INPUT_PULLUP);
    pinMode(W4, INPUT_PULLUP);

    // interupt when a wheel tick is detected. I.e when one of the Wn pins falls, run the wheeln function wn
    attachInterrupt(digitalPinToInterrupt(W1), w1, FALLING);
    attachInterrupt(digitalPinToInterrupt(W2), w2, FALLING);
    attachInterrupt(digitalPinToInterrupt(W3), w3, FALLING);
    attachInterrupt(digitalPinToInterrupt(W4), w4, FALLING);

    // create the mutexes
    spi_mutex = xSemaphoreCreateMutex();
    can_mutex = xSemaphoreCreateMutex();

    // init all with correct pins
    can_init(CAN_RX, CAN_TX, 500000);
    lcd_init(MOSI, MISO, SCK, LCD_CS);
    bms_init(MOSI, MISO, SCK, BMS_CS);

    // Create tasks
    // 2480 byte stack per task for temp vars etc
    // trq has highest priority 
    xTaskCreate(trq, "trq", 2048, NULL, 3, NULL);  // pedal
    xTaskCreate(fsm, "FSM", 2048, NULL, 2, NULL);  // what state are we in
    xTaskCreate(can_rx, "can_rx", 2048, NULL, 2, NULL);  // can. buffer can't overflow so check & clear quick
    xTaskCreate(lcd, "lcd", 2048, NULL, 1, NULL);  // display
    xTaskCreate(bms, "bms", 2048, NULL, 1, NULL);  // battery does not change state quickly so we can check this infrequently
}

void loop() {}   // MCU needs this to exist. however, all looping and running is handles by while loops in tasks

// trq has 1khz freq. read sensor, get torque, send to motor
void trq(void *p) {
    // time in ticks
    TickType_t last_time = xTaskGetTickCount();
    // what is 1ms in ticks?
    TickType_t freq = pdMS_TO_TICKS(1);

    while(1) {
        if (state == rtd) {
            uint32_t temp_count[4];

            // count how many ticks per ms. this loop runs every ms

            portENTER_CRITICAL(&m);
            for(int i = 0; i < 4; i++) {
                temp_count[i] = wheel_cnts[i];
                wheel_cnts[i] = 0; // Reset counter for the next ms
            }
            portEXIT_CRITICAL(&m); /

            // Get RPM! conversion factor --> (teeth/1ms)(1000ms/1sec)*(60sec/1min)/17teeth = 60000 / 17
            for(int i = 0; i < 4; i++) {
                rpms[i] = (float)temp_count[i] * (60000.0f / 17.0f);
            }

            // analogRead is 0-4095 where 4095 is 3.3V and 0 is 0V
            // 3300 mV / 4096 converts the analogRead signal to mV
            // dividing by 10 converts mV to Amps
            curr = (float)analogRead(CURRENT) * (3300.0f / 4096.0f) / 10.0f;

            calculate_torque_cmd(trqs, curr, rpms, steer);

            // can message. carries 8 byte. We need to send 4 wheel torques.
            // Each wheel speed is a float. but we convert to 16 bit int.
            // now, we can have 4 wheel speeds in 1 can message
            uint64_t can_msg_trq = 0;
            can_msg_trq |= (uint16_t)((int16_t)trqs[0]);          // make a 64 byte message with trqs[0] at the start
            can_msg_trq |= (uint64_t)((uint16_t)((int16_t)trqs[1])) << 16;   // shift trqs[1] 16 left
            can_msg_trq |= (uint64_t)((uint16_t)((int16_t)trqs[2])) << 32;   // shift trqs[2] 32 left
            can_msg_trq |= (uint64_t)((uint16_t)((int16_t)trqs[3])) << 48;

            // send can (while using mutex) 
            xSemaphoreTake(can_mutex, portMAX_DELAY);
            can_send(CAN_ID_TORQUE, can_msg_trq);
            xSemaphoreGive(can_mutex);
        } 
        else {
            // well, if we aren't driving, its 0
            for(int i=0; i<4; i++) trqs[i] = 0;
        }

        // get the 1ms loop period by waiting 1ms after every loop
        // modify last_time by reference to equal last_time+freq and wait till then
        vTaskDelayUntil(&last_time, freq);
    }
}

// FSM
// 50ms period

void fsm(void *p) {
    while(1) {
        int ts_btn = digitalRead(TS_ON);    // our buttons
        int rtd_btn = digitalRead(RTD_BTN);
        
        if (state == lv) {
            // ts btn is active low
            if (ts_btn == LOW) {   // switch lv --> ts
                state = precharge;
                digitalWrite(AIR_NEG, HIGH);
                digitalWrite(PRECHARGE, HIGH);

                // we wait 5s. But what if we have an interupt like shutdown during that 5s?
                // we split the 5s into increments and check if there was a shutdown 
                // if so, we dont proceed

                for(int i = 0; i < 50; i++) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    if(state != precharge) break; 
                }

                if (state == precharge) {
                    digitalWrite(AIR_POS, HIGH); 
                    digitalWrite(PRECHARGE, LOW); 
                    state = ts; 
                }
            }
        } 
        
        // ts 
        else if (state == ts) {
            if (rtd_btn == LOW) {   // condition for rtd is ts as previous state
                state = rtd; 
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));   // 50ms period
    }
}

// LCD display 
// 100 ms period

void lcd(void *p) {
    TickType_t last_time = xTaskGetTickCount();
    TickType_t freq = pdMS_TO_TICKS(100);

    while(1) {
        // spi also uses the BMS, so mutex used

        xSemaphoreTake(spi_mutex, portMAX_DELAY);   // Wait forever until mutex avalible
        
        lcd_printf("ST: %d\n", state);
        lcd_printf("I: %.1f\n", curr);
        lcd_printf("R: %.0f %.0f %.0f %.0f\n", rpms[0], rpms[1], rpms[2], rpms[3]);
        lcd_printf("T: %.0f %.0f %.0f %.0f\n", trqs[0], trqs[1], trqs[2], trqs[3]);
        lcd_printf("V: %.2f - %.2f\n", v_min, v_max);
        
        xSemaphoreGive(spi_mutex);

        vTaskDelayUntil(&last_time, freq);
    }
}

// BMS 
// 100ms frequency

void bms(void *p) {
    while(1) {
        float min = 1000.0f; // volt
        float max = 0.0f;
        float max_temperature = 0.0f;

        xSemaphoreTake(spi_mutex, portMAX_DELAY);
        // loop through all cells, record worst cell
        for(uint8_t i = 0; i < NUM_CELLS; i++) {
            float v = bms_get_voltage(i);
            float t = bms_get_temperature(i);

            if (v < min) 
            min = v;
            if (v > max) 
            max = v;
            if (t > max_temperature) 
            max_temperature = t;}
        xSemaphoreGive(spi_mutex);

        v_min = min;
        v_max = max;
        t_max = max_temperature;

        if (min < 2.8f || max > 4.3f || max_temperature > 60.0f) {
            shutdown(); }

        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms    // fun fact, the argument is solved during compilation
    }
}

// CAN recieve (rx) // 10ms polling

void can_rx(void *p) {
    while(1) {
        uint64_t rx = 0; 
        
        xSemaphoreTake(can_mutex, portMAX_DELAY);
        can_receive(&rx, CAN_ID_STEER);
        xSemaphoreGive(can_mutex);

        // I assume the angle signal is not the full 8 CAN bytes (int 64) since 64 bits is a huge integer
        // lets assume the signal is in the first 2 bytes
        // lets also assume its scaled up by some factor
        // I'll cast to the first 2 bytes and divide by 256/(2pi) so the range is 0->2pi

        float steer = (float)(2*3.14*(int16_t)(rx))/256;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}