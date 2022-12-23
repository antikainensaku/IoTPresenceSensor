#include "esphome.h"
#include "Arduino.h"
#include <queue>
#include "esp_timer.h"

#define pin_ldr_1 34
#define pin_ldr_2 35

using namespace std;

class LaserSensor : public Component, public BinarySensor {
    public:
        // statemachine variables for laser #1
        unsigned short int state_ldr1 = 0;
        bool release_ldr1 = false;
        bool hold_ldr1 = false;
        unsigned short int value_ldr1 = 0;
        unsigned long int t_ldr1 = 0;
        unsigned long int t_0_ldr1 = 0;
        unsigned long int t_ldr1_timeout = 0;
        unsigned long int t_0_ldr1_timeout = 0;

        // statemachine variables for laser #2
        unsigned short int state_ldr2 = 0;
        bool release_ldr2 = false;
        bool hold_ldr2 = false;
        unsigned short int value_ldr2 = 0;
        unsigned long int t_ldr2 = 0;
        unsigned long int t_0_ldr2 = 0;
        unsigned long int t_ldr2_timeout = 0;
        unsigned long int t_0_ldr2_timeout = 0;

        // statemachine variables for presence detection
        unsigned short int state_presence = 0;
        unsigned short int persons = 0;
        unsigned long int t_timer = 0;
        unsigned long int t_0_timer = 0;
        const unsigned short int timeout = 2000; // reset time in milliseconds
        unsigned long int t_multi = 0;
        unsigned long int t_0_multi = 0;
        const unsigned short int multi_threshold = 500; // time in ms before next person is calculated to go in or out of a room

        // statemachine variables for interval
        unsigned short int state_interval = 0;
        unsigned long int t_interval = 0;
        unsigned long int t_0_interval = 0;
        const unsigned short int interval = 500; // how many ms between an interval

        // statemachine variables for debugger
        unsigned short int state_debugger = 0;
        unsigned long int t_debugger = 0;
        unsigned long int t_0_debugger = 0;
        const unsigned short int debugger_interval = 5000; // how many ms between an interval

        // other variables
        const unsigned short int debounce_time = 10; // debounce time in milliseconds
        const unsigned short int ldr_timeout = 8000;  // ldr hold timeout in ms
        bool active = false;

        // for calculating moving average
        queue<unsigned short int> ldr1_mean_q;
        queue<unsigned short int> ldr2_mean_q;
        queue<unsigned short int> ldr1_mean_buffer;
        queue<unsigned short int> ldr2_mean_buffer;
        unsigned short int ldr1_mean;
        unsigned short int ldr2_mean;
        unsigned short int ldr1_threshold;
        unsigned short int ldr2_threshold;
        const unsigned short int queue_length = 5;

        bool first = false;
        bool second = false;

        void setup() override {
            // This will be called by App.setup()
            pinMode(pin_ldr_1, INPUT);
            pinMode(pin_ldr_2, INPUT);
        }
        void loop() override {
            // This will be called by App.loop()
            SM_s1();
            SM_s2();
            SM_presence();
            SM_interval();
            Debugger();
            if (persons > 0)
            {
                lightSwitch(true);
            }
            else
            {
                lightSwitch(false);
            }
        }

        void SM_interval() 
        {
            switch(state_interval)
            {
            case 0: // reset
                if (!hold_ldr1) {
                    average_ldr1();
                }
                if (!hold_ldr2) {
                    average_ldr2();
                }
                state_interval = 1;
                break;
            
            case 1: // start timer
                t_0_interval = esp_timer_get_time() / 1000; // current milliseconds timestamp
                state_interval = 2;
                break;
            
            case 2: // wait for interval time to pass
                t_interval = esp_timer_get_time() / 1000;
                if (t_interval - t_0_interval >= interval) {
                    state_interval = 0;
                }
                break; 
            }
        }

        void Debugger() 
        {
            switch(state_debugger)
            {
            case 0: // reset
                state_debugger = 1;
                break;
            
            case 1: // start timer
                t_0_debugger = esp_timer_get_time() / 1000; // current milliseconds timestamp
                state_debugger = 2;
                break;
            
            case 2: // wait for interval time to pass
                t_debugger = esp_timer_get_time() / 1000;
                if (t_debugger - t_0_debugger >= debugger_interval) {
                    state_debugger = 3;
                }
                break;

            case 3: // print
                // debugging code
                ESP_LOGD("custom", "LDR1 Value: %hu", value_ldr1);
                ESP_LOGD("custom", "LDR2 Value: %hu", value_ldr2);
                ESP_LOGD("custom", "LDR1 mean: %hu", ldr1_mean);
                ESP_LOGD("custom", "LDR2 mean: %hu", ldr2_mean);
                ESP_LOGD("custom", "Persons: %hu", persons);

                state_debugger = 0; // back to reset
                break;
            }   
        }

        void average_ldr1() {
            unsigned int ldr1_sum = 0;
            // keep removing the first (oldest) value
            // while pushing a new value to the tail
            if (ldr1_mean_q.size() >= queue_length) {
                ldr1_mean_q.pop();
            }
            ldr1_mean_q.push(value_ldr1);
            // copies ldr queues to buffer queues
            ldr1_mean_buffer = ldr1_mean_q;
            // gets the sum of ldr values in the queues
            while (!ldr1_mean_buffer.empty()) {
                ldr1_sum += ldr1_mean_buffer.front();
                ldr1_mean_buffer.pop();
            }
            // counts the averages to global variables
            ldr1_mean = (unsigned short int) (ldr1_sum / ldr1_mean_q.size());
            // get threshold value
            ldr1_threshold = (unsigned short int) ldr1_mean / 2;
        }

        void average_ldr2() {
            unsigned int ldr2_sum = 0;
            // keep removing the first (oldest) value
            // while pushing a new value to the tail
            if (ldr2_mean_q.size() >= queue_length) {
                ldr2_mean_q.pop();
            }
            ldr2_mean_q.push(value_ldr2);
            // copies ldr queues to buffer queues
            ldr2_mean_buffer = ldr2_mean_q;
            // gets the sum of ldr values in the queues
            while (!ldr2_mean_buffer.empty()) {
                ldr2_sum += ldr2_mean_buffer.front();
                ldr2_mean_buffer.pop();
            }
            // counts the averages to global variables
            ldr2_mean = (unsigned short int) (ldr2_sum / ldr2_mean_q.size());
            // get threshold value
            ldr2_threshold = (unsigned short int) ldr2_mean / 2;
        }

        void lightSwitch(bool on)
        {
            if (on && !active)
            {
                active = true;
                ESP_LOGD("custom", "Lights on!");
                publish_state(active);
            }
            else if (!on && active)
            {
                active = false;
                ESP_LOGD("custom", "Lights off!");
                publish_state(active);
            }
        }

        void SM_s1()
        {
            switch (state_ldr1)
            {
            case 0: // reset
                state_ldr1 = 1;
                break;

            case 1: // start
                value_ldr1 = analogRead(pin_ldr_1);
                if (value_ldr1 < ldr1_threshold)
                {   
                    state_ldr1 = 2;
                }
                break;

            case 2:
                t_0_ldr1 = esp_timer_get_time() / 1000; // current milliseconds timestamp
                state_ldr1 = 3;
                break;

            case 3: // debounce check
                value_ldr1 = analogRead(pin_ldr_1);
                if (value_ldr1 > ldr1_threshold) 
                {
                    state_ldr1 = 0; // going back to case 0 (reset) if laser no more tripped
                }
                t_ldr1 = esp_timer_get_time() / 1000; // second timestamp
                if (t_ldr1 - t_0_ldr1 >= debounce_time)
                {
                    hold_ldr1 = true;
                    t_0_ldr1_timeout = esp_timer_get_time() / 1000;
                    state_ldr1 = 4;
                }
                break;

            case 4: // armed
                value_ldr1 = analogRead(pin_ldr_1);
                t_ldr1_timeout = esp_timer_get_time() / 1000;
                if (value_ldr1 > ldr1_threshold) 
                {
                    state_ldr1 = 5;
                }
                // if sensor is "on hold" for too long, timeout and reset
                else if (t_ldr1_timeout - t_0_ldr1_timeout >= ldr_timeout) {
                    hold_ldr1 = false;
                    state_ldr1 = 0;
                    ESP_LOGD("custom", "'hold' -timeout, ldr1");
                }
                break;

            case 5: // released
                release_ldr1 = true;
                hold_ldr1 = false;
                state_ldr1 = 0;
                ESP_LOGD("custom", "Laser 1 tripped");
                break;
            }
        }

        void SM_s2()
        {
            switch (state_ldr2)
            {
            case 0: // reset
                state_ldr2 = 1;
                break;

            case 1: // start
                value_ldr2 = analogRead(pin_ldr_2);
                if (value_ldr2 < ldr2_threshold)
                {
                    state_ldr2 = 2;
                }
                break;

            case 2:
                t_0_ldr2 = esp_timer_get_time() / 1000; // current milliseconds timestamp
                state_ldr2 = 3;
                break;

            case 3: // debounce check

                value_ldr2 = analogRead(pin_ldr_2);
                if (value_ldr2 > ldr2_threshold)
                {
                    state_ldr2 = 0; // going back to case 0 (reset) if laser no more tripped
                }
                t_ldr2 = esp_timer_get_time() / 1000; // second timestamp
                if (t_ldr2 - t_0_ldr2 >= debounce_time)
                {
                    hold_ldr2 = true;
                    t_0_ldr2_timeout = esp_timer_get_time() / 1000;
                    state_ldr2 = 4;
                }
                break;

            case 4: // armed
                value_ldr2 = analogRead(pin_ldr_2);
                t_ldr2_timeout = esp_timer_get_time() / 1000;
                if (value_ldr2 > ldr2_threshold)
                {
                    state_ldr2 = 5;
                }
                // if sensor is "on hold" for too long, timeout and reset
                else if (t_ldr2_timeout - t_0_ldr2_timeout >= ldr_timeout) {
                    hold_ldr2 = false;
                    state_ldr2 = 0;
                    ESP_LOGD("custom", "'hold' -timeout, ldr2");
                }
                break;

            case 5: // released
                release_ldr2 = true;
                hold_ldr2 = false;
                state_ldr2 = 0;
                ESP_LOGD("custom", "Laser 2 tripped");
                break;
            }
        }

        void SM_presence()
        {
            switch (state_presence)
            {
            case 0: // reset
                state_presence = 1;
                break;

            case 1: // start
                if (release_ldr1 == true && release_ldr2 == false)
                {
                    t_0_timer = esp_timer_get_time() / 1000; // current milliseconds timestamp
                    state_presence = 2;
                }
                else if (release_ldr1 == false && release_ldr2 == true)
                {
                    t_0_timer = esp_timer_get_time() / 1000; // current milliseconds timestamp
                    state_presence = 3;
                }
                else {
                    release_ldr1 = false;
                    release_ldr2 = false;
                }
                break;

            case 2: // first laser tripped
                if (release_ldr2 == true)
                {
                    state_presence = 4;
                }
                t_timer = esp_timer_get_time() / 1000;
                if (t_timer - t_0_timer >= timeout)
                {
                    release_ldr1 = false;
                    state_presence = 0;
                }
                break;

            case 3: // second laser tripped
                if (release_ldr1 == true)
                {
                    state_presence = 5;
                }
                t_timer = esp_timer_get_time() / 1000;
                if (t_timer - t_0_timer >= timeout)
                {
                    release_ldr2 = false;
                    state_presence = 0;
                }
                break;

            case 4: // person entered
                persons++;
                state_presence = 6;
                break;

            case 5:              // person left
                if (persons > 0) // prevents going below 0
                {
                    persons--;
                }
                state_presence = 6;
                break;

            case 6: // person "debounce" timer
                t_0_multi = esp_timer_get_time() / 1000;
                state_presence = 7;
                break;

            case 7:
                t_multi = esp_timer_get_time() / 1000;
                if (t_multi - t_0_multi >= multi_threshold) {
                    state_presence = 8;
                }
                break;

            case 8: // variable reset
                release_ldr1 = false;
                release_ldr2 = false;
                state_presence = 0;
                break;
            }
        }
};
