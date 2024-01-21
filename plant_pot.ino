/*
 *  BSD 3-Clause License
 *
 * Copyright (c) 2024, Northern Mechatronics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "Adafruit_seesaw.h"
#include "lorawan.h"
#include <Arduino.h>

Adafruit_seesaw ss;

#define CAPTHRESHOLD \
    700 // Capacitive threshold for the soil sensor. Anything below triggers a water request

int flag = 0;

static TaskHandle_t xTaskToNotify = NULL;

static void lorawan_on_receive(lorawan_rx_packet_t *packet, lorawan_rx_params_t *params)
{
}

static void lorawan_on_join_request(lorawan_join_params_t *params)
{
    if (params->Status == LORAMAC_HANDLER_ERROR)
    {
        lorawan_join();
    }
    else
    {
        lorawan_class_set(LORAWAN_CLASS_A);
    }
}

static void lorawan_on_wake(void)
{
}

static void lorawan_on_sleep(void)
{
}

static void lorawan_setup()
{
    // LoRa enable is on pin 18 on old petal core, 10 on new
    pinMode(10, OUTPUT);
    digitalWrite(10, LOW);
    pinMode(18, OUTPUT);
    digitalWrite(18, LOW);

    lorawan_enable();

    lorawan_tracing_set(1);

    lorawan_network_config(LORAWAN_REGION_US915, LORAWAN_DATARATE_0, true, true);

    lorawan_activation_config(LORAWAN_ACTIVATION_OTAA, NULL);
    lorawan_key_set_by_str(LORAWAN_KEY_JOIN_EUI, "b4c231a359bc2e3d");
    lorawan_key_set_by_str(LORAWAN_KEY_APP, "01c3f004a2d6efffe32c4eda14bcd2b4");
    lorawan_key_set_by_str(LORAWAN_KEY_NWK, "3f4ca100e2fc675ea123f4eb12c4a012");

    lorawan_event_callback_register(LORAWAN_EVENT_RX_DATA,
                                    (lorawan_event_callback_t)lorawan_on_receive);
    lorawan_event_callback_register(LORAWAN_EVENT_JOIN_REQUEST,
                                    (lorawan_event_callback_t)lorawan_on_join_request);

    lorawan_event_callback_register(LORAWAN_EVENT_SLEEP,
                                    (lorawan_event_callback_t)lorawan_on_sleep);
    lorawan_event_callback_register(LORAWAN_EVENT_WAKE, (lorawan_event_callback_t)lorawan_on_wake);

    lorawan_stack_state_set(LORAWAN_STACK_WAKE);

    if (lorawan_get_join_state())
    {
        lorawan_class_set(LORAWAN_CLASS_A);
    }

    if (!lorawan_get_join_state())
    {
        lorawan_join();
    }
}

void button0_handler()
{
    flag = 1;

    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void setup()
{

    // need to set pin 30 to high to enable gpio
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(30, OUTPUT);
    digitalWrite(30, HIGH);
    // IR beacon power is on pin 2 on old env petal, 0 on new
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);
    pinMode(0, OUTPUT);
    digitalWrite(0, LOW);
    Serial.begin(115200);

    pinMode(16, INPUT);

    // start plant sensor
    if (!ss.begin(0x36))
    {
        Serial.println("ERROR! seesaw not found");
        while (1)
            delay(1);
    }
    else
    {
        Serial.print("seesaw started! version: ");
        Serial.println(ss.getVersion(), HEX);
    }

    attachInterrupt(16, button0_handler, FALLING); // button override to trigger a water request

    lorawan_setup();
}

void loop()
{
    digitalToggle(LED_BUILTIN);
    xTaskToNotify = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, 14400000); //check plant every 4 hours

    // every 10 seconds, check plant status
    if (flag || ss.touchRead(0) < CAPTHRESHOLD)
    { // right now triggers if read value is > threshold, will change to < in final version, this is just to enable testing
        Serial.println("transmitting");
        static uint8_t data[12] = {'I', ' ', 'a', 'm', ' ', 't', 'h', 'i', 'r', 's', 't', 'y'};
        lorawan_transmit(1, 1, 12, data);
        // set beacon on for 100 seconds to stop the cart
        digitalWrite(2, HIGH);
        digitalWrite(0, HIGH);
        delay(30000);
        //30 seconds is tailored for the circular track in the demo. Realistically this could be much larger, but
        //  in order to have the beacon turn off before sending the robot to the next plant in the demo, it needed
        //  to be relatively small
        digitalWrite(2, LOW);
        digitalWrite(0, LOW);
        flag = 0;
    }
}
