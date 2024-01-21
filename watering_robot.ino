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

#include <Adafruit_MotorShield.h>
#include <String.h>
#include <lorawan.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// initialize stepper motor with 200 steps per revolution on stepper port 1 (M1 and M2)
Adafruit_StepperMotor *motor = AFMS.getStepper(200, 1);

// initialize DC motor on M4
Adafruit_DCMotor *pump = AFMS.getMotor(4);

#define SENSORPIN   35  // the analog pin the IR sensor is connected to
#define IRTHRESHOLD 200 // the threshold below which the IR sensor will trigger the robot to stop

int waterFlag = 0; // a flag to denote when the robot has been commanded to go water a plant
int IRFlag = 0;    // a flag to denote when the IR sensor has been triggered, currently NOT IN USE

static TaskHandle_t xTaskToNotify = NULL;

static void lorawan_on_receive(lorawan_rx_packet_t *packet, lorawan_rx_params_t *params)
{
    if (packet == nullptr || params == nullptr)
    {
        return;
    }
    Serial.print("message received, message port is: ");
    Serial.println(packet->Port);

    // any message on port 1 triggers a water call from the robot
    if (packet->Port == 1)
    {
        waterFlag = 1;
        // sets flag, then wakes processor
        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void lorawan_on_join_request(lorawan_join_params_t *params)
{
    if (params->Status == LORAMAC_HANDLER_ERROR)
    {
        lorawan_join();
    }
    else
    {
        lorawan_class_set(LORAWAN_CLASS_C);
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
        // this switch is needed to ensure it is actually in class C
        lorawan_class_set(LORAWAN_CLASS_A);
        lorawan_class_set(LORAWAN_CLASS_C);
    }

    if (!lorawan_get_join_state())
    {
        lorawan_join();
    }
}

void drive()
{
    digitalWrite(17, HIGH);
    Serial.println("drive called");

    int loopTimeout = millis();
    while (1 && millis() - loopTimeout < 100000)
    {
        motor->step(1, BACKWARD, SINGLE);

        // Serial.println(analogRead(SENSORPIN));
        if (analogRead(SENSORPIN) < IRTHRESHOLD /*|| IRFlag */)
        {
            Serial.print("IR sensor reading: ");
            Serial.println(analogRead(SENSORPIN));

            motor->release();
            water();
            IRFlag = 0;
            digitalWrite(17, LOW);
            return;
        }
    }
    // if the loop times out after 100 seconds
    Serial.println("drive failed");
    motor->release();
    digitalWrite(17, LOW);
}

void water()
{
    Serial.println("watering");
    pump->run(FORWARD);
    delay(10000);

    pump->run(RELEASE);
    Serial.println("done watering");
}

void button0_handler()
{
    waterFlag = 1;
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IR_handler()
{
    detachInterrupt(SENSORPIN);
    IRFlag = 1;
}

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ;
    Serial.println("Serial started");

    // initialize motor shield
    if (!AFMS.begin())
    {
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1)
            ;
    }

    motor->setSpeed(500); // rpm for stepper
    pump->setSpeed(255);  // max speed for DC motor

    pinMode(SENSORPIN, INPUT);

    pinMode(17, OUTPUT);
    pinMode(16, INPUT); // testing button (closest to pin 35 header)

    attachInterrupt(16, button0_handler, FALLING); // to wake from sleep and call drive()

    lorawan_setup();

    Serial.print("Join state: ");
    Serial.println(lorawan_get_join_state());
}

void loop()
{
    // wait to receive a message or override with button0 press
    xTaskToNotify = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, 1800000); //30 minutes per cycle

    if (waterFlag)
    {
        drive();
        waterFlag = 0;
    }
}
