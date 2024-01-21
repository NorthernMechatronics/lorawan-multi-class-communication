# Arduino LoRaWAN Plant-Watering Robot

This project is a technology demonstration showcasing our Arduino Core support on the NM180100 and LoRaWAN Class A and Class C connectivity in Arduino.

## Table of Contents

- [Arduino LoRaWAN Plant-Watering Robot](#arduino-lorawan-plant-watering-robot)
  - [Table of Contents](#table-of-contents)
  - [Hardware](#hardware)
    - [Robot](#for-the-plant-watering-robot)
    - [Plant Pots](#for-the-plant-pots)
  - [Connecting to AWS](#connecting-to-aws)
  - [Software](#software)
  - [How Does it Work?](#how-does-it-work)
    - [Plant Pot](#plant-pot)
    - [AWS](#aws)
    - [Watering Robot](#watering-robot)
    - [Expandability](#expandability)

### Hardware

#### For the Plant-Watering Robot

- [NM180100EVB](https://www.northernmechatronics.com/nm180100evb) available at [TOP Electronics](https://www.top-electronics.com/en/northern-mechatronics)
- [Adafruit Motor Shield V2](https://www.adafruit.com/product/1438)
- A NEMA 17 stepper motor
- [Adafruit Peristaltic Pump](https://www.adafruit.com/product/1150)
- [SparkFun Analog Line Sensor](https://www.sparkfun.com/products/9453)
- [3 AA Battery Holder](https://www.digikey.ca/en/products/detail/keystone-electronics/2465/303814)
- 3 AA Batteries

#### For the Plant Pots

- [NM180400 Petal Core](https://www.northernmechatronics.com/nm180410)
- NM180420 Environment Petal
- NM180430 Power Petal
- [Adafruit Soil Sensor](https://www.adafruit.com/product/4026) and [Cable](https://www.adafruit.com/product/4528)
- [SparkFun Analog Line Sensor](https://www.sparkfun.com/products/9453)
- Any [Lithium Polymer Battery](https://www.adafruit.com/product/258) with a 2-pin JST-PH connector. The one that is linked will fit in the slot under the pot, which is 35mm wide by 10mm tall.

For this project, we put the robot on a circular track with the plant pots on
the outside of the track. This was to keep the robot aligned with the pots, but
there are other ways of doing this. In our design the robot uses its IR sensor
to see another IR sensor attached to the track, to know when to stop at a pot,
but again this is just one way of getting the robot to stop.

#### CAD Models
All CAD models are provided under the CAD directory.  The native CAD models are
in Creo Parametric v8 format (the advanced assembly extension license option is
required to modify the models).  A step file export of the entire assembly is
made available for those without access to Creo Parametric.

### Connecting to AWS

Follow these instructions to get set up with AWS IoT

- [AWS IoT Core Integration](https://github.com/NorthernMechatronics/nmapp2/blob/master/doc/aws_iot_core_integration.md)
- [Adding a LoRaWAN gateway in AWS IoT Core](https://github.com/NorthernMechatronics/nmapp2/blob/master/doc/aws_iot_add_gateway.md)
- [Adding a LoRaWAN device in AWS IoT Core](https://github.com/NorthernMechatronics/nmapp2/blob/master/doc/aws_iot_add_device.md)

### Software

Follow the [<b>Getting Started With Arduino IDE Guide</b>](http://gitlab.northernmechatronics.com:50250/nmi/software/arduino/ArduinoCore-nm180100/-/blob/master/doc/NM180100EVB_User_Guide.md) to get up and running writing code for the NM180100EVB and NM180410 Petal Development Board.

## How Does it Work?

### Plant Pot

The plant pot is connected to AWS IoT through the gateway operating in class A.

The soil sensor in the plant pot works by measuring the capacitance of the medium around it. This means that wetter soil will measure a higher capacitance than dry soil. From this, we can detect when the soil is dry, and use that to determine when the plant needs to be watered.

When the plant needs to be watered, it sends a string "I am thirsty" to the server. Simultaneously, the plant pot turns on an IR beacon, which is attached to the track facing upwards, that shines an infrared light towards the sky. This is to tell the robot where to stop when it arrives.

### AWS

All plant pots were set up in AWS to send messages to the same MQTT topic.  This is specified under [LPWAN devices destination](https://github.com/NorthernMechatronics/nmapp2/blob/master/doc/aws_iot_add_device.md).
The message is then routed to an AWS Lambda function using the Message Routing Rule that performs the downlink transmission to the watering robot.
The Lambda function is written in Python and is shown below:

```
import json
import boto3

def lambda_handler(event, context):

    if event["PayloadData"] == None:
            return "failed"

    client = boto3.client('iotwireless')

    if (event["PayloadData"] == "SSBhbSB0aGlyc3R5"):
        response = client.send_data_to_wireless_device(
            Id='92cf6398-1989-4279-a214-14a9a09516cc',
            TransmitMode=0,
            PayloadData=event["PayloadData"],
            WirelessMetadata={
                'LoRaWAN': {
                    'FPort': 1,
                }
            }
        )

    return response
```

The code has some basic error handling, then passes the message along to the watering robot using the AWS Iot API `send_data_to_wireless_device()`. The listed ID is that of the NM180100EVB inside the robot. Transmit mode 0 means it is sending an unacknowledged message, meaning it expects nothing back saying the message was received. Finally, it is sending on port 1.

### Watering Robot

In the idle state, the watering robot processor goes to sleep, and awaits a message from AWS. The robot is operating as a LoRaWAN Class C LINK HERE device, meaning it is always listening for messages. When it receives one, it checks to make sure the message is on port 1, matching what was sent by AWS, and then triggers the processor to wake up.

The robot starts driving forward until the IR sensor on the bottom detects the beam produced by the plant pot unit. At this point, it has arrived at its destination, and stops. It then runs the pump to water the plant in the pot. After watering is complete, it goes back to the idle state and awaits a new call.

### Expandability

This example works with any number of plant pots along the track. Each plant pot only activates its IR beacon when it needs to be watered, the robot will skip all plant pots along the track that do not have their beacons activated. There is no code modification necessary to add new plant pots to the system. Since any message of "I am thirsty" will trigger the robot to move, adding a new plant pot to the system is as easy as setting it down next to the track, wiring up its IR beacon, and running the plant pot code on it.

### Practical Use Cases

While the primary purpose of this technology demonstration is to showcase how
two different classes of LoRaWAN devices interact with each other in a system,
there are practical agricultural use cases where this is applicable.
For example, this system could be used to implement precision irrigation,
fertilization, herbicide or pesticide application.