# GPIO on Iot2050

![](../../image/iot2050-gpio-board.png)

## Setup

GPIOs are available on the Iot2050. These are routed to the outside via a breakout board. To be able to use these, they must be configured using the **iot2050setup** tool. To do so, please execute following command on the robot:

```bash
sudo iot2050setup
```

It will open following screen. 

![](../../image/iot2050setup-gpio-select-peripherals.png)

Please select **Peripherals** and press enter. It opens the **Peripherals** setup menu.

![](../../image/iot2050setup-gpio-select-io.png)

Please select **Configure Arduino I/O**. Now the current GPIO configuration of the Iot2050 is listed.

![](../../image/iot2050setup-gpio-overview.png)

This tables show the current configuration. The left column shows the name of the GPIO pin (IOx corresponds to Dx on the board). One column right it shows the current configuration of the GPIO pin. The next three columns show the possible configurations:

* **GPIO_Input** means the pin could be an input,
* **GPIO Output** means the pin could be an output and
* the last column shows the special functionality of the GPIO pin like PWM

Below we show how to setup an GPIO pin to be an input, output or an PWM output. At the moment the pin functionality is limited to these three.

### Input and Output

For configuring an digital input or output, please select **Enable GPIO**. Following screen shows up:

![](../../image/iot2050setup-configure-gpio.png)

All GPIOs are listed on top. Using the bottom line a GPIO pin could be configured. With the **Gpio** input filed a GPIO pin can be selected. The **Direction** field allows to configure the GPIO pin as **Input** or **Output**. The last field the **Pull-Mode** allows to enable an resistor. Following modes are exist:

* Hiz, no resistor activated. Input is floating.
* Pull-Up, resistor to V+ is activated.
* Pull-Down, resistor to GND is activated.

>**Note:** an activated resistor makes no sense, when GPIO pin is configured as **Output**!

Select according your needs and confirm it by pressed the **OK** button on the right. The new configuration should be shown on the table above.

### PWM Output

For configuring an PWM output, please select **Enable PWM on IO4-IO9**. Following screen shows up:

![](../../image/iot2050setup-configure-pwm.png)

By pressing the space bar a PWM output can be activated or deactivated. Confirm your choice by pressing **OK**.

### Configuring EduArt control software for controlling GPIOs

EduArt provides a way to address the GPIOs via ROS2 Topics. The EduArt robots come with a preconfigured GPIO setting. This can be found under the path **edu_robot/docker/iot2050/launch_content**. Among other things, the files eduard.urdf and eduard_ros2_control_iot2050.yaml are contained there. These two files describe how ROS has to address the GPIOs. A plugin provided by EduArt then maps these commands to the hardware.

