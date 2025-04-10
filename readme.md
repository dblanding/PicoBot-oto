# PicoBot with Optical Tracking Odometry
* I have long wondered if optical flow technology could be used to replace wheel odometry but I have never gotten around to undertaking the project. Well, it turns out this sensor was worth the wait because it works very well on the laminated vinyl flooring throughout my house.
* This project describes the use of SparkFun's recently introduced [Optical Tracking Odometry Sensor (OTOS)](https://www.sparkfun.com/sparkfun-optical-tracking-odometry-sensor-paa5160e1-qwiic.html) to
replace the wheel odometry system I have been using, which relies on using motor encoders and an IMU.
* The robot used is the PicoBot described in this [earlier project](https://github.com/dblanding/PicoBot-dev).

![PicoBot](imgs/picobot.jpeg)
![OTOS](imgs/otos.jpeg)

## Power to the Pico
* Power is supplied to the Pico from multiple sources:
    1. Connected to a laptop by USB cable
    2. Powered by an onboard battery
    3. Both USB & battery at once

![Pico Power](imgs/pico_power_bb.png)

* When powered by a 5V source, care must be taken to connect the source to the VSYS pin (pin 39) through a Schottky diode.
* Conveniently, the L298N Motor Driver board provides a 5V power supply.
* If the Pico is connected to the laptop by USB cable, regardless of whether the battery switch is ON or OFF, the voltage at the Vsys pin is just a bit under 5 Volts.
* If the Pico is *not* connected to the laptop, and the battery switch is ON, the voltage at the Vsys pin is still just a bit under 5 V.
* In either case, the **diode** safely resolves any voltage conflicts on the Vsys pin and the Pico remains happily powered, putting out 3.3V on its 3V3 pin.

## PicoBot CAD model
* It's not essential to have a CAD model but it can be helpful when developing and exploring alternative configurations. Here is a link to the [PicoBot CAD model](PicoBotAsy.step), showing some of the critical components in the actual physical PicoBot.

## Mounting the OTO Sensor
* SparkFun recommends the Sensor be set to have a working range of 10 - 27 mm, and that for FTC it be set at exactly 10 mm.
* I have the gap set at about 12 mm on the PicoBot and it seems to work very well.

![OTO Sensor gap](imgs/otos-gap.png)

## Running the code:
* The [robot folder](robot) contains files that are run on the Pico
* The [laptop folder](laptop) contains files that are run on the laptop

* To Operate the PicoBot:
    1. Place the PicoBot in its **Home** pose = (0, 0, 0) on the floor of the arena.
    2. Turn the PicoBot power switch **ON**. This starts the file `main.py` on the robot.
    3. Open the Bluefruit Connect cell phone app.
    
    ![bluefruit connect app](imgs/bluefruit_connect_app.png)
    
    * 2 BLE UART Friend devices will be listed, but you don't neccesarily know which device is on which uart.
        * The one marked **0** is on UART0 and the one marked **1** is on UART1, both with a flashing red LED
    * Connect to the one on **UART0**
        * The connected device will now have a solid blue LED
        * If the UART1 device LED is lit, disconnect and click on the other one.
        * Now, only the device on UART0 has its blue LED on.
    * Next, select **Controller** on the phone app
    * Lay the phone down on a horizontal surface and click **Accelerometer**
    4. On the laptop, run the file `display_from_robot.py`.
    * It's important to have the phone app already connected so the laptop has no other choice but to connect to the device on UART1.
    * Hopefully, a window will be launched with a map of the Arena with some waypoints plotted in Magenta. Also, some buttons will be displayed across the top.
        * Sometimes, an `Error parsing JSON` will occur and the map doesn't get drawn.
        * If this happens, just exit and run the file again.
        * Make sure the phone is connected to the bluetooth module on UART0 and the laptop program is connected to the one on UART1.
    * When the window appears with a map of the Arena (and the waypoints plotted in Magenta), you're ready to proceed to the next step.
    
![Empty Arena Map](imgs/start_map.png)
    
## Driving the car in Tele-Op mode.
* After completing steps 1 through 4 above...
    * Cick on the **Tele** button to enable Tele-op driving mode.
    * Then click **Start**.
        * Now the PicoBot is ready to go. (The motors might sing briefly.)
    * Pick up the phone, being careful to keep it level..
    * The PicoBot is driven and steered by tipping and tilting the phone.
        * Imagine there is a small ball on the face of the phone.
        * Tipping the phone so the ball rolls away from you (toward the top) causes the robot to drive forward.
        * Tilting the phone to the right or left causes the robot to turn right or left.
        * Returning the phone to a level surface causes the robot to stop.
* Colored dots will be drawn on the map as the robot drives.
    * Blue dots show the path the robot has taken.
    * Red dots show the location of objects detected by the left distance sensor.
    * Green dots show the location of objects detected by the right distance sensor.
    * Yellow dots show the location of objects detected by the forward looking sensor.

### Nulling any tendency to veer to the right or left
* Test to make sure that when the phone is carefully tipped forward with no tilting to either side, the robot does indeed drive straight ahead without any tendency to veer to the right or left.
    * A good way to do this is to start with the phone laying flat on a level table top, then carefully lift the bottom edge of the phone, while keeping the entire top edge in contact with the table. 
* You can null out any tendency to veer by adjusting the trim pot connected to the Pico's analog input (Pin 26). This adjustment has the effect of changing the relative magnitude of the signals sent to the R & L motors.

## Driving the car in Auto mode
* Once the veer adjustment is done, the PicoBot can be driven in Auto mode  (automatically following a series of waypoints).
* Start by repeating steps 1 - 4 in the **Running the Code** section above.
* This time, Click on the **Auto** button at the top of the map window.
* Then click **Start**.
* If everything is working perfectly, the robot will proceed to drive through the series of waypoints, mapping any obstacles detected along the way, ultimately arriving at the last waypoint, which happens to be its starting (home) position.

![Loop_run](imgs/loop_map.png)

