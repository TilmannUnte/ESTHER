# ESTHER: Embedded System Theory Evaluation Robot

ESTHER is a demonstrator based on the Ubiquity Robotics Magni robot platform. The demonstrator can be used in lectures, as a case study for research, and as a baseline for individual student projects.
It has proven very popular as a testing ground for attaching new sensors and trying simple robot applications.

# Hardware
The Magni is shipped with a Raspberry Pi running ROS which communicates with the Motor Control Board (MCB) to read out sensor data and send drive commands.
For ESTHER, the Raspberry Pi was removed and replaced with a microcontroller running the Zephyr RTOS.
Zephyr offers a high degree of hardware abstraction, meaning a variety of microcontrollers can be used with the same application code.

The following microcontroller boards are supported currently:
- ST STM32L475 Discovery IOT01 (*disco_l475_iot1*)
- ST STM32F4 Discovery (*stm32f4_disco*)
- ST Nucleo F767ZI (*nucleo_f767zi*)
- WeAct Black Pill v3.0 (*blackpill_f401ce*)
- Infineon XMC4500 (*xmc45_relax_kit*)
- Native POSIX Compatibility Layer (*native_sim*)
- Arduino Portenta (*arduino_portenta_h7_m7*)
- ESP32 DevkitC (*esp32_devkitc_procpu*)
- ESP32C6 DevkitC (*esp32c6_devkitc_hpcore*)
- ST STM32F103 Minimum Development Board (stm32_min_dev@blue)

Of these, the *disco_l475_iot1* board has been used most frequently and extensively. Therefore, it is recommended as the default choice with the highest degree of functionality. It can easily and securely be mounted to the robot chassis with screws.
The *stm32f4_disco* and *nucleo_f767zi* are not easily mounted to the robot chassis, but offer a larger amount of I/O pins, which can be useful.
The *blackpill_f401ce* is annoying to use as it requires a seperate programmer. It also cannot be mounted easily.
The *xmc45_relax_kit* is natively supported by the AbsInt aiT software used for static timing analysis. It does have screw holes, but mounting it to the robot chassis has not been attempted.

The use of *native_sim* is possible, if USB-to-UART adapters are hooked up to the robot and other external hardware, such as the LiDAR.
This allows for a better debugging experience, however the timing of the application is broken and you should expect the application to fail rather quickly.

The Arduino Portenta and ESP32/ESP32C6 DevkitC boards use dual-cores and are in experimental stage, as we have little experience with them!

Many other boards unfortunately will only ever be partially supported, due to inherent hardware limitations.
On small-scale boards, this is often related to a lack individual UART interfaces (3 are required). This includes popular options, such as many Espressif boards and the Raspberry Pi Pico boards.
On resource-constrained boards, such as the *stm32_min_dev@blue*, the internal memory may be exceeded. This is related to the use of large LUTs in libfixmath (in flash) and the use of generously sized individual task stacks in the application code (in RAM).

## Peripherals

Every peripheral (sensors, actuators, displays, etc) is inteded to have its own library. Some are shipped with Zephyr, others we have supplied in this repository.
Most peripherals are optional! The only mandatory connection is a single UART interface between the microcontroller and the MCB.

For most applications, the LiDAR is also recommended, which requires an additional UART interface. Along with the Logging output, this means you will need three UARTs!

We provide the following drivers:

* ur-magni
* slamtech-rplidar

Peripheral drivers are designed to only depend on Zephyr itself and not require external libraries. This is not true for top-level application code.

# Getting Started

Application development uses the C programming language exclusively. However, when using Zephyr this is supplemented by Devicetree and KConfig files.
It is important to note that particularly in the hardware abstraction layers Zephyr makes heavy use of compile-time code generation. Approach this with caution!

## Zephyr
Follow the steps outlined in the official Zephyr documentation:
https://docs.zephyrproject.org/latest/getting_started/index.html

Familiarize yourself with Zephyr first using the provided example programs. If you're using the *disco_l475_iot1* board (as recommended!), then you'll be able to play around with the onboard sensors.
As a first project you may want to combine several example programs to access multiple sensors. This way you'll learn to use all of Zephyrs components.

## Safety Notice

The robot is heavy and, eventhough it is slow, it can be very forceful and potentially move unpredictably. It can carry heavy loads (up to 100kg) which may fall off and cause further damage or injury.

The robot has a built-in watchdog which will halt it if the software fails to send drive commands periodically. This protects against wiring problems and fully halted software.
To further mitigate dangerous drive commands the robot is hard limited to drive at absolute speeds of less than 1m/s. Typically, this is considerably slower than walking speed and slow enough that the robot will not tumble on sudden breaking manouvers.

The robot has two power buttons on the front wired as breaking connections. The black one is the main power switch. You will need to *depress* it for any operation.
The red button is the motor power switch. You can use it to turn off the motors completely while still keeping the MCB and microcontroller operational.
Be careful about turning the motors back on! The software guarantees you about one second of delay before the wheels start turning, but it's your responsibility to make sure that you don't cause dangerous manouvers.

When testing software changes affecting the movement, it is recommended to prop up the robot so that the wheels spin freely at first.
*Under no circumstances should the robot ever be operated on an elevated surface, such as a table. Do not operate it in the vicinity of stairs either.*

When wiring up peripherals, first disconnect at least one battery terminal. Make sure that power wiring is done correctly. Double check with a connectivity tester (multimeter).
Once you're certain, you can reattach the batteries. The robot has fuses for the motors, but offers no protection against wiring mistakes!

# REDACTED CONFERENCE 2026

The ESTHER case study has been used for experiments presented at REDACTED CONFERENCE 2026. The evaluation can be replicated as follows:

## Gathering new Reports

To conduct a new experiment you can use pre-recorded experimental data as described in the paper, with the following hardware:

- a microcontroller capable of running the ESTHER application, with three UART peripherals
- 2x (or more) USB-to-UART interfaces
- jumper wires

### Steps
1. Hook up the USB-to-UART interfaces to the pins for the Lidar and Magni on your MCU.
2. Build and Flash the ESTHER application for your MCU
3. Build the UART-Playback application for the *nativ_sim* target
3. Monitor the logging output of the MCU, e.g. using GNU screen, it should report an error because it cannot connect to the robot
4. Reset your MCU and immediately run the UART-Playback application with the --rtc-reset command line option
5. If all goes well, the ESTHER application will successfully boot with pre-recorded communication data. If it doesn't, try switching the USB-to-UART interfaces, check your wiring, and try a few more attempts at synchronizing the UART-Playback app with the boot sequence
6. Record a session using e.g. screen -L
7. Optional: Clean your report data using strings -w


## Analyzing Reports
Reports are analyzed using the ThreadAnalyzerAnalyzer. To generate reports used in the paper, just run taa.py.

If you have gathered new experimental data, you will need to extend taa.py to accomodate for it!
