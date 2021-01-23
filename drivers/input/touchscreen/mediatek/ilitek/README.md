# Introduce

Ipio touch driver is implemented by ILI Technology Corp, which is mainly used on its new generation touch ICs, TDDI.

# Support ICs

The following lists which of TDDI IC types supported by the driver.

* ILI7807F
* ILI7807H
* ILI9881F

# Verified platform

* Firefly-RK3288 (Rockchip)
* Helio X20 (MT6797)

The default in this driver works on RK3288.

If you woule like to port it on MTK platforms, you must open the macro shown on the below:

**common.h**
```
#define PLATFORM_MTK
```
# Functions

## Gesture

To enable the support of this feature, you can either open it by its node under /proc :

```
echo on > /proc/ilitek/gesture
echo off > /proc/ilitek/gesture
```

or change its variable from the file **config.c** :
```
core_config->isEnableGesture = false;
```

## Check battery status

Some specific cases with power charge may affect on our TDDI IC so that we need to protect it if the event is occuring.

In order to enable this function, the first you need to do is to enable its macro at **common.h**

```
/* Check battery's status in order to avoid some effects from charge. */
#define BATTERY_CHECK
```

Once the function has been built in, you must have to on it by echoing:

```
echo on > /proc/ilitek/check_battery
echo off > /procilitek/check_battery
```

Or you can run it by default by setting its flag as true.

```
ipd->isEnablePollCheckPower = true;
```

## DMA

If your platform needs to use DMA with I2C, you can open its macro from **common.h** :
```
#define ENABLE_DMA 
```
Note, it is disabled as default.

## I2C R/W Segment

Some of platforms with i2c bus may have no ability to R/W data with large length at once, so we provide th way of segmental operation.

Before enable its macro, you first need to know how much length your platform could accept. In **core/i2c.c** where you can find its variable:

```
core_i2c->seg_len = 256; // length of segment
```

You can change its value as long as you know the length that your platform can work on.

After that, as mentioned before, you need to enable its macro, which defines at **common.h**, to actually let driver run its function.

```
/* Split the length written to or read from IC via I2C. */
#define I2C_SEGMENT
```

## FW upgrade at boot time
Apart from manual firmware upgrade, we also provide the way of upgrading firmware when system boots initially as long as the verions of firmware

in the IC is different the version that you're going to upgrade.

```
/* Be able to upgrade fw at boot stage */
#define BOOT_FW_UPGRADE
```

## Glove/Proximity/Phone cover

These features need to be opened by the node only.

```
echo enaglove > /proc/ilitek/ioctl  --> enale glove
echo disglove > /proc/ilitek/ioctl  --> disable glove
echo enaprox > /proc/ilitek/ioctl   --> enable proximity
echo disprox > /proc/ilitek/ioctl   --> disable proximity
echo enapcc > /proc/ilitek/ioctl    --> enable phone cover
echo dispcc > /proc/ilitek/ioctl    --> disable phone cover
```

# The metho of debug

To do so, you must firstlly ensure that the version of image is compiled for the debug instead of the user, otherwise it won't allow users to write commands through our device nodes.

## Debug message

It is important to see more details with debug messages if an error happends somehow. To look up them, you can set up debug level via a node called debug_level under /proc.

At the begining, you should read its node by cat to see what the levels you can choose for.

```
cat /proc/ilitek/debug_level

DEBUG_NONE = 0
DEBUG_IRQ = 1
DEBUG_FINGER_REPORT = 2
DEBUG_FIRMWARE = 4
DEBUG_CONFIG = 8
DEBUG_I2C = 16
DEBUG_BATTERY = 32
DEBUG_MP_TEST = 64
DEBUG_IOCTL = 128
DEBUG_NETLINK = 256
DEBUG_ALL = -1
```

The default level is zero once you get the driver at the first time. Let's say that we want to check out what is going on when the driver is upgrading firmware.

```
echo 4 > /proc/ilitek/debug_level
```

The result will only print the debug message with the process of firmware. Futhermore, you can also add two or three numbers to see multiple debug levels.

```
echo 7 > /proc/ilitek/debug_level
```

In this case the debusg message prints the status of IRQ, FINGER_REPORT and FIRMWARE. Finally, you can definitly see all of them without any thoughts.

```
echo -1 > /proc/ilitek/debug_level
```

## I2C R/W

Sometime it is necessary to see IC's direct output by sending I2C commands in special cases.

We provide three functions to achieve it by echoing a specific device nodev which is created under /proc/ilitek/ioctl.

The command line defines as below :

```
# echo <function>,<length>,<data> > /proc/ilitek/ioctl
```

The comma must be added in each following command to let the driver recognize them.

In the function you can order specific actions to operate I2C such as Write (i2c_w), Read (i2c_r) and Write/Read (i2c_w_r). For instance, if you want to order write command, you can then type this as following:

```
# echo i2c_w,0x3,0x40,0x0,0x1
```

In this case the function is Write, the length is 3, and the i2c command is 0x40, 0x0 and 0x1. Note that it is important to add "0x" with every numbers that you want to cmmand except for the functions.

Additionly, we offer a useful feature to do Write/Read at once: Delay time. In this function it will do Write first and then delay a certain time before reading data through I2C. Note that the default of delay time is 1ms

```
# echo i2c_w_r, <length of wirte>, <length of read>, <delay time>, <data>
```

# File sturcture

```
├── common.h
├── core
│   ├── config.c
│   ├── config.h
│   ├── finger_report.c
│   ├── finger_report.h
│   ├── firmware.c
│   ├── firmware.h
│   ├── flash.c
│   ├── flash.h
│   ├── i2c.c
│   ├── i2c.h
│   ├── Makefile
│   ├── mp_test.c
│   ├── mp_test.h
│   └── sup_chip.c
├── Makefile
├── m.sh
├── platform.c
├── platform.h
├── README.md
└── userspace.c
```

* The concepts of this driver is suppose to hide details, and you should ignore the core functions and follow up the example code **platform.c**
to write your own platform c file (like copy it to platform_mtk.c and modify gpio numbers, dts's match name etc.)

* If you want to create more nodes on the device driver for user space, just refer to **userspace.c**.

* In **common.h** file where has a lots of definations about IC's addres and protocol commands on firmware.

* The directory **Core** includes our touch ic settings, functions and other features.

# Porting

## Tell driver what IC types it should support to.

To find a macro called *ON_BOARD_IC* in **chip.h** and chose one of them:

```
// This macro defines what types of chip supported by the driver.
#define ON_BOARD_IC		0x7807
//#define ON_BOARD_IC		0x9881
```
 In this case the driver now supports ILI7807.
 
## Check your DTS file and i2c bus number

The DTS file with RK3288 development platform locates at **arch/arm/boot/dts/firefly-rk3288.dts**.

Open it and add the gpio number in one of i2c buses.

```
&i2c1 {
		ts@41 {
			status = "okay";
        	compatible = "tchip,ilitek";
      		reg = <0x41>;
         	touch,irq-gpio = <&gpio8 GPIO_A7 IRQ_TYPE_EDGE_RISING>;
         	touch,reset-gpio = <&gpio8 GPIO_A6 GPIO_ACTIVE_LOW>;
    	};

};
```
In this case the slave address is 0x41, and the name of table calls **tchip,ilitek**.

**touch,irq-gpio** and **touch,reset-gpio** represent INT pin and RESET pin.

## Copy **platform.c** as your platform c. file.

Once you have done that, the first thing you should make sure is what the i2c name calls in your dts file:

```
/*
 * The name in the table must match the definiation
 * in a dts file.
 *
 */
static struct of_device_id tp_match_table[] = {
	{ .compatible = "tchip,ilitek"},
    {},
};
```
Change it according to your DTS file. 

The next we should modify is the name of gpios. Find out the two macros and chang it.

```
#define DTS_INT_GPIO	"touch,irq-gpio"
#define DTS_RESET_GPIO	"touch,reset-gpio"
```

For more information about gpio settins, you can refer to the function called **ilitek_platform_gpio**

```
static int ilitek_platform_gpio(void)
{
	int res = 0;
#ifdef CONFIG_OF
	struct device_node *dev_node = TIC->client->dev.of_node;
	uint32_t flag;
#endif
	uint32_t gpios[2] = {0};// 0: int, 1:reset

#ifdef CONFIG_OF
	gpios[0] = of_get_named_gpio_flags(dev_node, DTS_INT_GPIO, 0, &flag);
	gpios[1] = of_get_named_gpio_flags(dev_node, DTS_RESET_GPIO, 0, &flag);
#endif

    ....
}
```

# Release Note

* V1.0.1.2
 * Introduce Test Mode and MP Test function.
 * Add gesture code.
 * Add a debug node that can be run under user-built image.
 * Rewrote Function Control.
 * Fixed several bugs.

* V1.0.1.1
 * Add i2c segmental read.
 * Redesign protcol structure. The protocol now became single moudle to have its own c file.
 * Define macros to enable some common functions.
 * Enable IRQ before handling the event triggered by fw.
 * Add block tag at boot upgrade and reserved block for customers.
 * Support new type of version of protocol (expend to 3 byte).
 * Able to print report data if image is built for user

* V1.0.1.0 (Offical Release)
 * Fixed the size of CTPM_FW isn't subtracted by 32 before filling data into buffer.
 * FW upgrade at boot stage is disable as default.

* V1.0.0.10
 * Fixed the issue of I2CUart when its length of next buffer is larger
   than previous one.
 * Add FW upgrade at boot stage. It will be actiaved when the version of FW that is going to
   upgrade is different the previous version, otherwise it will deny to upgrade.

* V1.0.0.9
  * Add a new way to program firmware if the hex file includes block information.
  * Define the rule to show up detailed debug messages.
  * Add a node to enable the thread to check battery status.
  * Fix some bugs.

* V1.0.0.8
  * Add support of MTK and DMA with I2C.
  * Add kthread to handle interrupt event.
  * Add support of gesture wake up in suspend/resume.
  * Remove power supply notifier at check battery status.
  * Add the functions such as glove, proximity and phone cover.
  * Support new calculation with i2cuart mode.

* V1.0.0.7
  * Fixed issue of showing upgrade status while using APK
  * Added a flag to enable if resolution is set by default or fw
  * support the length of i2cuart and send to users with one package
  * i2c error is no long showing up if doing ic reset
  * Added a feature to check battery status 
  * fixed the error while programming fw (garbage data left)

* V1.0.0.6
  * Finger report won't send to user if checksum error occurs
  * Fixed the error sometimer work queue couldn't be executed while interrupting

* V1.0.0.5
  * Fix the error of checksum with finger report
  * Add flash table in order to get flash information dynamatically.
  * fix the error crc to calculate length instead of address.
  * Optimise the calculation of fw upgrade 
  * Remove ILI2121 from support list
  * Remove the old method of fw upgrade
  * Add the delay of 1ms after sending 0xF6 command

* V1.0.0.4
  * Add 7807H in the support list
  * Support 7807 FW upgrade
  * Set 1920*1080 as fixed resolution
  * Add 0xF6 cmd for P5.0 befre reading data
  * Change Netlink's port from 31 to 21
  * Add Regulator power on
  * Add I2CUART mode
  * Add a new way to program flash

* V1.0.0.3
  * Fixed the issue of create skb buff in netlink
  * Fixed the issue of doing bitwise with tp info got from ic
  * Improved the stability with IRAM upgrade.
  * Improved the stability while getting chip id, particularly 7807F.
  * Added Notifier FB as main suspend/resume function called.
  * Added MP Test in Test mode but 7807F not supported it yet.
  * Optimised code structure.

* V1.0.0.2
  * Fixed the issue of no resposne while using A protocol.
  * Now the resolution of input device can correctly be set by TP info.
  * Added compiler flag -Wall
  * Added disable/enable irq while chiang firmware mode before and after.
  * Added dynamic debug outputs.
  * Added a feature that allows firmware upgrading into IRAM directly.

* V1.0.0.1
  * Support firmware upgrade for 9881F
  * Improved the stability while upgrading firmware
  * Improved the stability while reading chid id from touch ic
  * Fixed some bugs

* V1.0.0.0
  * Support ILI7807F, ILI9881F
  * Support protocol v5.0
  * Support upgrade firmware for 7807F (9881F not yet)
  * Support mode switch (demo/debug/test/i2cUart)
  * Support demo/debug mode with packet ID while reporting figner touch.
  * Support early suspend
  * Fixed some bugs
