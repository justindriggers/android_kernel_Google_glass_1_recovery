This is a tarball of the current Synaptics kernel.org touch device driver.
Included in the tarball are the full sources for the driver, along with some
related files.  Files in the tarball are:

      README-SYNAPTICS.txt
          This file.
      CHANGELOG-SYNAPTICS.txt
          Important changes since the last tarball.
      include/linux/rmi.h
          Public header file for the driver.
      drivers/input/rmi4/rmi*.[ch]
          Headers and source files for the driver.
      drivers/input/rmi4/Makefile
          Builds the driver.
      drivers/input/rmi4/Kconfig
          Config file
      drivers/input/Makefile
          For reference.
      drivers/input/Kconfig
          For reference.
      arch/arm/mach-omap2/board-omap4panda.c
          A testing board file, for reference.
      arch/arm/configs/panda_defconfig
          A testing kernel configuration, for reference.
      Documentation/input/rmi*.txt
      Documentation/ABI/testing/*rmi4
          Associated documentation.



To apply the new driver codebase to your current system, please follow
the steps below:

+ This step is needed ONLY if you were previous using an older version of the
Synaptics driver that had sources in drivers/input/touchscreen.  In this case
you need to do the following before proceeding to the next step.
    * remove the files drivers/input/touchscreen/rmi*
    * edit drivers/input/touchscreen/Makefile and
drivers/input/touchscreen/KConfig to remove RMI driver references


+ If you are updating from a previous tarball of the Synaptics driver, review
the file CHANGELOG-SYNAPTICS.txt for any important changes that might have
occurred since the last tarball was issued.


+ copy the drivers/input/rmi4 folder from the tarball into your kernel tree


+ copy include/linux/rmi.h from the tarball into include/linux in your
kernel tree


+ edit drivers/input/Makefile and drivers/input/KConfig to reference
drivers/input/rmi4 appropriately (see the reference files in the tarball)


+ if you plan to use the in-driver firmware update feature, please refer to
the file Documentation/input/rmi4.txt and follow the instructions there.


+ use make menuconfig to update your configuration and select the appropriate
RMI4 features for you sensor.  You may safely omit function implementations
that are not present on your sensor.


+ (alternative to make menuconfig) edit your defconfig file to remove
TOUCHSCREEN_SYNAPTICS_RMI4_I2C and related symbols (if they are present);
replace them with the appropriate CONFIG_RMI4_XXX symbols.  See the
panda_defconfig file in the tarball for reference, and the
drivers/input/rmi4/Kconfig file for details on each setting.  The following
lines are required as a minimum for a functional RMI4 device:
    CONFIG_RMI4_BUS=y
    CONFIG_RMI4_GENERIC=y
and one of either
    CONFIG_RMI4_I2C=y
or
    CONFIG_RMI4_SPI=y
depending on your system.  You must also enable any RMI4 functions that are
present on your sensor.


+ update your board file to specify the appropriate platform data.  See the
attached board_omap4panda.c file for reference.


+ do a make distclean to get rid of old object files and binaries


+ make defconfig to apply your configuration updates


+ finally, rebuild your kernel

