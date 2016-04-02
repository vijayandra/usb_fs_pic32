How to initial setup

    1. Install Microchip PIC32 compiler,(Choose Free lincense)

    http://www.microchip.com/mplab/compilers

    Windows (x86/x64)
        MPLAB® XC32 Compiler v1.40 	6/17/2015 	64.3 MB 	ZIP
    Linux 32-Bit and Linux 64-Bit (Requires 32-Bit Compatibility Libraries)
        MPLAB® XC32 Compiler v1.40 	6/17/2015 	62.6 MB 	ZIP
    Mac (10.X)
        MPLAB® XC32 Compiler v1.40 	6/17/15 	64.0 MB 	ZIP

    2. [LINUX Only] Copy following files into /etc/udev/rules.d/

    udev_linux/z011_mchp_jlink.rules
    udev_linux/z010_mchp_tools.rules
    udev_linux/50-iwscope.rules
    This is must to bootloader will work flawless

    3. Add PIC32 Installation compiler path to global system path

    Windows

    Linux

    4. Do some testing about if compiler installed properly

    5. Now start Learning tool
