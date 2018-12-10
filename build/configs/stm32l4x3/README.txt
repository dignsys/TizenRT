README
======

Configurations
==============

Each STM32F429I-DISCO configuration is maintained in a sub-directory and
can be selected as follows:

    cd ../../../os/tools
    ./configure.sh stm32f429i-disco/<subdir>
    cd ..
    make

If this is a Windows native build, then configure.bat should be used
instead of configure.sh:

    configure.bat stm32f429i-disco\<subdir>

Where <subdir> is one of the following:

  hello:
  -----
    Default configuration set for running "hello example" on
    stm32f429i-disco board.

  tc:
  --
    Default configuration set for running "kernel testcase example".
    If you want to run other testcases, you should modify the configuration
    using menuconfig.
    There are 11 testcases, kernel, network unit testcase (utc) / interaction
    testcase (itc), system I/O utc / itc, file system utc / itc, database utc /
    itc, device management utc / itc.
