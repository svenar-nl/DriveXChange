# DriveXChange code

This is the source-code for the DriveXChange robot.

You can build the project with all supported [Mbed OS build tools](https://os.mbed.com/docs/mbed-os/latest/tools/index.html).


## Building and running

1. Connect a USB cable between the USB port on the board and the host computer.
1. Run the following command to build the example project and program the microcontroller flash memory:

    ```bash
    $ mbed compile -m NUCLEO_F303K8 -t ARMC6 --flash
    ```

Your PC may take a few minutes to compile your code.

The binary is located at `./BUILD/NUCLEO_F303K8/ARMC6/DriveXChange.bin`.

Alternatively, you can manually copy the binary to the board, which you mount on the host computer over USB.

Depending on the target, you can build the example project with the `GCC_ARM`, `ARM` or `IAR` toolchain. After installing Arm Mbed CLI, run the command below to determine which toolchain supports your target:

```bash
$ mbed compile -S
```


### License and Contributing

The software is provided under GNU General Public License v3.0. Contributions to this project are accepted under the same license.