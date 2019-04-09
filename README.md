# nrf5-multlink-lbs-nus-its-example

## Description

Inside Nordic nRF5 SDK, it has the BLE mult-link central example which uses to connect more BLE Blinky Application as peripherals.

In this example, there are three difference roles.
* Peripheral role 
* Central role
* 2 x Peripheral Role + 2 x Central Role

All of them contains the GATT Server / Client.
* NUS (Nordic Uart Service)
* Image Transfer Service (ITS)
* BLE Blinky service (LBS)


## Requirement

* Peripheral Role -- NRF52840 DK / NRF52832 DK
* Central Role -- NRF52840 DK
* IDE Segger Embedded Studio

The project may need modifications to work with later versions or other boards.

To compile it, clone the repository in the /nRF5_SDK_15.3.0/examples/ directory.

The application is built to be used with the official nRF5 SDK that can be downloaded from developer.nordicsemi.com

For details, you can refer to the [URL](https://jimmywongbluetooth.wordpress.com/2019/04/09/ble-multiple-role-peripheral-central).



