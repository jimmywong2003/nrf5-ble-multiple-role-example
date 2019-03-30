# nrf5-multilink-lbs-nus-its-example

## Description

Inside Nordic nRF5 SDK, it has the BLE mult-link central example which uses to connect more BLE Blinky Application as peripherals.

In this example, it adds the NUS (Nordic Uart Service) and Image Transfer Service (ITS) with BLE Blinky service (LBS) together.

### Central Role

```
// Set the number of connection to 8 in this example

// Central Side (inside the sdk_config.h) 

// <o> NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - Maximum number of peripheral links. 
#ifndef NRF_SDH_BLE_PERIPHERAL_LINK_COUNT
#define NRF_SDH_BLE_PERIPHERAL_LINK_COUNT 0
#endif

// <o> NRF_SDH_BLE_CENTRAL_LINK_COUNT - Maximum number of central links. 
#ifndef NRF_SDH_BLE_CENTRAL_LINK_COUNT
#define NRF_SDH_BLE_CENTRAL_LINK_COUNT 8
#endif

// <o> NRF_SDH_BLE_TOTAL_LINK_COUNT - Total link count. 
// <i> Maximum number of total concurrent connections using the default configuration.

#ifndef NRF_SDH_BLE_TOTAL_LINK_COUNT
#define NRF_SDH_BLE_TOTAL_LINK_COUNT 8
#endif
```
### Peripheral Role

```
// <o> NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - Maximum number of peripheral links. 
#ifndef NRF_SDH_BLE_PERIPHERAL_LINK_COUNT
#define NRF_SDH_BLE_PERIPHERAL_LINK_COUNT 1
#endif

// <o> NRF_SDH_BLE_CENTRAL_LINK_COUNT - Maximum number of central links. 
#ifndef NRF_SDH_BLE_CENTRAL_LINK_COUNT
#define NRF_SDH_BLE_CENTRAL_LINK_COUNT 0
#endif

// <o> NRF_SDH_BLE_TOTAL_LINK_COUNT - Total link count. 
// <i> Maximum number of total concurrent connections using the default configuration.

#ifndef NRF_SDH_BLE_TOTAL_LINK_COUNT
#define NRF_SDH_BLE_TOTAL_LINK_COUNT 1
#endif

```


## Requirement

* Peripheral Role -- NRF52840 DK / NRF52832 DK
* Central Role -- NRF52840 DK
* IDE Segger Embedded Studio

The project may need modifications to work with later versions or other boards.

To compile it, clone the repository in the /nRF5_SDK_15.3.0/examples/ directory.

The application is built to be used with the official nRF5 SDK that can be downloaded from developer.nordicsemi.com



