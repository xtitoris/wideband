#include "port_shared.h"

// board-specific stuff shared between bootloader and firmware

static const CANConfig canConfig125 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 32MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=16, Seq 1=13 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(16 - 1)  | CAN_BTR_TS1(13 - 1) | CAN_BTR_TS2(2 - 1),
};

static const CANConfig canConfig250 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 32MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=8, Seq 1=13 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(8 - 1)  | CAN_BTR_TS1(13 - 1) | CAN_BTR_TS2(2 - 1),
};

static const CANConfig canConfig500 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 32MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=4, Seq 1=13 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(4 - 1)  | CAN_BTR_TS1(13 - 1) | CAN_BTR_TS2(2 - 1),
};

static const CANConfig canConfig1000 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 32MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=2, Seq 1=13 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(2 - 1)  | CAN_BTR_TS1(13 - 1) | CAN_BTR_TS2(2 - 1),
};

const CANConfig& GetCanConfig(CanBaudRate baudRate) {
    switch (baudRate) {
        case CanBaudRate::Baud500Kbps:
            return canConfig500;
        case CanBaudRate::Baud1Mbps:
            return canConfig1000;
        case CanBaudRate::Baud125Kbps:
            return canConfig125;
        case CanBaudRate::Baud250Kbps:
            return canConfig250;
    }

    // default to 500kbps
    return canConfig500;
}
