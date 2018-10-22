# USB MULTI CDC

USB communications device class with N channel FIFO's for XMOS.
One core handles endpoint0 AND all recieving endpoints. It also transmit data from the output FIFO buffers.
Outgoing block transfers are handles by the CDC core directly and does not affect the FIFO buffer.

Each CDC has 2 FIFO's (RX+TX) and they can have different size.

All CDC's must be running on the same tile as the XUD manager.

The code is intended to work with https://libusb.info/ on the host side.

Under Windows, an USB device called XMOS BLDC motor driver should enumerate. 
To add a Windows driver, choose WINUSB in the list over already available Windows drivers.

This code is under development.
