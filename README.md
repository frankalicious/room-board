room-board-schematic
====================
# hardware
- MSP430G2553IN20
- pir module
- brightness measuring with led (kicad version uses phototransistor)
- nrf24l01 radio module

# gateway
https://github.com/frankalicious/nrf24l01-serial-gateway

# old schematic
gschem was used for first schematic version

# current schematic and board
designed in kicad

# git submodule
msp430-gcc firmware uses msprf24
```bash
$ git submodule init
$ git submodule update
```
For more information about git submodules see here: http://git-scm.com/book/en/v2/Git-Tools-Submodules
