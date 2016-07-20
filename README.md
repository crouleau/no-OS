no-OS
=====

This branch is the full system working-ish with the Analog Devices IP block. The system can transmit at whatever frequency you choose, and choose among DDS settings. Issues:
 - Loopback didn't work. Read was timing out - probably related to all the IOdelay logic not being setup properly. I fixed a bug in the Verilog but never got around to testing if it worked
 - DMA option didn't work, caused system crash, no idea why (perhaps fixed by the bug I fixed in recent commit)