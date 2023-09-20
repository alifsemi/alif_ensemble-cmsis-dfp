from com.arm.debug.flashprogrammer.IFlashClient import MessageLevel

from flashprogrammer.device import ensureDeviceOpen
from flashprogrammer.execution import ensureDeviceStopped
from flashprogrammer.device_memory import writeToTarget, readFromTarget, intToBytes, intFromBytes

import time

SYST_CSR = 0xE000E010 # SysTick Control and Status Register

# IMPORTANT
# This setup script is target specific for the STM32F10x

def setup(client, services):
    # get a connection to the core
    conn = services.getConnection()
    dev = conn.getDeviceInterfaces().get("Cortex-M55_1")
    ensureDeviceOpen(dev)
    ensureDeviceStopped(dev)

    # The following code asserts Reset to all subsystems to prevent data corruption while programming MRAM

