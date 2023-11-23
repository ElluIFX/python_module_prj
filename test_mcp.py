import time

import numpy as np
from scipy.optimize import curve_fit

from drivers.interface.if_cp2112 import CP2112_I2CInterfaceBuilder
from drivers.mcp3421 import MCP3421
from drivers.mcp4725 import MCP4725

CP2112_I2CInterfaceBuilder(clock=400000).register()

dac = MCP4725(vref=3.39409744)
adc = MCP3421()

raw_list = [x for x in range(0, 2600, 200)]
reads = []
raws = []
for raw in raw_list:
    dac.raw_value = raw
    raws.append(raw)
    time.sleep(0.5)
    rd = adc.voltage
    reads.append(rd)
    est = dac.voltage
    print(
        f"Raw-Set: {raw} Voltage-Read: {rd:.4f}V Voltage-Est: {est:.4f}V Diff: {rd-est:.4f}V"
    )

x = np.array(raws)
y = np.array(reads)


def func(x, a):
    return a * x / dac.max_raw + 0


popt, pcov = curve_fit(func, x, y)
print(f"Vref: {popt[0]:.9f}V")
