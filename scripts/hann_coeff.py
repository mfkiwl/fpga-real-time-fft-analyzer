#!/usr/bin/env python3
import numpy as np
N = 16384
hann = 0.5*(1-np.cos(2*np.pi*np.arange(N)/(N-1)))              # 0…1
q15  = np.round((hann-0.5)*2**16).astype(np.int16)             # −32768…+32767

with open("hann.vhd","w") as f:
    f.write("library IEEE;\n")
    f.write("use IEEE.STD_LOGIC_1164.ALL;\n")
    f.write("use IEEE.NUMERIC_STD.ALL;\n")
    f.write("package hann is\n")
    f.write(f"type rom_t is array(0 to {int(N)}-1) of signed(15 downto 0); \n")
    f.write("    constant Hann_ROM : rom_t := (\n")
    for i,v in enumerate(q15):
        comma = ',' if i<N-1 else ''
        f.write(f"        {i:3d} => to_signed({int(v)},16){comma}\n")
    f.write("    );\n")
    f.write("end package;\n")
print("Hann ROM generated (signed Q15)")


