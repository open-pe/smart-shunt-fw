import pyvisa
rm = pyvisa.ResourceManager()
print(rm.list_resources())

import pyvisa_py
pyvisa_py.