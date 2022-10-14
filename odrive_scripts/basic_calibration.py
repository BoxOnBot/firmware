import odrive 

od = odrive.find_any()
print(od['vbus_voltage'])
