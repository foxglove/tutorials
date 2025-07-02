
from pylogix import PLC
import time

with PLC() as comm:
    comm.IPAddress = '0.0.0.0'
    comm.Micro800 = True

    while True:
        try:
            tank_val_1 = comm.Read('Tank_Level_1')
            print(tank_val_1.TagName, tank_val_1.Value, tank_val_1.Status)
            tank_val_2 = comm.Read('Tank_Level_2')
            print(tank_val_2.TagName, tank_val_2.Value, tank_val_2.Status)
            tank_val_3 = comm.Read('Tank_Level_3')
            print(tank_val_3.TagName, tank_val_3.Value, tank_val_3.Status)
        except Exception as e:
            print(f"Error reading tank levels: {e}")
            tank_val_1 = None
            tank_val_2 = None
            tank_val_3 = None
        time.sleep(0.2)
