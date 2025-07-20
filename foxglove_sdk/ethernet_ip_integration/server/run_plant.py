import time
import math
import random
from pylogix import PLC

def run(comm):
    counter = 0
    emergency_stop_timer = 0  # Simple countdown timer
    emergency_stop_duration = 10  # 15 cycles (3 seconds at 0.2s intervals)

    while True:
        # Temperature simulation with sinusoidal pattern + noise
        base_temp = 75.0 + 8.0 * math.sin(counter * 0.05) + random.uniform(-3, 3)

        # Occasionally create error conditions (1% chance)
        error_state = random.random() < 0.01
        if error_state:
            # Push base temp higher to trigger alarm states
            base_temp += random.uniform(15, 25)

        temp = max(60, min(100, base_temp))
        comm.Write("Temperature", round(temp, 2))

        # Pressure with small variations
        base_pressure = 14.7 + random.uniform(-1, 1)

        # Occasionally create low pressure conditions (1% chance)
        if random.random() < 0.01:
            # Drop pressure below alarm threshold
            base_pressure -= random.uniform(3, 6)

        pressure = max(10, min(20, base_pressure))
        comm.Write("Pressure", round(pressure, 2))

        # Flow rate with occasional spikes
        flow_var = random.uniform(-8, 8)
        if random.random() < 0.03:  # 3% chance of spike
            flow_var += random.uniform(15, 25)
        flow = max(0, min(100, 50.0 + flow_var))
        comm.Write("Flow_Rate", round(flow, 1))

        # Tank levels with slow changes
        for i in range(1, 5):
            tag_name = f"Tank_Level_{i}"
            current = comm.Read(tag_name)
            current_val = current.Value if current.Value is not None else 50.0
            change = random.uniform(-1.5, 1.5)
            new_level = max(0, min(100, current_val + change))
            comm.Write(tag_name, round(new_level, 1))

        # Sensor array simulation
        for i in range(1, 5):  # Updated to match config (Sensor_1 to Sensor_4)
            sensor_name = f"Sensor_{i}"
            base_val = 28.0 + i * 0.5
            variation = random.uniform(-2, 2)
            sensor_val = max(20, min(35, base_val + variation))
            comm.Write(sensor_name, round(sensor_val, 1))

        # Check alarm conditions early for emergency stop logic
        high_temp_alarm = temp > 90.0

        # Simple emergency stop logic
        if high_temp_alarm:
            emergency_stop_timer = emergency_stop_duration  # Reset timer when alarm triggers
        elif emergency_stop_timer > 0:
            emergency_stop_timer -= 1  # Count down when temp is OK

        emergency_stop = emergency_stop_timer > 0  # Emergency stop active while timer > 0

        # Motor parameters
        motor_running_result = comm.Read("Motor_Running")
        motor_running = motor_running_result.Value if motor_running_result.Value is not None else True

        # Override motor running during emergency stop
        if emergency_stop:
            motor_running = False
            comm.Write("Motor_Running", False)
            comm.Write("Valve_Position", 0.0)  # Close valve during emergency
        else:
            # Restore motor to running when emergency stop clears
            if not motor_running:
                motor_running = True
                comm.Write("Motor_Running", True)
                comm.Write("Valve_Position", 50.0)  # Restore valve position

        if motor_running and not emergency_stop:
            # Motor current varies with load
            current = 12.5 + random.uniform(-2, 3)
            comm.Write("Motor_Current", round(current, 1))

            # Motor power correlates with current
            power = current * 0.68 + random.uniform(-0.5, 0.5)
            comm.Write("Motor_Power", round(power, 2))

            # Motor voltage - normal operating voltage
            comm.Write("Motor_Voltage", 480.0)

            # Motor temperature correlates with base temp and error states
            temp_result = comm.Read("Motor_Temp")
            motor_temp = temp_result.Value if temp_result.Value is not None else 65.0

            # Normal temperature change
            temp_change = random.uniform(-0.5, 1.5)

            # Correlate with base temperature - when base temp is high, motor temp increases more
            if base_temp > 85:
                temp_change += (base_temp - 85) * 0.3  # Amplify heating when base temp is high

            # During error states, motor temp can spike unpredictably
            if error_state:
                temp_change += random.uniform(5, 15)  # Motor overheating during system errors

            # Occasionally create independent motor overheating (2% chance)
            if random.random() < 0.02:
                temp_change += random.uniform(8, 20)

            new_motor_temp = min(120, motor_temp + temp_change*random.uniform(-0.2, 0.3))
            comm.Write("Motor_Temp", round(new_motor_temp, 1))

            # Pump speed varies slightly
            speed = 1750 + random.randint(-75, 75)
            comm.Write("Pump_Speed", speed)
        else:
            comm.Write("Motor_Current", 0.0)
            comm.Write("Motor_Power", 0.0)
            comm.Write("Pump_Speed", 0)
            comm.Write("Motor_Voltage", 0.0)

        # Vibration simulation
        vib_base = 0.25 + 0.1 * math.sin(counter * 0.3)
        vibration = max(0, vib_base + random.uniform(-0.05, 0.15))
        comm.Write("Vibration", round(vibration, 3))

        # Pressure alarm check (temperature alarm already checked above)
        pressure_result = comm.Read("Pressure")
        pressure_val = pressure_result.Value if pressure_result.Value is not None else 14.7
        low_pressure_alarm = pressure_val < 12.0

        comm.Write("Emergency_Stop", emergency_stop)
        comm.Write("High_Temp_Alarm", high_temp_alarm)
        comm.Write("Low_Pressure_Alarm", low_pressure_alarm)

        # Set error codes based on alarm conditions
        error_code = 0  # Default: no error
        if emergency_stop:
            error_code = 10  # Emergency stop error code
        elif high_temp_alarm:
            error_code = 34  # Temperature error code
        elif low_pressure_alarm:
            error_code = 65  # Pressure error code

        comm.Write("Error_Code", error_code)

        # Increment alarm count for any error condition
        if error_code != 0:
            alarm_count_result = comm.Read("Alarm_Count")
            alarm_count = alarm_count_result.Value if alarm_count_result.Value is not None else 0
            comm.Write("Alarm_Count", alarm_count + 1)

        # Update counters
        if counter % 60 == 0:  # Every minute
            prod_count_result = comm.Read("Production_Count")
            prod_count = prod_count_result.Value if prod_count_result.Value is not None else 0
            comm.Write("Production_Count", prod_count + 1)

        # Runtime hours
        runtime_result = comm.Read("Runtime_Hours")
        runtime = runtime_result.Value if runtime_result.Value is not None else 0
        comm.Write("Runtime_Hours", round(runtime + 1/3600, 3))

        counter += 1
        time.sleep(0.2)

with PLC() as comm:
    comm.IPAddress = '0.0.0.0'
    try:
        run(comm)
    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"An error occurred: {e}")
