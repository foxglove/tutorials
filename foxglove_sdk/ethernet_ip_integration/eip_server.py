#!/usr/bin/env python3
"""
Comprehensive Mock Ethernet/IP Server with Dynamic Data
Uses cpppo with a wider range of tag types and simulated dynamic values
"""

import subprocess
import sys
import signal
import threading
import time
import random
import math
from pylogix import PLC

class ComprehensiveMockServer:
    def __init__(self, host='0.0.0.0', port=44818):
        self.host = host
        self.port = port
        self.running = False
        self.server_process = None
        self.comm = None

        # Define comprehensive tag set
        self.tags = {
            # Process variables
            "Temperature": {"type": "REAL", "init": 75.5, "min": 60, "max": 100},
            "Pressure": {"type": "REAL", "init": 14.7, "min": 10, "max": 20},
            "Flow_Rate": {"type": "REAL", "init": 50.0, "min": 0, "max": 100},
            "Level": {"type": "REAL", "init": 80.0, "min": 0, "max": 100},
            "Vibration": {"type": "REAL", "init": 0.25, "min": 0, "max": 5},

            # Control variables
            "Motor_Running": {"type": "BOOL", "init": True},
            "Pump_Speed": {"type": "INT", "init": 1750, "min": 0, "max": 3600},
            "Valve_Position": {"type": "REAL", "init": 50.0, "min": 0, "max": 100},
            "Pump_Enable": {"type": "BOOL", "init": True},
            "Emergency_Stop": {"type": "BOOL", "init": False},

            # Status and alarms
            "High_Temp_Alarm": {"type": "BOOL", "init": False},
            "Low_Pressure_Alarm": {"type": "BOOL", "init": False},
            "System_Status": {"type": "INT", "init": 1},  # 0=Stopped, 1=Running, 2=Error
            "Error_Code": {"type": "INT", "init": 0},
            "Alarm_Count": {"type": "INT", "init": 0},

            # Production data
            "Production_Count": {"type": "DINT", "init": 12345},
            "Runtime_Hours": {"type": "REAL", "init": 1234.5},
            "Batch_Number": {"type": "DINT", "init": 1001},
            "Recipe_Number": {"type": "INT", "init": 5},
            "Quality_Rating": {"type": "REAL", "init": 95.5, "min": 0, "max": 100},

            # Setpoints
            "Setpoint_Temp": {"type": "REAL", "init": 80.0, "min": 60, "max": 100},
            "Setpoint_Pressure": {"type": "REAL", "init": 15.0, "min": 10, "max": 20},
            "Setpoint_Flow": {"type": "REAL", "init": 55.0, "min": 0, "max": 100},

            # Tank levels
            "Tank_Level_1": {"type": "REAL", "init": 75.0, "min": 0, "max": 100},
            "Tank_Level_2": {"type": "REAL", "init": 60.0, "min": 0, "max": 100},
            "Tank_Level_3": {"type": "REAL", "init": 45.0, "min": 0, "max": 100},
            "Tank_Level_4": {"type": "REAL", "init": 85.0, "min": 0, "max": 100},

            # Sensor arrays (individual tags for cpppo compatibility)
            "Sensor_1": {"type": "REAL", "init": 25.5, "min": 20, "max": 35},
            "Sensor_2": {"type": "REAL", "init": 28.0, "min": 20, "max": 35},
            "Sensor_3": {"type": "REAL", "init": 30.2, "min": 20, "max": 35},
            "Sensor_4": {"type": "REAL", "init": 27.8, "min": 20, "max": 35},
            "Sensor_5": {"type": "REAL", "init": 29.1, "min": 20, "max": 35},
            "Sensor_6": {"type": "REAL", "init": 31.4, "min": 20, "max": 35},
            "Sensor_7": {"type": "REAL", "init": 26.9, "min": 20, "max": 35},
            "Sensor_8": {"type": "REAL", "init": 33.0, "min": 20, "max": 35},

            # Motor data
            "Motor_Current": {"type": "REAL", "init": 12.5, "min": 0, "max": 50},
            "Motor_Voltage": {"type": "REAL", "init": 480.0, "min": 400, "max": 500},
            "Motor_Power": {"type": "REAL", "init": 8.5, "min": 0, "max": 25},
            "Motor_Temp": {"type": "REAL", "init": 65.0, "min": 20, "max": 120},

            # Additional data types
            "BOOL_Test_1": {"type": "BOOL", "init": True},
            "BOOL_Test_2": {"type": "BOOL", "init": False},
            "INT_Test": {"type": "INT", "init": 1234},
            "DINT_Test": {"type": "DINT", "init": 123456789},
            "REAL_Test": {"type": "REAL", "init": 3.14159},
        }

    def get_tag_definitions(self):
        """Convert tags to cpppo command line format"""
        tag_defs = []
        for tag_name, tag_info in self.tags.items():
            tag_type = tag_info["type"]
            tag_defs.append(f"{tag_name}={tag_type}[1]")
        return tag_defs

    def print_tag_summary(self):
        """Print summary of all available tags"""
        print("\n" + "="*80)
        print("COMPREHENSIVE MOCK ETHERNET/IP SERVER - TAG SUMMARY")
        print("="*80)

        categories = {
            "Process Variables": [k for k, v in self.tags.items()
                                if k in ["Temperature", "Pressure", "Flow_Rate", "Level", "Vibration"]],
            "Control Variables": [k for k, v in self.tags.items()
                                if "Motor" in k or "Pump" in k or "Valve" in k or k in ["Emergency_Stop"]],
            "Status & Alarms": [k for k, v in self.tags.items()
                              if "Alarm" in k or "Status" in k or "Error" in k],
            "Production Data": [k for k, v in self.tags.items()
                              if k in ["Production_Count", "Runtime_Hours", "Batch_Number", "Recipe_Number", "Quality_Rating"]],
            "Setpoints": [k for k, v in self.tags.items() if k.startswith("Setpoint")],
            "Tank Levels": [k for k, v in self.tags.items() if k.startswith("Tank_Level")],
            "Sensors": [k for k, v in self.tags.items() if k.startswith("Sensor_")],
            "Test Tags": [k for k, v in self.tags.items() if k.endswith("_Test")]
        }

        for category, tag_list in categories.items():
            if tag_list:
                print(f"\n{category}:")
                for tag in sorted(tag_list):
                    tag_info = self.tags[tag]
                    init_val = tag_info.get("init", "N/A")
                    tag_type = tag_info["type"]
                    print(f"  {tag:<25} ({tag_type:<4}) = {init_val}")

        print(f"\nTotal Tags: {len(self.tags)}")
        print(f"Server Address: {self.host}:{self.port}")
        print("="*80 + "\n")

    def initialize_tag_values(self):
        """Initialize tags with their starting values"""
        try:
            self.comm = PLC()
            self.comm.IPAddress = self.host
            self.comm.Port = self.port

            print("Initializing tag values...")
            for tag_name, tag_info in self.tags.items():
                init_value = tag_info.get("init")
                if init_value is not None:
                    try:
                        result = self.comm.Write(tag_name, init_value)
                        if result.Status != "Success":
                            print(f"Warning: Could not initialize {tag_name}: {result.Status}")
                    except Exception as e:
                        print(f"Warning: Error initializing {tag_name}: {e}")

            print("Tag initialization complete.")

        except Exception as e:
            print(f"Error during tag initialization: {e}")
        finally:
            if self.comm:
                self.comm.Close()

    def simulate_dynamic_data(self):
        """Continuously update tags with realistic simulated data"""
        counter = 0

        try:
            self.comm = PLC()
            self.comm.IPAddress = self.host
            self.comm.Port = self.port

            while self.running:
                try:
                    # Temperature simulation with sinusoidal pattern + noise
                    base_temp = 75.0 + 8.0 * math.sin(counter * 0.05) + random.uniform(-3, 3)
                    temp = max(60, min(100, base_temp))
                    self.comm.Write("Temperature", round(temp, 2))

                    # Pressure with small variations
                    base_pressure = 14.7 + random.uniform(-1, 1)
                    pressure = max(10, min(20, base_pressure))
                    self.comm.Write("Pressure", round(pressure, 2))

                    # Flow rate with occasional spikes
                    flow_var = random.uniform(-8, 8)
                    if random.random() < 0.03:  # 3% chance of spike
                        flow_var += random.uniform(15, 25)
                    flow = max(0, min(100, 50.0 + flow_var))
                    self.comm.Write("Flow_Rate", round(flow, 1))

                    # Tank levels with slow changes
                    for i in range(1, 5):
                        tag_name = f"Tank_Level_{i}"
                        current = self.comm.Read(tag_name).Value or 50.0
                        change = random.uniform(-1.5, 1.5)
                        new_level = max(0, min(100, current + change))
                        self.comm.Write(tag_name, round(new_level, 1))

                    # Sensor array simulation
                    for i in range(1, 9):
                        sensor_name = f"Sensor_{i}"
                        base_val = 28.0 + i * 0.5
                        variation = random.uniform(-2, 2)
                        sensor_val = max(20, min(35, base_val + variation))
                        self.comm.Write(sensor_name, round(sensor_val, 1))

                    # Motor parameters
                    motor_running = self.comm.Read("Motor_Running").Value
                    if motor_running:
                        # Motor current varies with load
                        current = 12.5 + random.uniform(-2, 3)
                        self.comm.Write("Motor_Current", round(current, 1))

                        # Motor power correlates with current
                        power = current * 0.68 + random.uniform(-0.5, 0.5)
                        self.comm.Write("Motor_Power", round(power, 2))

                        # Motor temperature rises slowly when running
                        temp = self.comm.Read("Motor_Temp").Value or 65.0
                        temp_change = random.uniform(-0.5, 1.5)
                        new_temp = max(20, min(120, temp + temp_change))
                        self.comm.Write("Motor_Temp", round(new_temp, 1))

                        # Pump speed varies slightly
                        speed = 1750 + random.randint(-75, 75)
                        self.comm.Write("Pump_Speed", speed)
                    else:
                        self.comm.Write("Motor_Current", 0.0)
                        self.comm.Write("Motor_Power", 0.0)
                        self.comm.Write("Pump_Speed", 0)

                    # Vibration simulation
                    vib_base = 0.25 + 0.1 * math.sin(counter * 0.3)
                    vibration = max(0, vib_base + random.uniform(-0.05, 0.15))
                    self.comm.Write("Vibration", round(vibration, 3))

                    # Alarm conditions
                    temp_val = self.comm.Read("Temperature").Value or 75.0
                    pressure_val = self.comm.Read("Pressure").Value or 14.7

                    high_temp_alarm = temp_val > 90.0
                    low_pressure_alarm = pressure_val < 12.0

                    self.comm.Write("High_Temp_Alarm", high_temp_alarm)
                    self.comm.Write("Low_Pressure_Alarm", low_pressure_alarm)

                    # Update counters
                    if counter % 60 == 0:  # Every minute
                        prod_count = self.comm.Read("Production_Count").Value or 0
                        self.comm.Write("Production_Count", prod_count + 1)

                    # Runtime hours
                    runtime = self.comm.Read("Runtime_Hours").Value or 0
                    self.comm.Write("Runtime_Hours", round(runtime + 1/3600, 3))

                    counter += 1
                    time.sleep(0.2)

                except Exception as e:
                    print(f"Error in simulation: {e}")
                    time.sleep(1)

        except Exception as e:
            print(f"Error setting up simulation: {e}")
        finally:
            if self.comm:
                self.comm.Close()

    def start_server(self):
        """Start the cpppo EIP server process"""
        tag_defs = self.get_tag_definitions()

        cmd = [
            sys.executable, "-m", "cpppo.server.enip",
            "--address", f"{self.host}:{self.port}",
            "--verbose",
            "--print"
        ] + tag_defs

        print("Starting cpppo EIP server...")
        self.server_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )

        # Give server time to start
        time.sleep(3)
        return self.server_process.poll() is None

    def start(self):
        """Start the complete mock server system"""
        print("Starting Comprehensive Mock EIP Server")
        print("=" * 50)

        self.print_tag_summary()

        # Start the EIP server
        if not self.start_server():
            print("Failed to start EIP server")
            return

        print("EIP server started successfully")

        # Initialize tag values
        time.sleep(2)
        self.initialize_tag_values()

        # Start dynamic simulation
        print("Starting dynamic data simulation...")
        self.running = True
        sim_thread = threading.Thread(target=self.simulate_dynamic_data)
        sim_thread.daemon = True
        sim_thread.start()

        print("\nServer is running! Press Ctrl+C to stop")

        try:
            # Monitor server process
            while self.running and self.server_process.poll() is None:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        """Stop the server and simulation"""
        print("\nStopping server...")
        self.running = False

        if self.server_process:
            self.server_process.terminate()
            try:
                self.server_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.server_process.kill()

        if self.comm:
            self.comm.Close()

        print("Server stopped")

def signal_handler(signum, frame):
    """Handle shutdown signals"""
    if hasattr(signal_handler, 'server'):
        signal_handler.server.stop()
    sys.exit(0)

def main():
    server = ComprehensiveMockServer()
    signal_handler.server = server

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    server.start()

if __name__ == "__main__":
    main()
