"""
================================================================================
IoTextra Analog Module Test Program
================================================================================

Author: Arshia Keshvari
Role: Electrical and Electronics Engineer
Date: October 5, 2025

Project Description:
This program provides a comprehensive test interface for the IoTextra analog 
input module, designed to work with both ESP32-S3 and RP2040 microcontrollers.
The module uses dual ADS1115/ADS1015 ADCs for general high-precision analog measurements.

Features:
- Platform selection (ESP32-S3 or RP2040) with automatic I2C pin configuration
- Dual ADC support (ADS1115 16-bit) at addresses 0x48 and 0x49
- Multiple measurement ranges for voltage (0-0.5V, 0-5V, 0-10V, ±0.5V, ±5V, ±10V)
- Current measurement ranges (0-20mA, 4-20mA, ±20mA, 0-40mA) via shunt resistors (120 Ohm)
- Configurable sampling rates (8-860 SPS)
- Real-time monitoring with range limit detection
- Automatic gain optimization for selected measurement ranges

Hardware Configuration:
- ESP32-S3: I2C on GPIO15 (SCL) and GPIO16 (SDA)
- RP2040: I2C on GPIO5 (SCL) and GPIO4 (SDA)
- Raspberry Pi Pico W: I2C ON GPIO21 (SCL) and GPIO20 (SDA)
- ADC U4 at address 0x48 or 0x4B (Channels 0-1: A0-A1, A2-A3)
- ADC U2 at address 0x49 (Channels 2-3: A0-A1, A2-A3)
- Hardware gain factor: Selectable per channel via jumpers
(default 0.2376 with two 49.9kΩ resistors in parallel; 0.4752 with one resistor)

- Hardware gain factor: 0.237619047619048 (OpAmp Gain 0.237619047619048x)
- Current sensing via 0.249 or 249Ω shunt resistor
- Old Hardware gain factor: 0.2 (OpAmp Gain 0.2x)
- Old Current sensing via 0.12 or 120Ω shunt resistor

Usage:
Run the program and follow the interactive prompts to configure platform,
ADC resolution, measurement range, sampling rate, and channel selection.
The program will then provide real-time monitoring of the selected channel.

Dependencies:
- MicroPython machine module (I2C, Pin)
- ads1x15.py library for ADS1115/ADS1015 ADC communication
- Ensure the lib file is uploaded within a subfolder on either MCU

================================================================================
"""
from machine import I2C, Pin
import time
import sys
import ads1x15

class IoTextraAnalogTest:
    def __init__(self):
        # Platform configuration - will be set by user selection
        self.platform = None
        self.i2c = None
        
        # ADC instances (default addresses 0x48 and u0x49)
        self.adc1 = None  # Will be initialized based on bit depth selection
        self.adc2 = None
        
        # Configuration parameters
        self.bit_depth = 16
        self.measurement_range = None
        self.polling_rate = 128
        self.selected_channel = 0
        
        # Rate mapping (SPS to rate index for ADS1x15 driver)
        self.rate_map = {
            8: 0,    # 128/8 SPS for ADS1115/ADS1015
            16: 1,   # 250/16 SPS
            32: 2,   # 490/32 SPS
            64: 3,   # 920/64 SPS
            128: 4,  # 1600/128 SPS (default)
            250: 5,  # 2400/250 SPS
            475: 6,  # 3300/475 SPS
            860: 7   # -/860 SPS
        }
        
        # ADS1115 gain settings and their corresponding full-scale ranges
        # gain_index: (full_scale_voltage, gain_index_for_init)
        self.ads_gains = {
            0: (6.144, 0),   # ±6.144V, 2/3x gain
            1: (4.096, 1),   # ±4.096V, 1x gain
            2: (2.048, 2),   # ±2.048V, 2x gain
            3: (1.024, 3),   # ±1.024V, 4x gain
            4: (0.512, 4),   # ±0.512V, 8x gain
            5: (0.256, 5)    # ±0.256V, 16x gain
        }
        
        # Measurement range configurations with fixed gain settings
        self.voltage_ranges = {
            '0-0.5V': {
                'type': 'voltage', 
                'min': 0, 
                'max': 0.5, 
                'bipolar': False,
                'ads_gain': 4,  # ±0.512V range - perfect for 0-0.5V with headroom
                'hardware_gain': 0.237619047619048,
                'offset': 0.0
            },
            '0-5V': {
                'type': 'voltage', 
                'min': 0, 
                'max': 5.0, 
                'bipolar': False,
                'ads_gain': 1,
                'hardware_gain': 0.237619047619048,
                'offset': 0.0
            },
            '0-10V': {
                'type': 'voltage', 
                'min': 0, 
                'max': 10.0, 
                'bipolar': False,
                'ads_gain': 0,
                'hardware_gain': 0.237619047619048,
                'offset': 0.0
            },
            '±0.5V': {
                'type': 'voltage', 
                'min': -0.5, 
                'max': 0.5, 
                'bipolar': True,
                'ads_gain': 4,
                'hardware_gain': 0.237619047619048,
                'offset': 0.0
            },
            '±5V': {
                'type': 'voltage', 
                'min': -5.0, 
                'max': 5.0, 
                'bipolar': True,
                'ads_gain': 1,
                'hardware_gain': 0.237619047619048,
                'offset': 0.0
            },
            '±10V': {
                'type': 'voltage', 
                'min': -10.0, 
                'max': 10.0, 
                'bipolar': True,
                'ads_gain': 0,
                'hardware_gain': 0.237619047619048,
                'offset': 0.0
            }
        }
        
        self.current_ranges = {
            '0-20mA': {
                'type': 'current', 
                'min': 0, 
                'max': 20, 
                'bipolar': False,
                'ads_gain': 1,
                'hardware_gain': 0.237619047619048,
                'offset': 0.0,
                'shunt_resistance': 0.249
            },
            '4-20mA': {
                'type': 'current', 
                'min': 4, 
                'max': 20, 
                'bipolar': False,
                'ads_gain': 1,
                'hardware_gain': 0.237619047619048,
                'offset': 0.0,
                'shunt_resistance': 0.249
            },
            '±20mA': {
                'type': 'current', 
                'min': -20, 
                'max': 20, 
                'bipolar': True,
                'ads_gain': 1,
                'hardware_gain': 0.237619047619048,
                'offset': 0.0,
                'shunt_resistance': 0.249
            },
            '0-40mA': {
                'type': 'current', 
                'min': 0, 
                'max': 40, 
                'bipolar': False,
                'ads_gain': 0,
                'hardware_gain': 0.237619047619048,
                'offset': 0.0,
                'shunt_resistance': 0.249
            }
        }

    def print_banner(self):
        print("\n" + "="*60)
        print("       IoTextra Analog Module Test Program")
        print("    Compatible with IoTsmart RP2040 & ESP32-S3")
        print("         Fixed Gain Range Configuration")
        print("="*60)

    def configure_platform(self):
        """Configure the microcontroller platform for I2C communication"""
        print("\n--- Platform Configuration ---")
        print("Select your microcontroller platform:")
        print("1. ESP32-S3 (SCL=Pin 15, SDA=Pin 16) - Default")
        print("2. RP2040 (SCL=Pin 5, SDA=Pin 4)")
        print("3. Raspberry Pi Pico W (SCL=Pin 21, SDA=Pin 20)")
        
        while True:
            try:
                choice = input("Select platform (1-3, default=1): ").strip()
                if choice == '' or choice == '1':
                    self.platform = 'ESP32-S3'
                    # Initialize I2C bus for ESP32-S3
                    self.i2c = I2C(0, scl=Pin(15), sda=Pin(16), freq=400000)
                    print("Selected: ESP32-S3 (SCL=GPIO15, SDA=GPIO16)")
                    break
                elif choice == '2':
                    self.platform = 'RP2040'
                    # Initialize I2C bus for RP2040
                    self.i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
                    print("Selected: RP2040 (SCL=GP5, SDA=GP4)")
                    break
                elif choice == '3':
                    self.platform = 'Raspberry Pi Pico W'
                    # Initialize I2C bus for Raspberry Pi Pico W
                    self.i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq=400000)
                    print("Selected: Raspberry Pi Pico W (SCL=GPIO21, SDA=GPIO20)")
                    break
                else:
                    print("Invalid choice. Please enter 1 or 2.")
            except KeyboardInterrupt:
                print("\nConfiguration interrupted.")
                sys.exit()
            except Exception as e:
                print(f"Error initializing I2C: {e}")
                print("Please check your pin connections and try again.")

    def configure_bit_depth(self):
        print("\n--- ADC Bit Depth Configuration ---")
        print("1. 16-bit (ADS1115) - Default")
        print("2. 12-bit (ADS1015)")
        
        while True:
            try:
                choice = input("Select bit depth (1-2, default=1): ").strip()
                if choice == '' or choice == '1':
                    self.bit_depth = 16
                    print("Selected: 16-bit ADC (ADS1115)")
                    break
                elif choice == '2':
                    self.bit_depth = 12
                    print("Selected: 12-bit ADC (ADS1015)")
                    break
                else:
                    print("Invalid choice. Please enter 1 or 2.")
            except KeyboardInterrupt:
                print("\nConfiguration interrupted.")
                sys.exit()

    def initialize_adcs(self):
        """Initialize ADCs based on selected bit depth"""
        if self.i2c is None:
            print("Error: I2C bus not initialized. Please configure platform first.")
            sys.exit()
            
        try:
            if self.bit_depth == 16:
                # Initialize with default gain - will be recreated with proper gain later
                self.adc1 = ads1x15.ADS1115(self.i2c, 0x48, gain=1)
                self.adc2 = ads1x15.ADS1115(self.i2c, 0x49, gain=1)
                self.adc3 = ads1x15.ADS1115(self.i2c, 0x4A, gain=1)
                self.adc4 = ads1x15.ADS1115(self.i2c, 0x4B, gain=1)
                print(f"ADS1115 (16-bit) ADCs initialized on {self.platform} at addresses 0x48 and 0x49")
            else:
                self.adc1 = ads1x15.ADS1015(self.i2c, 0x48, gain=1)
                self.adc2 = ads1x15.ADS1015(self.i2c, 0x49, gain=1)
                self.adc3 = ads1x15.ADS1015(self.i2c, 0x4A, gain=1)
                self.adc4 = ads1x15.ADS1015(self.i2c, 0x4B, gain=1)
                print(f"ADS1015 (12-bit) ADCs initialized on {self.platform} at addresses 0x48 and 0x49")
        except Exception as e:
            print(f"Error initializing ADCs: {e}")
            print("Please check I2C connections and ADC addresses.")
            print(f"Current platform: {self.platform}")
            if self.platform == 'ESP32-S3':
                print("Expected connections: SCL=GPIO15, SDA=GPIO16")
            elif self.platform == 'Raspberry Pi Pico W':
                print("Expected connections: SCL=GPIO21, SDA=GPIO20")
            else:
                print("Expected connections: SCL=GP5, SDA=GP4")
            sys.exit()

    def configure_measurement_range(self):
        print("\n--- Measurement Range Configuration ---")
        print("Voltage Ranges:")
        volt_options = list(self.voltage_ranges.keys())
        for i, option in enumerate(volt_options, 1):
            range_info = self.voltage_ranges[option]
            ads_range = self.ads_gains[range_info['ads_gain']][0]
            print(f"  {i}. {option} (ADC range: ±{ads_range}V)")
        
        print("Current Ranges:")
        curr_options = list(self.current_ranges.keys())
        for i, option in enumerate(curr_options, len(volt_options) + 1):
            range_info = self.current_ranges[option]
            ads_range = self.ads_gains[range_info['ads_gain']][0]
            print(f"  {i}. {option} (ADC range: ±{ads_range}V)")
        
        while True:
            try:
                choice = int(input(f"Select measurement range (1-{len(volt_options) + len(curr_options)}): "))
                if 1 <= choice <= len(volt_options):
                    range_key = volt_options[choice - 1]
                    self.measurement_range = self.voltage_ranges[range_key].copy()
                    self.measurement_range['key'] = range_key
                    print(f"Selected: {range_key}")
                    break
                elif len(volt_options) < choice <= len(volt_options) + len(curr_options):
                    range_key = curr_options[choice - len(volt_options) - 1]
                    self.measurement_range = self.current_ranges[range_key].copy()
                    self.measurement_range['key'] = range_key
                    print(f"Selected: {range_key}")
                    break
                else:
                    print("Invalid choice.")
            except (ValueError, KeyboardInterrupt):
                if KeyboardInterrupt:
                    print("\nConfiguration interrupted.")
                    sys.exit()
                print("Invalid input. Please enter a number.")

    def configure_polling_rate(self):
        print("\n--- Polling Rate Configuration ---")
        available_rates = [8, 16, 32, 64, 128, 250, 475, 860]
        for i, rate in enumerate(available_rates, 1):
            default_marker = " (Default)" if rate == 128 else ""
            print(f"  {i}. {rate} SPS{default_marker}")
        
        while True:
            try:
                choice = input(f"Select polling rate (1-{len(available_rates)}, default=5): ").strip()
                if choice == '' or choice == '5':
                    self.polling_rate = 128
                    print("Selected: 128 SPS (Default)")
                    break
                else:
                    choice_idx = int(choice) - 1
                    if 0 <= choice_idx < len(available_rates):
                        self.polling_rate = available_rates[choice_idx]
                        print(f"Selected: {self.polling_rate} SPS")
                        break
                    else:
                        print("Invalid choice.")
            except (ValueError, KeyboardInterrupt):
                if KeyboardInterrupt:
                    print("\nConfiguration interrupted.")
                    sys.exit()
                print("Invalid input. Please enter a number.")

    def select_channel(self):
        print("\n--- Channel Selection ---")
        print("Available channels (differential pairs):")
        print("  0. Channel 0 (A0-A1) on ADC U4 (0x48)")
        print("  1. Channel 1 (A2-A3) on ADC U4 (0x48)")
        print("  2. Channel 2 (A0-A1) on ADC U2 (0x49)")
        print("  3. Channel 3 (A2-A3) on ADC U2 (0x49)")
        print("  4. Channel 4 (A0-A1) on ADC U3 (0x4A)")
        print("  5. Channel 5 (A2-A3) on ADC U3 (0x4A)")
        print("  6. Channel 6 (A0-A1) on ADC U4 (0x4B)")
        print("  7. Channel 7 (A2-A3) on ADC U4 (0x4B)")
        
        while True:
            try:
                choice = int(input("Select channel (0-7): "))
                if 0 <= choice <= 7:
                    self.selected_channel = choice
                    print(f"Selected: Channel {choice}")
                    break
                else:
                    print("Invalid choice. Please enter 0-7.")
            except (ValueError, KeyboardInterrupt):
                if KeyboardInterrupt:
                    print("\nConfiguration interrupted.")
                    sys.exit()
                print("Invalid input. Please enter a number.")
                
    def configure_hardware_gain(self):
        """Configure the hardware gain (division factor) based on jumper settings for the selected channel"""
        print("\n--- Hardware Gain Configuration ---")
        print("Select the division factor (hardware gain) for this channel (set by jumpers):")
        print("1. Default: Two 49.9kΩ resistors in parallel (24.95kΩ total, K ≈ 0.23761904761904762) - Default")
        print("2. Modified: One 49.9kΩ resistor (K ≈ 0.47523809523809524, requires cutting jumpers)")
        
        while True:
            try:
                choice = input("Select gain option (1-2, default=1): ").strip()
                if choice == '' or choice == '1':
                    self.measurement_range['hardware_gain'] = 0.23761904761904762 # ≈0.23761904761904762 or 24.95 / 105
                    print("Selected: Default hardware gain (≈0.2376)")
                    break
                elif choice == '2':
                    self.measurement_range['hardware_gain'] = 0.47523809523809524  # ≈0.47523809523809524 or 49.9 / 105
                    print("Selected: Modified hardware gain (≈0.4752)")
                    break
                else:
                    print("Invalid choice. Please enter 1 or 2.")
            except KeyboardInterrupt:
                print("\nConfiguration interrupted.")
                sys.exit()

    def reinitialize_adcs_with_gain(self):
        """Reinitialize ADCs with the correct gain for selected measurement range"""
        gain_index = self.ads_gains[self.measurement_range['ads_gain']][1]  # Get gain index for initialization
        
        try:
            if self.bit_depth == 16:
                self.adc1 = ads1x15.ADS1115(self.i2c, 0x48, gain=gain_index)
                self.adc2 = ads1x15.ADS1115(self.i2c, 0x49, gain=gain_index)
                self.adc3 = ads1x15.ADS1115(self.i2c, 0x4A, gain=gain_index)
                self.adc4 = ads1x15.ADS1115(self.i2c, 0x4B, gain=gain_index)
                print(f"ADS1115 ADCs reinitialized with gain index {gain_index} for range {self.measurement_range['key']}")
            else:
                self.adc1 = ads1x15.ADS1015(self.i2c, 0x48, gain=gain_index)
                self.adc2 = ads1x15.ADS1015(self.i2c, 0x49, gain=gain_index)
                self.adc3 = ads1x15.ADS1115(self.i2c, 0x4A, gain=gain_index)
                self.adc4 = ads1x15.ADS1115(self.i2c, 0x4B, gain=gain_index)
                print(f"ADS1015 ADCs reinitialized with gain index {gain_index} for range {self.measurement_range['key']}")
        except Exception as e:
            print(f"Error reinitializing ADCs with gain: {e}")
            sys.exit()

    def set_adc_gain(self, adc, gain_setting):
        """Set the gain for the specified ADC - removed as not supported"""
        pass

    def read_channel_raw(self, channel):
        """Read raw value from specified channel with appropriate gain"""
        rate_idx = self.rate_map[self.polling_rate]
        
        try:
            if channel == 0:  # A0-A1 on ADC1
                return self.adc1.read(rate=rate_idx, channel1=0, channel2=1)
            elif channel == 1:  # A2-A3 on ADC1
                return self.adc1.read(rate=rate_idx, channel1=2, channel2=3)
            elif channel == 2:  # A0-A1 on ADC2
                return self.adc2.read(rate=rate_idx, channel1=0, channel2=1)
            elif channel == 3:  # A2-A3 on ADC2
                return self.adc2.read(rate=rate_idx, channel1=2, channel2=3)
            elif channel == 4:  # A0-A1 on ADC3
                return self.adc3.read(rate=rate_idx, channel1=0, channel2=1)
            elif channel == 5:  # A2-A3 on ADC3
                return self.adc3.read(rate=rate_idx, channel1=2, channel2=3)
            elif channel == 6:  # A0-A1 on ADC4
                return self.adc4.read(rate=rate_idx, channel1=0, channel2=1)
            elif channel == 7:  # A2-A3 on ADC4
                return self.adc4.read(rate=rate_idx, channel1=2, channel2=3)
        except Exception as e:
            print(f"Error reading channel {channel}: {e}")
            return None

    def convert_raw_to_physical(self, raw_value, channel):
        """Convert raw ADC value to physical measurement"""
        if raw_value is None:
            return None
        
        # Get ADC voltage using the appropriate ADC instance
        if channel < 2:
            voltage = self.adc1.raw_to_v(raw_value)
        elif channel < 4:
            voltage = self.adc2.raw_to_v(raw_value)
        elif channel < 6:
            voltage = self.adc3.raw_to_v(raw_value)
        else:
            voltage = self.adc4.raw_to_v(raw_value)
        
        # Apply hardware scaling
        scaled_voltage = voltage / self.measurement_range['hardware_gain']
        
        if self.measurement_range['type'] == 'voltage':
            # Direct voltage measurement
            physical_value = scaled_voltage
        else:
            # Current measurement through shunt resistor
            shunt_r = self.measurement_range['shunt_resistance']
            physical_value = (scaled_voltage / shunt_r)
        
        # Apply offset
        physical_value += self.measurement_range['offset']
        
        # Range limiting - clamp to specified range
        range_min = self.measurement_range['min']
        range_max = self.measurement_range['max']
        
        if physical_value < range_min:
            physical_value = range_min
        elif physical_value > range_max:
            physical_value = range_max
        
        return physical_value

    def format_physical_value(self, value):
        """Format physical value with appropriate units"""
        if value is None:
            return "ERROR"
        
        if self.measurement_range['type'] == 'voltage':
            return f"{value:.4f} V"
        else:
            return f"{value:.4f} mA"

    def check_range_limits(self, physical_value):
        """Check if measurement is within specified range and warn if at limits"""
        if physical_value is None:
            return "ERROR"
        
        range_min = self.measurement_range['min']
        range_max = self.measurement_range['max']
        
        status = ""
        if physical_value <= range_min + 0.001:  # Near minimum
            status = " [MIN]"
        elif physical_value >= range_max - 0.001:  # Near maximum
            status = " [MAX]"
        
        return status

    def monitor_channel(self):
        """Continuously monitor selected channel"""
        print(f"\n--- Monitoring Channel {self.selected_channel} ---")
        print(f"Platform: {self.platform}")
        if self.platform == 'ESP32-S3':
            print("I2C Pins: SCL=GPIO15, SDA=GPIO16")
        elif self.platform == 'Raspberry Pi Pico W':
            print("I2C Pins: SCL=GPIO21, SDA=GPIO20")
        else:
            print("I2C Pins: SCL=GP5, SDA=GP4")
        print(f"Range: {self.measurement_range['key']}")
        print(f"Polling Rate: {self.polling_rate} SPS")
        print(f"Bit Depth: {self.bit_depth}-bit")
        print(f"ADC Gain: {self.measurement_range['ads_gain']} (±{self.ads_gains[self.measurement_range['ads_gain']][0]}V)")
        print(f"Hardware Gain: {self.measurement_range['hardware_gain']}")
        print("\nPress Ctrl+C to stop monitoring...")
        print("\nCh | Raw Value | Physical Value | Range")
        print("-" * 65)
        
        try:
            while True:
                raw_value = self.read_channel_raw(self.selected_channel)
                physical_value = self.convert_raw_to_physical(raw_value, self.selected_channel)
                physical_str = self.format_physical_value(physical_value)
                status = self.check_range_limits(physical_value)
                range_str = self.measurement_range['key']
                
                # Handle None values properly
                if raw_value is None:
                    raw_str = "ERROR"
                else:
                    raw_str = str(raw_value)
                
                print(f" {self.selected_channel} | {raw_str:>9} | {physical_str:>13} | {range_str:<12}")
                
                time.sleep(1.0 / (self.polling_rate / 10))  # Adjust sleep based on polling rate
                
        except KeyboardInterrupt:
            print("\nMonitoring stopped.")

    def run_test(self):
        """Main test execution"""
        self.print_banner()
        
        # Configuration phase
        self.configure_platform()  # platform selection step
        self.configure_bit_depth()
        self.initialize_adcs()
        self.configure_measurement_range()
        self.configure_polling_rate()
        self.select_channel()
        self.configure_hardware_gain()  # New hardware gain selection step
        
        # Reinitialize ADCs with correct gain for selected range
        self.reinitialize_adcs_with_gain()
        
        # Monitoring phase
        self.monitor_channel()
        
        print("\nTest completed.")

# Main execution
if __name__ == "__main__":
    test = IoTextraAnalogTest()
    test.run_test()