"""
================================================================================
IoTextra Analog - interactive test (MicroPython)
================================================================================
Module : IoTextra Analog Rev.3-11 / 3-12 (2x ADS1115, 4 differential channels)
Hosts  : IoTsmart ESP32-S3, IoTsmart RP2040, Raspberry Pi Pico W (IoTbase PICO)
Library: ads1x15.py (robert-hh, MIT) - copy to the board root or /lib

Measurement chain (datasheet Rev.3-12, schematic 3-12):
  Op-amp output  = VREF(2.5 V) + K * Vin,   K = R / 105 kOhm
  ADC (diff.)    = ADC_INP - ADC_INN = K * Vin   (ADC_INN = VREF)
  Current input  : Vin = I * 249 Ohm
  R per channel  : 24.95 kOhm (default, both jumpers closed) -> K = 0.237619
                   49.9  kOhm (both jumpers of the pair open) -> K = 0.475238

ADC addresses (auto-detected at start-up):
  CH1, CH2 -> U2: 0x49 (default) or 0x48 (SB2 closed)
  CH3, CH4 -> U4: 0x4B (default) or 0x4A (SB3 open, SB4 closed)
  CH1/CH3 = AIN0-AIN1, CH2/CH4 = AIN2-AIN3

The PGA is chosen automatically: smallest full-scale range that covers
K * |Vin|max plus 10 % headroom, so over-range can still be detected.
Values are NOT clamped: real readings are shown and flagged UNDER / OVER.
================================================================================
"""
from machine import I2C, Pin
import time
import ads1x15

# ------------------------------------------------------------------ constants
PLATFORMS = (
    # name,                        bus, scl, sda
    ("IoTsmart ESP32-S3",            0,  15,  16),
    ("IoTsmart RP2040",              0,   5,   4),
    ("Raspberry Pi Pico W (IoTbase PICO)", 0, 21, 20),
)
I2C_FREQ = 400000

K_BY_R = {
    "24.95k": 24.95 / 105.0,   # 0.237619 (default)
    "49.9k":  49.9 / 105.0,    # 0.475238
}
SHUNT_KOHM = 0.249             # 249 Ohm; V / kOhm = mA

VREF = 2.5                     # REF3425
OPAMP_OUT_MIN = 0.05           # usable op-amp output swing on 5 V_AN
OPAMP_OUT_MAX = 4.95

# ADS1115 PGA: index -> full-scale range (V), same order as ads1x15._GAINS
PGA_FSR = (6.144, 4.096, 2.048, 1.024, 0.512, 0.256)
PGA_HEADROOM = 1.10

# ADS1115 data rate (SPS) -> ads1x15 rate index
RATES = (8, 16, 32, 64, 128, 250, 475, 860)
DEFAULT_RATE = 128

PRINT_INTERVAL_S = 0.5         # console refresh, independent of ADC rate

# name: (type, min, max)   units: V or mA
RANGES = (
    ("0-0.5V",  "V",  0.0,   0.5),
    ("+-0.5V",  "V", -0.5,   0.5),
    ("0-5V",    "V",  0.0,   5.0),
    ("+-5V",    "V", -5.0,   5.0),
    ("0-10V",   "V",  0.0,  10.0),
    ("+-10V",   "V", -10.0, 10.0),
    ("0-20mA",  "mA", 0.0,  20.0),
    ("4-20mA",  "mA", 4.0,  20.0),
    ("+-20mA",  "mA", -20.0, 20.0),
    ("0-40mA",  "mA", 0.0,  40.0),
)

# channel -> (ADC group, mux pair)
CHANNELS = {
    1: ("U2", (0, 1)),
    2: ("U2", (2, 3)),
    3: ("U4", (0, 1)),
    4: ("U4", (2, 3)),
}
ADDR_OPTIONS = {
    "U2": (0x49, 0x48),   # default first
    "U4": (0x4B, 0x4A),
}


# ------------------------------------------------------------------ helpers
def ask_choice(title, options, default=None):
    """Print numbered options and return the selected index (0-based).
    Re-prompts on invalid input; Ctrl+C propagates to the caller."""
    print("\n--- {} ---".format(title))
    for i, text in enumerate(options, 1):
        mark = " (default)" if default is not None and i - 1 == default else ""
        print("  {}. {}{}".format(i, text, mark))
    while True:
        s = input("Select 1-{}{}: ".format(
            len(options), "" if default is None else ", Enter = {}".format(default + 1))).strip()
        if s == "" and default is not None:
            return default
        try:
            n = int(s)
        except ValueError:
            print("Please enter a number.")
            continue
        if 1 <= n <= len(options):
            return n - 1
        print("Out of range.")


def to_vin(rng_type, value):
    """Physical value -> voltage at the module input terminals."""
    return value * SHUNT_KOHM if rng_type == "mA" else value


def pick_pga(k, vin_min, vin_max):
    """Smallest FSR covering K*|Vin|max with headroom. Returns PGA index."""
    need = k * max(abs(vin_min), abs(vin_max)) * PGA_HEADROOM
    best = 0
    for idx, fsr in enumerate(PGA_FSR):
        if fsr >= need:
            best = idx
    return best


def opamp_ok(k, vin_min, vin_max):
    lo = VREF + k * vin_min
    hi = VREF + k * vin_max
    return OPAMP_OUT_MIN <= lo and hi <= OPAMP_OUT_MAX, lo, hi


# ------------------------------------------------------------------ test
class IoTextraAnalogTest:
    def __init__(self):
        self.i2c = None
        self.adcs = {}          # "U2"/"U4" -> ADS1115 instance
        self.range = None       # tuple from RANGES
        self.k = None
        self.pga = None
        self.rate_idx = RATES.index(DEFAULT_RATE)
        self.channels = []

    # -- configuration
    def configure_platform(self):
        idx = ask_choice("Host platform",
                         ["{} (SCL=GP{}, SDA=GP{})".format(n, scl, sda)
                          for n, _, scl, sda in PLATFORMS], default=0)
        name, bus, scl, sda = PLATFORMS[idx]
        self.i2c = I2C(bus, scl=Pin(scl), sda=Pin(sda), freq=I2C_FREQ)
        print("Using {}".format(name))

    def detect_adcs(self):
        found = self.i2c.scan()
        print("\nI2C scan: {}".format([hex(a) for a in found]))
        for grp, addrs in ADDR_OPTIONS.items():
            addr = next((a for a in addrs if a in found), None)
            if addr is None:
                print("  {}: not found (expected {}) - its channels are disabled".format(
                    grp, " or ".join(hex(a) for a in addrs)))
                continue
            self.adcs[grp] = ads1x15.ADS1115(self.i2c, addr)
            note = "default" if addr == addrs[0] else "alternate jumper setting"
            print("  {}: ADS1115 at {} ({})".format(grp, hex(addr), note))
        if not self.adcs:
            raise RuntimeError("No ADS1115 found - check module seating and I2C pins")

    def configure_range_and_r(self):
        while True:
            r_idx = ask_choice("Range", [r[0] for r in RANGES])
            self.range = RANGES[r_idx]
            r_names = list(K_BY_R.keys())
            k_idx = ask_choice("Gain resistor R (jumper pair of the channel)",
                               ["24.95 kOhm - both jumpers closed (K=0.2376)",
                                "49.9 kOhm - both jumpers open (K=0.4752)"], default=0)
            self.k = K_BY_R[r_names[k_idx]]

            _, typ, vmin, vmax = self.range
            vin_min, vin_max = to_vin(typ, vmin), to_vin(typ, vmax)
            ok, lo, hi = opamp_ok(self.k, vin_min, vin_max)
            if not ok:
                print("\n!! {} with R={} drives the op-amp to {:.2f}...{:.2f} V,"
                      " outside {:.2f}...{:.2f} V. Use R=24.95k for this range.".format(
                          self.range[0], r_names[k_idx], lo, hi, OPAMP_OUT_MIN, OPAMP_OUT_MAX))
                continue
            self.pga = pick_pga(self.k, vin_min, vin_max)
            for adc in self.adcs.values():
                adc.gain = self.pga
            print("PGA: +-{} V (op-amp output {:.3f}...{:.3f} V)".format(
                PGA_FSR[self.pga], lo, hi))
            return

    def configure_rate(self):
        self.rate_idx = ask_choice("ADC data rate",
                                   ["{} SPS".format(r) for r in RATES],
                                   default=RATES.index(DEFAULT_RATE))

    def configure_channels(self):
        avail = [ch for ch, (grp, _) in CHANNELS.items() if grp in self.adcs]
        opts = ["CH{}".format(ch) for ch in avail] + ["All available channels"]
        idx = ask_choice("Channel", opts, default=len(opts) - 1)
        self.channels = avail if idx == len(avail) else [avail[idx]]

    # -- measurement
    def read_channel(self, ch):
        """Returns (raw, physical value) or (None, None) on I2C error."""
        grp, (a, b) = CHANNELS[ch]
        adc = self.adcs[grp]
        try:
            raw = adc.read(rate=self.rate_idx, channel1=a, channel2=b)
        except OSError as e:
            print("CH{}: I2C error {}".format(ch, e))
            return None, None
        vin = adc.raw_to_v(raw) / self.k
        value = vin / SHUNT_KOHM if self.range[1] == "mA" else vin
        return raw, value

    def status(self, raw, value):
        name, typ, vmin, vmax = self.range
        if raw >= 32767 or raw <= -32768:
            return "ADC SATURATED"
        if name == "4-20mA":                     # NAMUR NE43 limits
            if value < 3.6:
                return "UNDER (<3.6 mA, wire break?)"
            if value > 21.0:
                return "OVER (>21 mA, sensor fault?)"
            return ""
        span = vmax - vmin
        if value < vmin - 0.02 * span:
            return "UNDER"
        if value > vmax + 0.02 * span:
            return "OVER"
        return ""

    def monitor(self):
        name, typ, _, _ = self.range
        print("\n--- Monitoring {} | R: K={:.4f} | PGA +-{} V | {} SPS ---".format(
            name, self.k, PGA_FSR[self.pga], RATES[self.rate_idx]))
        print("Ctrl+C to stop.\n")
        print("Ch  |    Raw |      Value | Status")
        print("-" * 50)
        while True:
            for ch in self.channels:
                raw, value = self.read_channel(ch)
                if raw is None:
                    continue
                print("CH{} | {:>6} | {:>8.4f} {:<2}| {}".format(
                    ch, raw, value, typ, self.status(raw, value)))
            if len(self.channels) > 1:
                print("-" * 50)
            time.sleep(PRINT_INTERVAL_S)

    def run(self):
        print("\n" + "=" * 60)
        print("       IoTextra Analog - interactive test")
        print("=" * 60)
        try:
            self.configure_platform()
            self.detect_adcs()
            self.configure_range_and_r()
            self.configure_rate()
            self.configure_channels()
            self.monitor()
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    IoTextraAnalogTest().run()
