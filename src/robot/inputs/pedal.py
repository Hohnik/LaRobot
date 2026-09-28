import serial
import json
from pathlib import Path

SETTINGS_PATH = Path(__file__).parent / 'pedal_settings.json'


class Pedal:
    """Read a serial pedal and normalize its calibrated potentiometer values.

    Opens the serial connection at 115200 baud without a read timeout and loads
    the minimum and maximum from pedal_settings.json beside this module.

    Parameters
    ----------
    port : str, optional
        Serial device path; defaults to /dev/ttyACM0.
    """

    def __init__(self, port: str = '/dev/ttyACM0'):
        self.ser = serial.Serial(port, 115200, timeout=None)
        with open(SETTINGS_PATH, 'r', encoding='utf-8') as f:
            settings = json.load(f)
            self.min = settings['min']
            self.max = settings['max']
        

    def _read_pot(self) -> int:
        """Wait for a fresh potentiometer reading from the serial stream.

        Clears buffered input and discards the first line, which may be partial.
        Then waits without a timeout for a non-empty line containing an integer.

        Returns
        -------
        Raw potentiometer value before calibration or clipping.
        ```
        int
        ```
        """
        self.ser.reset_input_buffer()
        self.ser.readline()
        while True:
            line = self.ser.readline().decode().strip()
            if line:
                break
        val = int(line)
        return val

    def calibrate(self):
        """Prompt for the pedal's minimum and maximum positions and save them.

        Press Enter at each position to sample its value. Repeats both prompts
        until the minimum is lower than the maximum, then updates the active
        limits and overwrites pedal_settings.json beside this module.
        """
        while True:
            print("Select minimal setting and press enter!")
            input()
            new_min = self._read_pot()

            print("\nSelect maximal setting and press enter!")
            input()
            new_max = self._read_pot()
            if new_min < new_max:
                break
            print("Minimum setting has to be lower than max!")
        print("Succes!")
        self.max = new_max
        self.min = new_min 

        with open(SETTINGS_PATH, "w", encoding='utf-8') as f:
            settings = {}
            settings['max'] = new_max
            settings['min'] = new_min
            json.dump(settings, f, ensure_ascii=False, indent=4)

    def read(self):
        """Wait for a fresh reading and map it to the calibrated range.

        Returns
        -------
        Pedal position scaled linearly between the calibrated limits. Values at
        or below the minimum return 0; values at or above the maximum return 1.
        ```
        int | float
        ```
        """
        raw_val = self._read_pot()
        if raw_val >= self.max:
            return 1
        if raw_val <= self.min:
            return 0
        range = self.max - self.min
        val_in_range = raw_val - self.min
        return val_in_range / range