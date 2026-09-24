import serial
import json
from pathlib import Path

SETTINGS_PATH = Path(__file__).parent / 'pedal_settings.json'


class Pedal:

    def __init__(self, port: str = '/dev/ttyACM0'):
        self.ser = serial.Serial(port, 115200, timeout=None)
        with open(SETTINGS_PATH, 'r', encoding='utf-8') as f:
            settings = json.load(f)
            self.min = settings['min']
            self.max = settings['max']
        

    def _read_pot(self) -> int:
        self.ser.reset_input_buffer()
        self.ser.readline()
        while True:
            line = self.ser.readline().decode().strip()
            if line:
                break
        val = int(line)
        return val

    def calibrate(self):
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

    def read_pedal(self):
        raw_val = self._read_pot()
        if raw_val >= self.max:
            return 1
        if raw_val <= self.min:
            return 0
        range = self.max - self.min
        val_in_range = raw_val - self.min
        return val_in_range / range