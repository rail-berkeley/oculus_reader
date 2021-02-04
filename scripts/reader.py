from ppadb.client import Client as AdbClient
from ppadb.client import Client
import sys
if sys.version_info[0] == 2:
    from FPS_counter import FPSCounter
    from install import get_device
    from buttons_parser import parse_buttons
else:
    from .FPS_counter import FPSCounter
    from .install import get_device
    from .buttons_parser import parse_buttons

import numpy as np
import threading
import time


class OculusReader:
    def __init__(self, print_FPS=False):
        self.last_pose_arrays = []
        self.last_buttons = {}
        self._lock = threading.Lock()
        self.running = True
        self.tag = 'wE9ryARX'

        self.print_FPS = print_FPS
        if print_FPS:
            self.fps_counter = FPSCounter()

        device = get_device()
        device.shell('am start -n "com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity" -a android.intent.action.MAIN -c android.intent.category.LAUNCHER')
        self.thread = threading.Thread(target=device.shell, args=("logcat -T 0", self.read_logcat_by_line))
        self.thread.start()

    def __del__(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()

    @staticmethod
    def process_data(string):
        try:
            array_strings, buttons_string = string.split('&')
        except ValueError:
            return None, None
        split_array_strings = array_strings.split('|')
        arrays = []
        for array_string in split_array_strings:
            array = np.empty((4,4))
            values = array_string.split(' ')
            c = 0
            r = 0
            count = 0
            for value in values:
                if not value:
                    continue
                array[r][c] = float(value)
                c += 1
                if c >= 4:
                    c = 0
                    r += 1
                count += 1
            if count == 16:
                arrays.append(array)
        buttons = parse_buttons(buttons_string)
        return arrays, buttons

    def extract_data(self, line):
        output = ''
        if self.tag in line:
            try:
                output += line.split(self.tag + ': ')[1]
            except ValueError:
                pass
        return output

    def get_arays_and_button(self):
        with self._lock:
            return self.last_pose_arrays, self.last_buttons

    def read_logcat_by_line(self, connection):
        file_obj = connection.socket.makefile()
        while self.running:
            try:
                line = file_obj.readline().strip()
                data = self.extract_data(line)
                if data:
                    pose_arrays, buttons = OculusReader.process_data(data)
                    with self._lock:
                        self.last_pose_arrays, self.last_buttons = pose_arrays, buttons
                    if self.print_FPS:
                        self.fps_counter.getAndPrintFPS()
            except UnicodeDecodeError:
                pass
        file_obj.close()
        connection.close()


def main():
    oculus_reader = OculusReader()

    while True:
        time.sleep(3)
        print(oculus_reader.get_arays_and_button())


if __name__ == '__main__':
    main()