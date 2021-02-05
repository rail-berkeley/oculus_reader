if __name__ == '__main__':
    from FPS_counter import FPSCounter
    from install import get_device, install
    from buttons_parser import parse_buttons
else:
    from .FPS_counter import FPSCounter
    from .install import get_device, install
    from .buttons_parser import parse_buttons

import numpy as np
import threading
import time


class OculusReader:
    def __init__(self, print_FPS=False):
        self.last_transforms = {}
        self.last_buttons = {}
        self._lock = threading.Lock()
        self.running = True
        self.tag = 'wE9ryARX'

        self.print_FPS = print_FPS
        if print_FPS:
            self.fps_counter = FPSCounter()

        device = get_device()
        install(device, verbose=False)
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
            transforms_string, buttons_string = string.split('&')
        except ValueError:
            return None, None
        split_transform_strings = transforms_string.split('|')
        transforms = {}
        for pair_string in split_transform_strings:
            transform = np.empty((4,4))
            pair = pair_string.split(':')
            if len(pair) != 2:
                continue
            left_right_char = pair[0] # is r or l
            transform_string = pair[1]
            values = transform_string.split(' ')
            c = 0
            r = 0
            count = 0
            for value in values:
                if not value:
                    continue
                transform[r][c] = float(value)
                c += 1
                if c >= 4:
                    c = 0
                    r += 1
                count += 1
            if count == 16:
                transforms[left_right_char] = transform
        buttons = parse_buttons(buttons_string)
        return transforms, buttons

    def extract_data(self, line):
        output = ''
        if self.tag in line:
            try:
                output += line.split(self.tag + ': ')[1]
            except ValueError:
                pass
        return output

    def get_transformations_and_buttons(self):
        with self._lock:
            return self.last_transforms, self.last_buttons

    def read_logcat_by_line(self, connection):
        file_obj = connection.socket.makefile()
        while self.running:
            try:
                line = file_obj.readline().strip()
                data = self.extract_data(line)
                if data:
                    transforms, buttons = OculusReader.process_data(data)
                    with self._lock:
                        self.last_transforms, self.last_buttons = transforms, buttons
                    if self.print_FPS:
                        self.fps_counter.getAndPrintFPS()
            except UnicodeDecodeError:
                pass
        file_obj.close()
        connection.close()


def main():
    oculus_reader = OculusReader()

    while True:
        time.sleep(0.3)
        print(oculus_reader.get_transformations_and_buttons())


if __name__ == '__main__':
    main()