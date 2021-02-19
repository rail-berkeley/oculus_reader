from oculus_reader.FPS_counter import FPSCounter
from oculus_reader.buttons_parser import parse_buttons
import numpy as np
import threading
import time
import os
from ppadb.client import Client as AdbClient


class OculusReader:
    def __init__(self,
            ip_address,
            port = 5555,
            APK_name='com.rail.oculus.teleop',
            print_FPS=False,
            run=True
        ):
        self.running = False
        self.last_transforms = {}
        self.last_buttons = {}
        self._lock = threading.Lock()
        self.tag = 'wE9ryARX'

        self.ip_address = ip_address
        self.port = port
        self.APK_name = APK_name
        self.print_FPS = print_FPS
        if self.print_FPS:
            self.fps_counter = FPSCounter()

        self.device = self.get_device()
        self.install(verbose=False)
        if run:
            self.run()

    def __del__(self):
        self.stop()

    def run(self):
        self.running = True
        self.device.shell('am start -n "com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity" -a android.intent.action.MAIN -c android.intent.category.LAUNCHER')
        self.thread = threading.Thread(target=self.device.shell, args=("logcat -T 0", self.read_logcat_by_line))
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()

    def get_device(self, retry=0):
        # Default is "127.0.0.1" and 5037
        client = AdbClient(host="127.0.0.1", port=5037)
        try:
            client.remote_connect(self.ip_address, self.port)
        except RuntimeError:
            os.system('adb devices')
            client.remote_connect(self.ip_address, self.port)

        device = client.device(self.ip_address + ':' + str(self.port))

        if device is None:
            if retry==1:
                os.system('adb tcpip ' + str(self.port))
            if retry==2:
                print('Make sure that device is running and is available at the IP address specified as the OculusReader argument `ip_address`.')
                print('Currently provided IP address:', self.ip_address)
                print('Run `adb shell ip route` to verify the IP address.')
                exit(1)
            else:
                self.get_device(retry=retry+1)
        return device

    def install(self, APK_path='APK/teleop-debug.apk', verbose=True, reinstall=False):
        installed = self.device.is_installed(self.APK_name)
        if not installed or reinstall:
            success = self.device.install(APK_path, test=True, reinstall=reinstall)
            installed = self.device.is_installed(self.APK_name)
            if installed and success:
                print('APK installed successfully.')
            else:
                print('APK install failed.')
        elif verbose:
            print('APK is already installed.')

    def uninstall(self, verbose=True):
        installed = self.device.is_installed(self.APK_name)
        if installed:
            success = self.device.uninstall(self.APK_name)
            installed = self.device.is_installed(self.APK_name)
            if not installed and success:
                print('APK uninstall finished.')
                print('Please verify if the app disappeared from the list as described in "UNINSTALL.md".')
                print('For the resolution of this issue, please follow https://github.com/Swind/pure-python-adb/issues/71.')
            else:
                print('APK uninstall failed')
        elif verbose:
            print('APK is not installed.')

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
    oculus_reader = OculusReader(ip_address='10.0.0.73')

    while True:
        time.sleep(0.3)
        print(oculus_reader.get_transformations_and_buttons())


if __name__ == '__main__':
    main()
