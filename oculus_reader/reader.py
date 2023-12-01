#!/usr/bin/env python
from oculus_reader.FPS_counter import FPSCounter
from oculus_reader.buttons_parser import parse_buttons
import numpy as np
import threading
import time
import os
from ppadb.client import Client as AdbClient
import sys

def eprint(*args, **kwargs):
    RED = "\033[1;31m"  
    sys.stderr.write(RED)
    print(*args, file=sys.stderr, **kwargs)
    RESET = "\033[0;0m"
    sys.stderr.write(RESET)

class OculusReader:
    def __init__(self,
            ip_address=None,
            port = 5555,
            APK_name='com.rail.oculus.teleop',
            print_FPS=False,
            run=True
        ):
        self.running = False
        self.last_hand_transforms = {}
        self.last_left_joints_transforms = {}
        self.last_right_joints_transforms = {}
        self.last_buttons = {}
        self._lock = threading.Lock()
        self.tag = 'wE9ryARX'
        self.joint_tag = 'jsk73b1z_'

        self.ip_address = ip_address
        self.port = port
        self.APK_name = APK_name
        self.print_FPS = print_FPS
        if self.print_FPS:
            self.fps_counter = FPSCounter()

        self.device = self.get_device()
        self.install(verbose=False, reinstall=True)
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

    def get_network_device(self, client, retry=0):
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
                eprint('Make sure that device is running and is available at the IP address specified as the OculusReader argument `ip_address`.')
                eprint('Currently provided IP address:', self.ip_address)
                eprint('Run `adb shell ip route` to verify the IP address.')
                exit(1)
            else:
                self.get_device(client=client, retry=retry+1)
        return device

    def get_usb_device(self, client):
        try:
            devices = client.devices()
        except RuntimeError:
            os.system('adb devices')
            devices = client.devices()
        for device in devices:
            if device.serial.count('.') < 3:
                return device
        eprint('Device not found. Make sure that device is running and is connected over USB')
        eprint('Run `adb devices` to verify that the device is visible.')
        exit(1)

    def get_device(self):
        # Default is "127.0.0.1" and 5037
        client = AdbClient(host="127.0.0.1", port=5037)
        if self.ip_address is not None:
            return self.get_network_device(client)
        else:
            return self.get_usb_device(client)

    def install(self, APK_path=None, verbose=True, reinstall=False):
        try:
            installed = self.device.is_installed(self.APK_name)
            if not installed or reinstall:
                if APK_path is None:
                    APK_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'APK', 'teleop-debug.apk')
                success = self.device.install(APK_path, test=True, reinstall=reinstall)
                installed = self.device.is_installed(self.APK_name)
                if installed and success:
                    print('APK installed successfully.')
                else:
                    eprint('APK install failed.')
            elif verbose:
                print('APK is already installed.')
        except RuntimeError:
            eprint('Device is visible but could not be accessed.')
            eprint('Run `adb devices` to verify that the device is visible and accessible.')
            eprint('If you see "no permissions" next to the device serial, please put on the Oculus Quest and allow the access.')
            exit(1)

    def uninstall(self, verbose=True):
        try:
            installed = self.device.is_installed(self.APK_name)
            if installed:
                success = self.device.uninstall(self.APK_name)
                installed = self.device.is_installed(self.APK_name)
                if not installed and success:
                    print('APK uninstall finished.')
                    print('Please verify if the app disappeared from the list as described in "UNINSTALL.md".')
                    print('For the resolution of this issue, please follow https://github.com/Swind/pure-python-adb/issues/71.')
                else:
                    eprint('APK uninstall failed')
            elif verbose:
                print('APK is not installed.')
        except RuntimeError:
            eprint('Device is visible but could not be accessed.')
            eprint('Run `adb devices` to verify that the device is visible and accessible.')
            eprint('If you see "no permissions" next to the device serial, please put on the Oculus Quest and allow the access.')
            exit(1)

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

    def extract_joint_data(self, line, file_obj):
        output = ''
        i = 0
        while not '!' in line:
            if self.joint_tag in line:
                tmp = line.split(self.joint_tag + str(i) + ': ')
                if len(tmp) > 1:
                    output += tmp[1] + ' '
                    i += 1
                else:
                    return None
            line = file_obj.readline().strip()
        tmp = line.split(self.joint_tag + str(i) + ': ')
        if len(tmp) > 1:
            output += tmp[1]
        else:
            return None
        return output

    def process_joint_data(self, string):
        try:
            hand1_string, hand2_string = string.split(',WristRoot')
        except ValueError:
            return None, None
        hand2_string = 'WristRoot' + hand2_string
        hand2_string = hand2_string.split('!')[0]

        joints1_transforms = self.process_joint_transforms(hand1_string)
        joints2_transforms = self.process_joint_transforms(hand2_string)
        return joints1_transforms, joints2_transforms

    def process_joint_transforms(self, hand_string):
        split_hand_strings = hand_string.split(' ,')
        transforms = {}
        for pair_string in split_hand_strings:
            transform = np.empty((4,4))
            pair = pair_string.split(':')
            if len(pair) != 2:
                continue
            joint_char = pair[0] # joint name
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
                transforms[joint_char] = transform
        return transforms

    def get_transformations_and_buttons(self):
        with self._lock:
            return self.last_hand_transforms, self.last_buttons

    def get_joint_transformations(self):
        with self._lock:
            return self.last_left_joints_transforms, self.last_right_joints_transforms

    def read_logcat_by_line(self, connection):
        file_obj = connection.socket.makefile()
        while self.running:
            try:
                line = file_obj.readline().strip()
                if self.joint_tag in line:
                    data = self.extract_joint_data(line, file_obj)
                    if data:
                        joints1_transforms, joints2_transforms = self.process_joint_data(data)
                        if joints1_transforms is not None and joints2_transforms is not None:
                            with self._lock:
                                if joints1_transforms['Thumb0'][0][3] > 0 :
                                    self.last_left_joints_transforms = joints1_transforms
                                    self.last_right_joints_transforms = joints2_transforms
                                else:
                                    self.last_left_joints_transforms = joints2_transforms
                                    self.last_right_joints_transforms = joints1_transforms
                else:
                    data = self.extract_data(line)
                    if data:
                        transforms, buttons = OculusReader.process_data(data)
                        with self._lock:
                            self.last_hand_transforms, self.last_buttons = transforms, buttons
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
        print(oculus_reader.get_joint_transformations())


if __name__ == '__main__':
    main()
