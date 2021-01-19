from ppadb.client import Client as AdbClient
import yaml
import os

current_dir = os.path.dirname(os.path.realpath(__file__))
cfg = yaml.load(open(current_dir + '/config.yml', 'r'), Loader=yaml.CLoader)

def get_device(retry=0):
    # Default is "127.0.0.1" and 5037
    client = AdbClient(host="127.0.0.1", port=5037)
    try:
        client.remote_connect(cfg['IP_ADDRESS'], cfg['PORT'])
    except RuntimeError:
        os.system('adb devices')
        client.remote_connect(cfg['IP_ADDRESS'], cfg['PORT'])

    device = client.device(cfg['IP_ADDRESS'] + ':' + str(cfg['PORT']))

    if device is None:
        if retry==1:
            os.system('adb tcpip ' + str(cfg['PORT']))
        if retry==2:
            print('Make sure that device is running and is available at specified IP address.')
            print('Run `adb shell ip route` to verify.')
            exit(1)
        else:
            get_device(retry=retry+1)
    return device

def install(device):
     # Check apk is installed
    installed = device.is_installed(cfg['APK_NAME'])
    if not installed:
        device.install(cfg['APK_PATH'], test=True)
    print('APK is installed.')

def uninstall(device):
     # Check apk is installed
    installed = device.is_installed(cfg['APK_NAME'])
    if installed:
        device.uninstall(cfg['APK_NAME'])
    print('APK is uninstalled.')

def reinstall(device):
    uninstall(device)
    install(device)

def main():
    import argparse

    parser = argparse.ArgumentParser(description='Utility to manage teleoperation APK. Installs APK by default.')
    parser.add_argument("--reinstall", action="store_true", help='reinstalls APK from the default path')
    parser.add_argument("--uninstall", action="store_true", help='uninstalls APK')
    args = parser.parse_args()

    device = get_device()

    if args.reinstall:
        reinstall(device)
    elif args.uninstall:
        uninstall(device)
    else:
        install(device)
    print('Done.')

if __name__ == "__main__":
    main()
