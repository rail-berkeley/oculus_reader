from ppadb.client import Client as AdbClient
import yaml
import os

current_dir = os.path.dirname(os.path.realpath(__file__))
config_path = current_dir + '/config.yml'
if not os.path.exists(config_path):
    print("Please copy config_example.yml to config.yml and adjust the specified IP address.")
    print("To this end, please follow the 'How to run the code' section of README.md in the project's root folder")
    exit(1)
cfg = yaml.load(open(config_path, 'r'), Loader=yaml.CLoader)

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
            print('Make sure that device is running and is available at the IP address specified in config.yml.')
            print('Run `adb shell ip route` to verify the IP address.')
            exit(1)
        else:
            get_device(retry=retry+1)
    return device

def install(device, verbose=True):
    installed = device.is_installed(cfg['APK_NAME'])
    if not installed:
        device.install(cfg['APK_PATH'], test=True)
        installed = device.is_installed(cfg['APK_NAME'])
        if installed:
            print('APK installed successfully.')
        else:
            print('APK install failed.')
    elif verbose:
            print('APK is already installed.')

def uninstall(device, verbose=True):
    installed = device.is_installed(cfg['APK_NAME'])
    if installed:
        device.uninstall(cfg['APK_NAME'])
        installed = device.is_installed(cfg['APK_NAME'])
        if installed:
            print('APK uninstall failed')
        else:
            print('APK uninstalled successfully.')
    elif verbose:
        print('APK is not installed.')

def reinstall(device):
    uninstall(device, verbose=False)
    install(device)

def main():
    import argparse

    parser = argparse.ArgumentParser(description='Utility to manage teleoperation APK. Installs APK if no arguments are provided.')
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
