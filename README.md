# oculus_reader

For the oculus reader two elements are necessary: python script which receives the readings from the APK and the APK itself. Currently the pose of the controllers and pressed buttons are transfered from the APK, but this behavior can be extended using provided APK source code.

If you intend to use the precompiled APK, please follow the steps here and the README from 'python folder'. If you plan to work on the APK development, additionally follow the README from 

## Setup Git LFS for APK

To pull the APK correctly, Git LFS has to be configured. The installation is described here https://git-lfs.github.com. On Ubuntu follow these steps:
```
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
git lfs install # has to be run only once on a single user account
```
If you are using HTTPS protocol, you can reduce the number of authentification prompts when pushing/pulling with:
```
git config lfs.https://github.com/rail-berkeley/oculus_reader.git/info/lfs.locksverify false
```
You should run this command from repository workspace.

## Setup of the ADB

For the communication between  [Android Debug Bridge](https://developer.android.com/studio/command-line/adb) 

1. Determine you Oculus Quest account name:

If you haven’t used Oculus Quest before, start it and follow the steps to create your profile and get yourself started. Otherwise follow these steps to find out your username:
a. Go to: [https://www.oculus.com/](https://www.oculus.com/) 

b. Log in to account:![image_0](https://user-images.githubusercontent.com/14967831/104061442-e32c4280-51f8-11eb-95d9-6f7fe7a55fbe.png)

c. After logging in **select your profile again** in top right corner and select **‘Profile’**

![image_1](https://user-images.githubusercontent.com/14967831/104061496-00f9a780-51f9-11eb-8632-2ad453480bc2.png)

d. You will be able to see your username on the following screen:

![image_2](https://user-images.githubusercontent.com/14967831/104061501-01923e00-51f9-11eb-80a2-53e90efd2f34.png)

2. Enable Oculus Quest development mode:

    a. Inform me ([jedrzej.orbik@berkeley.edu](mailto:jedrzej.orbik@berkeley.edu)) that you need to join the development organization and wait for the confirmation that this step is done.

    b. Turn on the device you want to use for development.

    c. Open the Oculus app on your phone and then go to **Settings**.

    d. Tap the device and then go to **More Settings** > **Developer Mode**.

    e. Turn on the **Developer Mode** toggle.

    f. Connect your device to your computer using a USB-C cable and then wear the device.

    g. Accept **Allow USB Debugging** and **Always allow from this computer** when prompted to on the device.
    
   ![image_3](https://user-images.githubusercontent.com/14967831/104061507-048d2e80-51f9-11eb-8327-7917f6a1ab60.png)

    h. (Windows only) Install the Oculus ADB Drivers

        1. Download[ the zip file containing the driver](https://developer.oculus.com/downloads/package/oculus-adb-drivers/).

        2. Unzip the file.

        3. Right-click on the .inf file and select **Install**.

3. Install ADB. On Ubuntu: `sudo apt install android-tools-adb`. On other systems follow the steps from the 'app' folder.

## How to run

After following README in either in 'python' or 'app' folder, the system is ready for the teleoperation with reader folder.

1. Make sure that Oculus Quest is connected to the same network as the computer.
2. Connect Oculus Quest to PC with USB cable.
3. Verify that a device is visible with: `adb devices`. Expected output:
`List of devices attached

    ce0551e7                device`

4. Check the IP address of the headset:
`adb shell ip route`
Expected output:
`10.0.30.0/19 dev wlan0  proto kernel  scope link  **src **10.0.32.101`

5. The IP address of the device follows `**src` **from the previous step.

6. Save the IP address in the ‘config.yaml’ file. Port no. in the file can remain unchanged.

7. Call `python python/install.py`

8. Start the installed app on Your VR device:

    1. Put on your Oculus Quest

    2. Tab the Apps button in the Main Menu

    3. Tab Unknown Sources
![image_5](https://user-images.githubusercontent.com/14967831/104061509-0525c500-51f9-11eb-98d8-1470dfd1eaa4.jpg)

    4. Start the app ‘RAIL Oculus Teleoperation’

9. Run `python python/reader.py`
