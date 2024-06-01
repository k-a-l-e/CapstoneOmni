
# Phantom Omni - Capstone

This project is a proof of concept for using a Phantom Omni haptic device in educational settings. The following will detail the initial setup.




## Author
[Kale Papalkar](https://www.github.com/k-a-l-e)


## Acknowledgements
The following instructions based on the [Install instructions](https://github.com/jhu-cisst-external/phantom-omni-1394-drivers?tab=readme-ov-file) by Anton Deguet.

The demo code in this repository is based on fsuarez8's example [here](https://github.com/fsuarez6/phantom_omni).



## Instructions

The install script below will clone and install [this repository](https://github.com/jhu-cisst-external/phantom-omni-1394-drivers?tab=readme-ov-file). Then use the script as described below. The script will:

- In directory ```files``` uncompress all archives ```(.tgz and .zip)```
- Using ```dpkg```, install the two ```.deb``` files provided by the vendor (driver and SDK)
- Remove all ```libPHANToMIO.so*``` files and replace them with those provided in the JUJU archives along with new symbolic links
- Configure your system to find the shared libraries in ```/usr/lib64```
- Secure fireWire port and/or dongle permissions 

Install script (functions as above):
```
    mkdir -p ~/phantom-omni-1394-drivers     # get the files
    cd ~/phantom-omni-1394-drivers
    git clone https://github.com/jhu-cisst-external/phantom-omni-1394-drivers
    cd ~/phantom-omni-1394-drivers/phantom-omni-1394-drivers
    sudo ./install-omni-drivers.sh # finally run the install script
    sudo rm -rf Linux_JUJU_PDD_64-bit OpenHapticsAE_Linux_v3_0  # this is just to clean temporary files
```

To uninstall:
```
   cd ~/phantom-omni-1394-drivers/phantom-omni-1394-drivers
   sudo ./uninstall-omni-drivers.sh
```

### Dependencies 

The drivers come with two executables: ```PHANToMConfiguration``` and ```PHANToMTest```. The test program is compiled against ```libraw1394.so.8``` which is not available on many Ubuntu distributions. I could not get it working on Ubuntu 20.04. The configuration executable works but you might need to install ```libGLs```. It can be installed using ```sudo apt install libglw1-mesa``` on Ubuntu 18.04, 20.04 and 22.04.

### Using PHANToMConfiguration

We need sudo privilages to run the configuration executable with: ```sudo PHANToMConfiguration```. Click "Add", enter a name for the device, select the arm type (Omni) and then click "Apply" and "OK" to quit.

