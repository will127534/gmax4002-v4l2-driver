# Minimum Driver for GMAX4002

GMAX4002 requires external triger to generate the frame, such that it can't really drive itself like most of the MIPI camera sensors, as such the driver here is missing critical V4L2 controls like V4L2_CID_VBLANK, V4L2_CID_HBLANK and V4L2_CID_EXPOSURE and the expect is that the users will generate the pulses externally via other means.  
The way this driver currently setup is with external exposure, which means exposure and framerate is determined by a external pulse frequency and the level high duration, and the driver won't do anything about the controls for V4L2_CID_VBLANK, V4L2_CID_HBLANK and V4L2_CID_EXPOSURE. 
It will still expose them to the upper layer because some applications (rpicam and libcaemra) requires these minimum controls.  
  
The driver requires 4-lane MIPI, as I did not see a way to use 2 or 1 lane from the leaked(?) datasheet here: https://informnapalm.org/ua/wp-content/uploads/sites/9/2024/01/GMAX4002_Pre_Datasheet_V0.3.2_20231109.pdf  
Additionally it only supports 10bit 2.4Mpix (2048x1200) mode, if anyone has a more up to date version of the datasheet, feel free to open an issue and send the datasheet.  


## Working platform
The code is tested with RPI5/Compute module 4 with Analog discovery 2 as pulse generator.  
![_DSC9912](https://github.com/user-attachments/assets/bb5db94e-b476-4b03-acfc-1de090bd2c00)  
See the action in video here: https://youtu.be/cdhxaaCYhbM
the 2048x1200 mode has been tested at least up to 150 fps stable, at 166 fps I starts to see frame drop on the RPI5 side, not exactly quite sure what happened there (Maybe overheat? I didn't have heatsink on the top)
![WIN_20250907_16_41_42_Pro](https://github.com/user-attachments/assets/9dc26838-d668-4146-864c-7a420930f4c0)  

For libcamera support, it is not added to my fork yet so here is a quick run down on how to enable the support in libcamera:

1. copy camera calibration files  
  src\ipa\rpi\pisp\data\uncalibrated.json -> to src\ipa\rpi\pisp\data\gmax4002.json  
  src\ipa\rpi\vc4\data\uncalibrated.json -> to src\ipa\rpi\vc4\data\gmax4002.json  
  Modify the src\ipa\rpi\pisp\meson.build and src\ipa\rpi\vc4\meson.build to add gmax4002.json to the list  
2. Create a blank camera helper file  
  copy src\ipa\rpi\cam_helper\cam_helper_imx585.cpp to cam_helper_gmax4002.cpp  
  Rename everything as gmax4002  
  And then modify src\ipa\rpi\cam_helper\meson.build and add cam_helper_gmax4002.cpp to the list  
Or save the following file as cam_helper_gmax4002.cpp  
```C
#include <assert.h>

#include "cam_helper.h"
#include "math.h"
using namespace RPiController;

class CamHelperGmax4002 : public CamHelper
{
public:
	CamHelperGmax4002();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
        unsigned int hideFramesStartup() const override;
	unsigned int hideFramesModeSwitch() const override;

private:
	static constexpr int frameIntegrationDiff = 4;
};


CamHelperGmax4002::CamHelperGmax4002()
	: CamHelper({}, frameIntegrationDiff)
{
}


uint32_t CamHelperGmax4002::gainCode(double gain) const
{
	int code = 66.6667 * log10(gain);
	return std::max(0, std::min(code, 0xf0));
}

double CamHelperGmax4002::gain(uint32_t gainCode) const
{
	return pow(10, 0.015 * gainCode);
}

unsigned int CamHelperGmax4002::hideFramesStartup() const
{
	/* On startup, we seem to get 1 bad frame. */
	return 1;
}

unsigned int CamHelperGmax4002::hideFramesModeSwitch() const
{
	/* After a mode switch, we seem to get 1 bad frame. */
	return 1;
}

```


4. Re-compile and install libcamera 



## Prerequisites

Before you begin the installation process, please ensure the following prerequisites are met:

- **Kernel version**: You should be running on a Linux kernel version 6.12 or newer. You can verify your kernel version by executing `uname -r` in your terminal.

- **Development tools**: Essential tools such as `gcc`, `dkms`, and `linux-headers` are required for compiling a kernel module. If not already installed, these can be installed using the package manager with the following command:
  
   ```bash 
   sudo apt install linux-headers dkms git
   ```
   
## Installation Steps

### Setting Up the Tools

First, install the necessary tools (`linux-headers`, `dkms`, and `git`) if you haven't done so:

```bash 
sudo apt install linux-headers dkms git
```

### Fetching the Source Code

Clone the repository to your local machine and navigate to the cloned directory:

```bash
git clone https://github.com/will127534/gmax4002-v4l2-driver.git
cd gmax4002-v4l2-driver/
```

### Compiling and Installing the Kernel Driver

To compile and install the kernel driver, execute the provided installation script:

```bash 
./setup.sh
```

### Updating the Boot Configuration

Edit the boot configuration file using the following command:

```bash
sudo nano /boot/config.txt
```

In the opened editor, locate the line containing `camera_auto_detect` and change its value to `0`. Then, add the line `dtoverlay=gmax4002`. So, it will look like this:

```
camera_auto_detect=0
dtoverlay=gmax4002
```

After making these changes, save the file and exit the editor.

Remember to reboot your system for the changes to take effect.

## dtoverlay options

### cam0

If the camera is attached to cam0 port, append the dtoverlay with `,cam0` like this:  
```
camera_auto_detect=0
dtoverlay=gmax4002,cam0
```

### always-on

If you want to keep the camera power always on (Useful for debugging HW issues, specifically this will set CAM_GPIO to high constantly), append the dtoverlay with `,always-on` like this:  
```
camera_auto_detect=0
dtoverlay=gmax4002,always-on
```
