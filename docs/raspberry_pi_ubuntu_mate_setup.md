# Raspberry Pi Ubuntu Mate Setup
These instructions will set up a Raspberry Pi 3 with:  

- Ubuntu Mate 18.04 (haven’t tried with 16.04 but things should be almost entirely the same)
- VUNet connection
- Static IP address
- SSH
- ROS Melodic (use Melodic with 18.04, Kinetic with 16.04)
- Mavlink ability
- Dronekit Python API
- Working Camera
- USB Flash drive storage capability
- Custom script that runs every time the pi boots up
- Intel Realsense T265 sensor
- X11 Forwarding

*Please amend these notes if anything changes or if you figure out easier ways to do anything!*

### First Steps

Download an Ubuntu Mate image and flash to the SD card.

On the first boot up you’ll need a monitor and there will be some first time setup windows you have to go through.

Hold off on connecting to the internet until you’re fully logged in since you’ll need to open a browser to connect even to VUGuest.

Use ```sudo raspi-config``` and set up this stuff:
- Boot to command line instead of GUI (unless you plan to use the GUI like a noob)
- Enable ssh
- Enable camera
- Disable login through serial, keep interface enabled (for mavlink) (Interfacing Options -> Serial -> "no" -> "yes")
- Expand filesystem

### VuNet, Static IP Configuration

This is pretty easy to do using the GUI so this is the recommended approach for a Pi.

*Connecting to VuNet:*

First connect to VUGuest, start browser, login, then go to wifi.vanderbilt.edu - follow the link for connecting laptops and mobile devices. It should detect you’re using Linux, download the file it gives you and run it using sh fileyoudownloaded.run - if this doesn’t work you may need to add executable permission by doing ```chmod +x fileyoudownloaded.run```

Now it’ll prompt you for vunet id and password, both for VuNet and eduroam, you just enter the same things for both. Now you should be connected to VuNet.

(note: VuNet will have priority by default, but if you want to you can change the connection priorities by clicking the wifi button -> Edit Connections, click the network, go to its settings, go to ‘general’ tab, and modifying the priority. Higher number = higher priority)

*Configuring static IP:*

Find current ip and netmask with ```ifconfig``` - you probably want to change the ip address to something easy to remember but it doesn’t really matter.

Find default gateway with ```route -n``` 

Find DNS servers with ```nmcli device show wlan0 | grep IP4.DNS```

Unless things have changed, ip should be 10.66.??.??, netmask should be 255.255.0.0, gateway should be 10.66.0.1

Then follow the instructions from this video to configure the static ip:
https://www.youtube.com/watch?v=6qsyPTDU2aY

(note: you’ll have to repeat this process with eth0 if you want to have a static ip for ethernet connection)

If you can’t access the GUI for some reason or are using a headless version (such as Ubuntu Server), the harder part will probably be connecting to VUNet - I’m not sure how to do this so good luck! Once you’re connected, you’ll need all the same network info so those commands are all the same, then there are one or two files you’ll have to edit to set up the static ip - there should be good instructions out there somewhere for that part

### Update / Upgrade

Do this now that internet is set up:

```	
sudo apt-get update   (run this or some packages may not be found later)
sudo apt-get upgrade
```

Enable SSH - should be straightforward with raspi-config but has never worked right for me when enabling that way - I’ve always had to uninstall and reinstall openssh-server for it to work

```
sudo apt-get purge openssh-server
sudo apt-get install openssh-server
```

(ubuntu mate download page says openssh-server isn’t installed by default so maybe the problem is that I’m trying to enable before it’s really installed - so maybe try installing right away, before you enable it with raspi-config)

### Set up MAVLink, mavproxy, pymavlink

Mostly follow this [tutorial](http://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html).

```
sudo apt-get install screen python-wxgtk3.0 python-matplotlib python-opencv python-pip python-numpy python-dev libxml2-dev libxslt-dev python-lxml
sudo pip install future
sudo pip install pymavlink
sudo pip install mavproxy
```

RPI 3B and later have built-in bluetooth which uses the hardware serial port on the pi - we want to use this for serial communication with the flight controller so we need to disable the Bluetooth to regain control of it (there’s actually another serial port we could use but it’s weaker so unless you need to use bluetooth you should do this).

Add to the end of /boot/config.txt: 

```dtoverlay=pi3-disable-bt	
enable_uart=1
```

Modify cmdline.txt to be:

```
dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles
```
Now if you run the mavproxy script it should connect and load the parameters from the vehicle.

*Possible issues:*
- Rx/Tx pins are switched - remember rx of one connects to tx of the other
- Baud rate mismatch between the two devices - make sure the baud rate param for serial2 in the pixhawk matches the number you’re using on the pi
- If it’s still not working triple check that there are no typos in any of the modified lines
- If it works with at least one pixhawk and not others, it’s either a baud-rate mismatch or some other parameter is set wrong on the other pixhawks

### Install Dronekit

```
sudo pip install dronekit
```

### Camera Setup

‘Should’ just work once you’ve enabled the camera from raspi-config. 

Test by running:
```
raspistill -o test.jpg
```

If it doesn’t work, be super sure that the cable is connected properly, make sure ```start_x=1``` and ```gpu_mem=128``` in /boot/config.txt (I think raspi-config should already have done this), then try updating firmware (```sudo rpi-update``` - you may have to ```sudo apt install rpi-update``` first), then idk, begin your journey into the forums.

### USB Flash Drive Setup

Useful if you want to move large files to or from pi frequently.

Do these the first time you’re setting up a USB drive with this pi:

```
sudo mkdir /media/usb  (create directory to mount drive to)

sudo chown -R pi:pi /media/usb (set your user as owner - change ‘pi’ to your username)
```

Do these for every drive you want to connect:

```
ls -l /dev/disk/by-uuid/

(write down the UUID for your drive - 8 chars with dash in the middle)

sudo nano /etc/fstab 

(add the following line to this file - ensures drive mounts on startup - change ‘pi’ to your username here too and change UUID to what you recorded earlier)


UUID=18A9-9943 /media/usb vfat auto,nofail,noatime,users,rw,uid=pi,gid=pi 0 0
```

Now you should be able to save things to ```/media/usb/``` and they’ll be saved to the flash drive. If you unplug while it’s powered on, you’ll have to manually remount the drive.
```
sudo mount -a    (remounts all drives)
```
To unmount, use: 
```
sudo umount /dev/sda1    (note ‘umount’ not ‘unmount’)
```
Here you may want to try capturing an image and video to the usb drive to verify that both of the previous two sections are working.

### ROS Setup

Follow these [instructions](http://wiki.ros.org/melodic/Installation/Ubuntu):

I recommend the bare bones version since you probably won’t be using the GUI on the pi much - plus it’s easy to install packages later as you need them.

Use Kinetic instead of Melodic if you’re using Ubuntu 16.04 (they probably both work with both Ubuntu versions but it isn’t guaranteed so unless you have a good reason not to just use the recommended pairings)

### Startup Script

There are a number of ways to do have a script run on startup (rc.local, upstart task, services) - cron seems to be the simplest way I’ve come across
```
crontab -e  
```

Lets you add cron jobs. Add...
```
@reboot /home/pi/myscript  
```
… To the file to run myscript at startup. You can add as many of these jobs as you want and they can be configured to run at regular intervals, specific times, etc.

I think the easiest way to use this for startup tasks is to create a bash file called ```startup.sh``` which starts up all the stuff you want, then add an ```@reboot``` cron job for that script. That way you only ever need to mess with that one bash script.

If you need root permission for anything, run: 
```
sudo crontab -e 
```

Anything you add to that will run with root access.

### Realsense T265 Setup

Since the Pi has an ARM architecture, the default [Realsense instructions](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md) don’t work, and the library must be compiled from source. Installation instructions for the Pi can be found [here](https://ardupilot.org/dev/docs/ros-vio-tracking-camera.html#ros-vio-tracking-camera), but there were some issues that you may run into:
 
- If ```sudo dphys-swapfile swapoff``` fails, try installing dphys with ```sudo apt install dphys-swapfile```.
- When running ```sudo apt install raspberrypi-kernel-headers```, the pi was unable to locate this package. This proved not to be a problem, so move along. If it somehow becomes a problem later, run:
```
sudo apt install linux-headers-generic 
sudo apt install linux-headers-$(uname -r)
```
- The Pi is unable to locate gcc-6 and g++-6: remove the “-6” from the two lines:

```  
export CC=/usr/bin/gcc-6
export CXX=/usr/bin/g++-6
```

- When building, it was unable to find X11 package and RandR headers. To fix, run:
```
sudo apt install libx11-dev xauth xorg-dev libglu1-mesa-dev
```

The ```make -j4``` command will take ~7-8 hours to make. Once you have finished all the instructions and rebooted, check installation was successful with:
```
rs-pose
```

To be able to use the pyrealsense Python wrapper, add this line to the bottom of ```~/.bashrc```:
```
export PYTHONPATH=$PYTHONPATH:/usr/local/lib
```
Then either reboot or run ```source ~/.bashrc``` to reload the file. Test that it works with:

```
cd /librealsense/wrappers/python/examples
python3 t265_example.py
```

*Note:* The ```realsense-viewer``` GUI does not by default work on the Pi. It is possible to make it work with X11 forwarding, but it is not worth it. Just debug plugging the Realsense into your laptop then go to the Pi. 

### X11 Forwarding

X11 forwarding is used to display a GUI on your laptop (the client) of a program running on the Raspberry Pi (the server). This can be very helpful for debugging programs on the Pi in headless mode, and can be used with functions like ```cv2.imshow()```. Only a few instructions are needed to set up:

*Raspberry Pi:*

Install xauth with: 
```
sudo apt install xauth
```

Enable X11 Forwarding over SSH:

```
sudo nano /etc/ssh/ssh_config
```
Remove the # comment for X11Forwarding and change from no to yes  
Remove the # comment for X11ForwardingTrusted and leave as yes

*Laptop:*

When you ssh, add the correct flag for X11 forwarding:

```
ssh -X username@ip
```

You can also make X11 forwarding enabled at startup with the same modifications to /etc/ssh/ssh_config that were made on the Pi. After this, no -X is needed.

To set up X11 Forwarding on Windows, follow the instructions [here](https://aruljohn.com/info/x11forwarding/).





	

	

