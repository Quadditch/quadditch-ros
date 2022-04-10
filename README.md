instead of ```source ~/quad_ws/devel/setup.sh```, use ```source ~/quad_ws/src/quadditch/scripts/setup.sh``` 


if there are issues with preflight checks (e.g. accel calibration)
``` 
rm -rf ~/.ros
```

Make sure rosbridge is at release 0.11.10 - the next version is updated for Python3 and breaks the ROSIntegration functionality


### Testing Tips

Have Pi disconnected from power supply and pixhawk at startup. I think there is an issue with transient current - when everything tries to start up at once the Pi can't get enough power. It doesn't happen all the time but turning it on second works consistently. After a few seconds (I wait for the pixhawk startup sequence to finish) power on the Pi. Give it several seconds to start booting then plug in the USB for the pixhawk connection. 

## Tasks
- [ ] Emergency landing button
- [ ] Standardize and freeze computer setup
  - [ ] use offline account instead of Wade's
  - [ ] make sure all Manycam are up-to-date with same settings
  - [ ] Git
- [ ] Use something like Ansible to standardize systems and control
  - [ ] for Pis and Laptops
  - [ ] Allow us to git pull and rebuild on all machines simultaneously
- [ ] Fix launch procedure
  - [ ]  Use ROSLaunch instead of shell script
  - [ ]  get MAV_SYS_ID from drone before starting ROS (use this to set UAV ID)
  - [ ]  make hostnames independent of UAV ID (Harry Potter names?)

