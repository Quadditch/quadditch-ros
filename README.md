instead of ```source ~/quad_ws/devel/setup.sh```, use ```source ~/quad_ws/src/quadditch/scripts/setup.sh``` 


if there are issues with preflight checks (e.g. accel calibration)
``` 
rm -rf ~/.ros
```


Make sure rosbridge is at release 0.11.10 - the next version is updated for Python3 and breaks the ROSIntegration functionality
