# if open(/dev/ttyUSB0) failed: No such file or directory,change to ttyUSB1
# if can't find camera, use ls /dev/ to find changed videoX
#before run    sudo -s
sleep 3
gcc -shared -fPIC -o canusb.so canusb.c

