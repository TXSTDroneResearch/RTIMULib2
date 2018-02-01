#!/bin/sh

echo "This script is for modern debian systems, and does everything that should"
echo "be necessary to use ftdi usb devices as a regular user with libftdi,"
echo "instead of the built-in dumb kernel driver."
echo
if [ $(id -u) != 0 ]; then
  echo "This script must be run as root."
  exit 1
else
  read -p "Press enter to continue, or ctrl-c to bail..." x
fi
echo
echo "** Blacklisting FTDI driver with FT232H".
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", RUN="/bin/sh -c '\''echo -n $kernel:1.0 > /sys/bus/usb/drivers/ftdi_sio/unbind'\''"' >> /etc/udev/rules.d/98-ftdi_sio.rules
echo
echo "** Retriggering udev"
/etc/init.d/udev trigger
echo
echo "!! Done. Replug your device!"