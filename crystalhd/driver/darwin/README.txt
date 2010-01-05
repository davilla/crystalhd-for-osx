OSX kext driver for Broadcom Crystal HD video decoder (BMC70012/BMC70010).

Notes:
Driver is built i386 against 10.4 SDK. This makes the driver loadable under 10..5 and 10.4 OSX systems as well as the AppleTV OS.

Tested under 10.5.5 and AppleTV OS 1.0 so should be compatible with all.

Broadcom firmware is expected in /usr/lib/bcmFilePlayFw.bin
Broadcom library is expected in /usr/lib/libcrystalhd.dyib

To load the kext:
sudo cp -R build/Development/BroadcomCrystalHD.kext /tmp
sudo kextload -v /tmp/BroadcomCrystalHD.kext

To unload the kext:
sudo kextunload /tmp/BroadcomCrystalHD.kext

To enable kext loading under the AppleTV you must first run Turbo's AppleTV Enabler (turbo_atv_enabler). Get it here - > http://0xfeedbeef.com/appletv/turbo_atv_enabler.bin. 
"sudo ./turbo_atv_enabler.bin"

Issues:
1) BSD ioctl process session tracking need a re-write. Not working properly.
2) Multiple Broadcom devices will likely barf the driver.

TODO:
1) Implement suspend/resume.







