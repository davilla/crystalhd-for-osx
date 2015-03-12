# Broadcom Crystal HD, It's Magic #
![http://crystalhd-for-osx.googlecode.com/svn/branches/web_items/crystalhd-pr-shot.png](http://crystalhd-for-osx.googlecode.com/svn/branches/web_items/crystalhd-pr-shot.png)

OSX driver and library support for the Broadcom Crystal HD Video Accelerator. Supported under XBMC for Mac on the AppleTV and under 10.4 and 10.5 OSX platforms. You can read the XBMC PR release at http://xbmc.org/davilla/2009/12/29/broadcom-crystal-hd-its-magic/ . The source code can be found in this svn tree, binaries are in the download section.

## Build/Install Instructions (library) ##
```
cd darwin_lib/libcrystalhd/
make
sudo make install
```

This will install "/usr/lib/libcrystalhd.dylib",  "/usr/lib/bcm70012fw.bin" and "/usr/lib/bcm70015fw.bin" as well as library headers located at "/usr/includes/libcrystalhd/".

## Build Instructions (kext driver) ##
```
cd driver/darwin/
xcodebuild -project BroadcomCrystalHD.xcodeproj -target BroadcomCrystalHD -configuration Deployment build
```

## Install Instructions (kext driver) ##
You must have correct permissions on the kext. The easy way to to copy the kext to /tmp and load it from there. This install is not permanent and will vanish on reboot. A permanent install will be documented later.
```
sudo cp -r driver/darwin/build/Deployment/BroadcomCrystalHD.kext /tmp
sudo kextload /tmp/BroadcomCrystalHD.kext
```

## Special Instructions for the AppleTV ##
The AppleTV OS will not allow loadoing of kexts until you run Turbo's kext enabler (http://0xfeedbeef.com/appletv/)  . Install the kext and setup loading in "/etc/rc.local". You also need the crystalhd library and firmware installed.
```
sudo cp -r BroadcomCrystalHD.kext /System/Library/Extensions/
sudo -S chmod -R 755 /System/Library/Extensions/BroadcomCrystalHD.kext
sudo -S chown -R root:wheel /System/Library/Extensions/BroadcomCrystalHD.kext
```

Your "/etc/rc.local" should have something like this:
```
# rc.local
/sbin/turbo_kext_enabler.bin

/sbin/kextload -v /System/Library/Extensions/BroadcomCrystalHD.kext
```

If "/etc/rc.local" does not exist, make it
```
sudo touch /etc/rc.local
sudo chmod 755 /etc/rc.local
sudo chown root:wheel /etc/rc.local
```

Copy the library and firmware  into place, this assumes you have already copied these items to the AppleTV.
```
sudo cp libcrystalhd.dylib /usr/lib/
sudo chmod 755 /usr/lib/libcrystalhd.dylib

sudo cp bcm70012fw.bin /usr/lib/
sudo cp bcm70015fw.bin /usr/lib/
```

## Notes ##
This svn tree contains both OSX and Linux source code for the Broadcom Crystal HD. It has patches that fix various things and is based off the official Broadcom source code release. There is also a public upstream git tree at http://git.wilsonet.com/crystalhd.git/ where patches/fixes found there will be feed back to Broadcom. For the most up to date OSX implementation, use the svn tree here. For the most up to date Linux implementation, use the upstream git tree.

## Where to Buy ##
There are two flavors of bmc70012 mini pci-e cards, old layout and new layout. Both are functionally identical and the driver, library and firmware will work with both flavors.  There is also the new bcm70015 Crystal HD cards and these are also supported with this driver/lib/firmware provided you build from svn trunk. A release version has not been made the includes bmc70015 support yet. New layout bcm70012 and bcm70015 cards can be found at [www.logicsupply.com ](http://www.logicsupply.com/products/bcm970015). Old layout bcm70012 cards can be found on ebay.

## Donations ##
Several users have asked for a way to donate to this project so here's my [paypal](https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=2YUW9FF9NK5UL&lc=US&currency_code=USD&bn=PP%2dDonationsBF%3abtn_donateCC_LG%2egif%3aNonHosted) link. There's no obligation of course but it does help convince my wife that spending insane amounts of time in the mancave is a good thing :)


