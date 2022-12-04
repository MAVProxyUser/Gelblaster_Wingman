# Gelblasters Wingman

If you are interested in building & programming your own wingman stop by: https://join.slack.com/t/robotdogs/shared_invite/zt-1fvixx89u-7T79~VxmDYdFSIoTnSagFQ and join #gelblasters 

* [Gelblasters Wingman](#gelblasters-wingman)
   * [Introduction](#introduction)
   * [Synopsis](#synopsis)
   * [Licensing](#licensing)
   * [Build Steps](#build-steps)
      * [Required parts](#required-parts)
      * [Recommended additional parts<br>](#recommended-additional-parts)
   * [OpenCV setup for Face tracking on RasPi4](#opencv-setup-for-face-tracking-on-raspi4)

## Introduction

[Gelblaster](https://gelblaster.com) XL "Wingman" is robotic turret, that shoots Super Absorbant Polymer balls at targets as selected by AI logic. The "Gelblaster turret SDK" was inspired by U.S. Army [TARDEC](https://asc.army.mil/web/news-alt-jfm18-wingman-is-first-step-toward-weaponized-robotics/) - Tank-automotive and Armaments Command, [Open Robotics](https://www.openrobotics.org) [ROS-M](https://rosmilitary.org/faq/) military fork of Robot Operation System from GVSC - Combat Capabilities Development Command Ground Vehicle Systems Center. Functionality seeks to mimic concepts found in the [RTK](https://vimeo.com/593277076?) - Robotic Technology Kernel program.

The Wingman (game) concept was designed by Kevin Finisterre, tested, & developed by John Cherbini with assistance from Gelblaster the design team. 

## Synopsis 

Ever since we saw James Cameron's "Aliens" in 1986, and Frank Marshall's "Congo" in 1995 we knew we needed to own an automated sentry turret at some point in our lifetime. It wasn't clear we'd eventually be sharing the design to one for others to use, but here we sit, doing exactly that. 

[![liquor pour](http://img.youtube.com/vi/IS2PtmM9mwU/0.jpg)](https://www.youtube.com/watch?v=IS2PtmM9mwU) [![Congo](http://img.youtube.com/vi/Ss35wHcN6iQ/0.jpg)](https://www.youtube.com/watch?v=Ss35wHcN6iQ)<br>

"Wingman" is a ![CC BY-NC-SA ](https://licensebuttons.net/l/by-nc-sa/3.0/88x31.png) licensed Open Source "GelBlaster" turret for ["general-purpose robots"](https://www.bostondynamics.com/open-letter-opposing-weaponization-general-purpose-robots). Built with gamification of non-lethal adversarial Gelblaster sparring matches in mind, "wingman" makes marksmanship a key comonent of playtime.

## Licensing 

Creative Commons + Attribution-NonCommercial-ShareAlike licensing ( 
CC BY-NC-SA ) applies to the 3D files shared along side this text. 

This license lets others remix, adapt, and build upon your work non-commercially, as long as they credit you and license their new creations under the identical terms.

https://creativecommons.org/licenses/by-nc-sa/4.0/
https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

Under the following terms:

Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.

NonCommercial — You may not use the material for commercial purposes.

ShareAlike — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original.

No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.

## Build Steps

### Required parts

[GelBlaster Surge XL](https://gelblaster.com/products/surge-xl)
or
[GelBlaster Starfire XL](https://gelblaster.com/products/starfire-xl). They can be purchased from [Walmart](https://www.walmart.com/ip/Gel-Blaster-Surge-XL-Day-N-Nite-Gel-Bead-Blaster-with-Glow-in-the-Dark-Starfire-Activator-5k-Starfire-Gellets-10k-Green-Gellets/1283028596), and [Target](https://www.target.com/p/gel-blaster-starfire-xl-glow-in-the-dark-gellet-blaster/-/A-86669382) respectively. 

![SurgeXL](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/SurgeXL.jpeg)
![StarFire XL](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/StarFire.jpeg)

Both STL & 3mf files for our "wingman" robotic Gelblaster turret are provided below Our personal test prints were sliced using [Cura](https://ultimaker.com/software/ultimaker-cura). You may slice with what ever makes you comfortable. 

[C_BYNCSA_R_Back.3mf](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_R_Back.3mf)<br>
[C_BYNCSA_R_Back.stl](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_R_Back.stl)<br>
[CC_BYNCSA_L_Back.3mf](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_L_Back.3mf)<br>
[CC_BYNCSA_L_Back.stl](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_L_Back.stl)<br>
[CC_BYNCSA_R_Front.3mf](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_R_Front.3mf)<br>
[CC_BYNCSA_R_Front.stl](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_R_Front.stl)<br>
[CC_BYNCSA_R_Front.3mf](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_R_Front.3mf)<br>
[CC_BYNCSA_R_Front.stl](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_R_Front.stl)<br>

### Recommended additional parts<br>
Co-computing:
[RasPi4](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)<br>
[RasPi4 Hiqh Quality Camera](https://www.raspberrypi.com/products/raspberry-pi-high-quality-camera/)<br>
(Jetson upgrade forthcoming)

Storage:
[SanDisk 512GB Ultra microSD UHS-I speed](https://www.westerndigital.com/products/outlet/memory-cards/sandisk-ultra-uhs-i-microsd#SDSQUAR-512G-AN6MA) - Minimum recommended speed<br>
[SanDisk 512GB Extreme microSDXC UHS-I](https://www.westerndigital.com/products/memory-cards/sandisk-extreme-uhs-i-microsd#SDSQXAV-512G-GN6MA) - Suggested speed<br>
[SanDisk 512GB Extreme pro microSDXC UHS-I](https://www.westerndigital.com/products/memory-cards/sandisk-extreme-pro-uhs-i-microsd#SDSQXCD-512G-GN6MA) - Optimal speed<br>

Stabilization & Mounting:
[Joby Compact Advanced](https://joby.com/ca-en/compact-advanced-tripod-for-smartphone-and-camera-jb01763-bww/) tripod. <br>

## OpenCV setup for Face tracking on RasPi4

https://github.com/MAVProxyUser/BloodHounds/blob/main/turret_v1_finisterre/v1_dynamixel.py

