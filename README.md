# Gelblaster Wingman Robotic Sentry Turret

[![Gelblasters](images/GelBlasterlogo.jpg)](https://gelblaster.com)

If you are interested in building & programming your own "Wingman" turret stop by:<br>
https://join.slack.com/t/robotdogs/shared_invite/zt-1fvixx89u-7T79~VxmDYdFSIoTnSagFQ and join #gelblasters 

* [Gelblaster Wingman Robotic Sentry Turret](#gelblaster-wingman-robotic-sentry-turret)
   * [Synopsis](#synopsis)
   * [Introduction](#introduction)
   * [Licensing](#licensing)
   * [Build Steps](#build-steps)
      * [Required parts](#required-parts)
      * [Printing](#printing)
      * [Transplant of GelBlaster XL electronics](#transplant-of-gelblaster-xl-electronics)
   * [Making a sentry gun](#making-a-sentry-gun)
      * [Required plumbing](#required-plumbing)
      * [Recommended additional parts<br>](#recommended-additional-parts)
      * [OpenCV setup for Face tracking on RasPi4](#opencv-setup-for-face-tracking-on-raspi4)

## Synopsis 

Ever since we saw James Cameron's "Aliens" in 1986, and Frank Marshall's "Congo" in 1995 we knew we needed to own an automated sentry turret at some point in our lifetime. It wasn't clear we'd eventually be sharing the design to one for others to use, but here we sit, doing exactly that. 

"Wingman" is a ![CC BY-NC-SA ](https://licensebuttons.net/l/by-nc-sa/3.0/88x31.png) licensed Open Source "GelBlaster" turret for ["general-purpose robots"](https://www.bostondynamics.com/open-letter-opposing-weaponization-general-purpose-robots). "Wingman" is designed with gamification of non-lethal adversarial Gelblaster sparring matches in mind. It makes robot assisted marksmanship a key compnent of playtime, are you ready to fight your robot overlords? No!? Well, you better get to training sooner than later!

[![liquor pour](http://img.youtube.com/vi/IS2PtmM9mwU/0.jpg)](https://www.youtube.com/watch?v=IS2PtmM9mwU) [![Congo](http://img.youtube.com/vi/Ss35wHcN6iQ/0.jpg)](https://www.youtube.com/watch?v=Ss35wHcN6iQ)<br>

## Introduction

[Gelblaster](https://gelblaster.com) XL "Wingman" is a robotic turret, that shoots Super Absorbant Polymer balls at targets as selected by AI logic. The Gelblaster based "Wingman turret SDK" was inspired by U.S. Army [TARDEC](https://asc.army.mil/web/news-alt-jfm18-wingman-is-first-step-toward-weaponized-robotics/) - Tank-automotive and Armaments Command, [Open Robotics](https://www.openrobotics.org) [ROS-M](https://rosmilitary.org/faq/) military fork of Robot Operation System from GVSC - Combat Capabilities Development Command Ground Vehicle Systems Center, and their ["Wingman Robotic Gunner"](https://apps.dtic.mil/sti/pdfs/AD1069401.pdf) platform. Functionality of this project seeks to mimic concepts found in the [RTK](https://vimeo.com/593277076?) - Robotic Technology Kernel program. 

A gamified version of the "Wingman" concept known as [BloodHounds](https://github.com/MAVProxyUser/BloodHounds) is being designed by Kevin Finisterre for use with Quadruped robot dogs. This project is being tested, & developed with assistence from John Cherbini & the Gelblaster design team, as a means to visualize the "game". 

The design concept was pitched to the GelBlaster team, and the initial ideation phase reulted in the following proof of concept work. Thanks again to [GlytchTech](https://twitter.com/GlytchTech) for the very early prototypes, pre Gelblaster pitch.<br> 
![Concept1](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/Concept1.png)
![Concept2](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/Concept2.png)

The barrel seemed a bit vulnerable so we asked for a shroud.<br> 
![Concept3](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/Concept3.png)
![Concept4](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/Concept4.png)

We also needed a mounting system, pic rail seemed reasonable.<br> 
![Concept5](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/Concept5.png)
![Concept6](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/Concept6.png)

Considerations over printability came next... and now we have something we can share. Mind you we still don't use the shroud, and we're testing sawed off barel designs. In otherwords, don't mind our mess while we are under construction.

## Licensing 

Creative Commons + Attribution-NonCommercial-ShareAlike licensing ( 
CC BY-NC-SA ) applies to the 3D files shared along side this text. This license lets others remix, adapt, and build upon your work non-commercially, as long as they credit you and license their new creations under the identical terms.

https://creativecommons.org/licenses/by-nc-sa/4.0/<br>
https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode<br>

Under the following terms:

Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.

NonCommercial — You may not use the material for commercial purposes.

ShareAlike — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original.

No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.

## Build Steps

### Required parts

[GelBlaster Surge XL](https://gelblaster.com/products/surge-xl)
or
[GelBlaster Starfire XL](https://gelblaster.com/products/starfire-xl). Either blaster can be purchased from [Walmart](https://www.walmart.com/ip/Gel-Blaster-Surge-XL-Day-N-Nite-Gel-Bead-Blaster-with-Glow-in-the-Dark-Starfire-Activator-5k-Starfire-Gellets-10k-Green-Gellets/1283028596), and [Target](https://www.target.com/p/gel-blaster-starfire-xl-glow-in-the-dark-gellet-blaster/-/A-86669382) respectively. 

![SurgeXL](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/SurgeXL.jpeg)
![StarFire XL](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/StarFire.jpeg)

Both STL & 3mf files for our "Wingman" robotic Gelblaster turret are provided below Our personal test prints were sliced using [Cura](https://ultimaker.com/software/ultimaker-cura). You may slice with what ever makes you comfortable. 

[C_BYNCSA_R_Back.3mf](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_R_Back.3mf)<br>
[C_BYNCSA_R_Back.stl](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_R_Back.stl)<br>
[CC_BYNCSA_L_Back.3mf](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_L_Back.3mf)<br>
[CC_BYNCSA_L_Back.stl](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_L_Back.stl)<br>
[CC_BYNCSA_R_Front.3mf](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_R_Front.3mf)<br>
[CC_BYNCSA_R_Front.stl](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_R_Front.stl)<br>
[CC_BYNCSA_R_Front.3mf](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_R_Front.3mf)<br>
[CC_BYNCSA_R_Front.stl](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_R_Front.stl)<br>

### Printing
This writeup will result in complete Wingman turret shells, ready to be filled with electronics, and assembled. Two example shells will be depicted during printing, cleanup, and assembly phases. These shells were in this case both printed with an [Ender S1 Pro 3d printer](https://www.creality3dofficial.com/products/ender-3s1-pro-3d-printer). 

When you begin printing prepare for 50 hours or so of print time. 6 hours each on the front, and 20 hours each on the rear slices. 
![Printing1](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/Printing1.jpg)

Once printing is complete you will need to cleanup the prints by removing support structure. 
![SupportWaitingForRemoval1](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/SupportWaitingForRemoval1.jpeg)

Removal is fairly striaght forward.
![SupportRemoval1](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/SupportRemoval1.jpeg)
![SupportRemoval2](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/SupportRemoval2.jpeg)
![SupportRemoval3](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/SupportRemoval3.jpeg)

There will be quite a bit of waste from the support structures
![SupportWaste1](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/SupportWaste1.jpeg)
![SupportWaste2](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/SupportWaste2.jpeg)
![Trash1](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/Trash1.jpeg)

Once the prints are cleaned up we need to Gorilla Glue the parts together. 
![Ready1](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/Ready1.jpeg)
![Ready2](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/Ready2.jpeg)
Gel based glue will remove dripping from the equation. 
![SuperGlue1](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/SuperGlue1.jpeg)
Add beads of superglue generously, and press the shell parts together. 
![SuperGlue1](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/SuperGlue2.jpeg)
![SuperGlue1](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/SuperGlue3.jpeg)

These shells are now ready to recieve the electronics to make them function! 

## Making a sentry gun

### Required plumbing
First remove all the supporting screws in the GelBlaster XL shell. 


### Transplant of GelBlaster XL electronics
Transplant all the parts to the shell (you will have some bits left over). Connect the relay to the trigger

### More 3d printing
For now the turrent mount is completely cobbled together. We use the base from a Dynamixel Planar Manipulator randomly found on Github:<br>
https://github.com/adilzhaniwe/planar-manipulator-CAD/base_new.sldprt has been converted to .stl for you:<br>
[base_new.stl](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_base_new.stl)<br>

We also use a mashup of these two files, since the files are listed as CC BY-NC-ND licensed, we can not share the change)
[Female Picatinny rail mount](https://www.thingiverse.com/thing:3821230)<br>
[Robot Bracket](https://www.thingiverse.com/thing:98266) 

We'll correct this shortly, by relesaing better solution. 

### Recommended additional parts<br>
Co-computing:<br>
[RasPi4](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)<br>
[RasPi4 Hiqh Quality Camera](https://www.raspberrypi.com/products/raspberry-pi-high-quality-camera/)<br>
(Jetson upgrade forthcoming)

Storage:<br>
[SanDisk 512GB Ultra microSD UHS-I speed](https://www.westerndigital.com/products/outlet/memory-cards/sandisk-ultra-uhs-i-microsd#SDSQUAR-512G-AN6MA) - Minimum recommended speed<br>
[SanDisk 512GB Extreme microSDXC UHS-I](https://www.westerndigital.com/products/memory-cards/sandisk-extreme-uhs-i-microsd#SDSQXAV-512G-GN6MA) - Suggested speed<br>
[SanDisk 512GB Extreme pro microSDXC UHS-I](https://www.westerndigital.com/products/memory-cards/sandisk-extreme-pro-uhs-i-microsd#SDSQXCD-512G-GN6MA) - Optimal speed<br>

Stabilization & Mounting:<br>
[Joby Compact Advanced](https://joby.com/ca-en/compact-advanced-tripod-for-smartphone-and-camera-jb01763-bww/) tripod. <br>

Power:<br>
[Volessence 50000 battery](https://www.amazon.com/Volessence-50000mAh-Laptop-Portable-Charger/dp/B07RNZZXRM)

### OpenCV setup for Face tracking on RasPi4
Current design relies on the following items:<br>
[MX-28AT](https://www.robotis.us/dynamixel-mx-28at/)<br>
[U2D2 Power Hub Board Set](https://www.robotis.us/u2d2-power-hub-board-set/)<br>
[U2D2](https://www.robotis.us/u2d2-power-hub-board-set/)<br>

Proof of concept code:<br>
https://github.com/MAVProxyUser/BloodHounds/blob/main/turret_v1_finisterre/v1_dynamixel.py

Earlier designs relied on stepper motors and a smaller GelBlaster. An upgraded stepper turret is still under 
construction.<br>
https://github.com/MAVProxyUser/BloodHounds/blob/main/turret_v1_cherbini/v1.py

