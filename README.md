# Gelblaster Wingman Robotic Sentry Turret

[![Gelblasters](images/GelBlasterlogo.jpg)](https://gelblaster.com)

If you are interested in building & programming your own "Wingman" turret stop by:<br>
https://join.slack.com/t/robotdogs/shared_invite/zt-24ep8mqn4-1p42Aq7owRv9klLI~3C5Pw and join #gelblasters 

* [Gelblaster Wingman Robotic Sentry Turret](#gelblaster-wingman-robotic-sentry-turret)
   * [Synopsis](#synopsis)
   * [December 2024 - First Outdoor Test](#december-2024---first-outdoor-test)
   * [Introduction](#introduction)
   * [Licensing](#licensing)
   * [Build Steps](#build-steps)
      * [Required parts](#required-parts)
      * [Printing](#printing)
   * [Making a sentry gun](#making-a-sentry-gun)
      * [Required plumbing](#required-plumbing)
      * [Transplant of GelBlaster XL electronics](#transplant-of-gelblaster-xl-electronics)
      * [More 3d printing](#more-3d-printing)
      * [Recommended additional parts](#recommended-additional-parts)
         * [Motion Control](#motion-control)
         * [Computing](#computing)
         * [Power System](#power-system)

## Synopsis 

Ever since we saw James Cameron's "Aliens" in 1986, and Frank Marshall's "Congo" in 1995 we knew we needed to own an automated sentry turret at some point in our lifetime. It wasn't clear we'd eventually be sharing the design to one for others to use, but here we sit, doing exactly that. 

"Wingman" is a ![CC BY-NC-SA ](https://licensebuttons.net/l/by-nc-sa/3.0/88x31.png) licensed Open Source "GelBlaster" turret for ["general-purpose robots"](https://www.bostondynamics.com/open-letter-opposing-weaponization-general-purpose-robots). "Wingman" is designed with gamification of non-lethal adversarial Gelblaster sparring matches in mind. It makes robot assisted marksmanship a key compnent of playtime, are you ready to fight your robot overlords? No!? Well, you better get to training sooner than later!

[![liquor pour](http://img.youtube.com/vi/IS2PtmM9mwU/0.jpg)](https://www.youtube.com/watch?v=IS2PtmM9mwU) 

## December 2024 - First Outdoor Test
[![Latest Status Dec 2024](http://img.youtube.com/vi/UjThLahqY9g/0.jpg)](https://youtube.com/shorts/UjThLahqY9g)<br>

## Current Tracking Response
[![Current Tracking Response](http://img.youtube.com/vi/NoIiMvJtIzc/0.jpg)](https://youtube.com/shorts/NoIiMvJtIzc?si=dIlFipVr6tI9Dh-7)<br>

## Introduction

[Gelblaster](https://gelblaster.com) XL "Wingman" is a robotic turret, that shoots gel balls aka Gellets(tm) at targets as selected by AI logic. The Gelblaster based "Wingman turret SDK" was inspired by U.S. Army [TARDEC](https://asc.army.mil/web/news-alt-jfm18-wingman-is-first-step-toward-weaponized-robotics/) - Tank-automotive and Armaments Command, [Open Robotics](https://www.openrobotics.org) [ROS-M](https://rosmilitary.org/faq/) military fork of Robot Operation System from GVSC - Combat Capabilities Development Command Ground Vehicle Systems Center, and their ["Wingman Robotic Gunner"](https://apps.dtic.mil/sti/pdfs/AD1069401.pdf) platform. Functionality of this project seeks to mimic concepts found in the [RTK](https://vimeo.com/593277076?) - Robotic Technology Kernel program. 

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

### Printing
The Wingman turret shell can be printed using the following pre-configured Bambu Lab build plates:

1. [Wingman GelBlaster Turret Case](https://makerworld.com/en/models/787567#profileId-725512)
2. [Wingman Turret Base](https://makerworld.com/en/models/787554#profileId-725499)

These build plates are optimized for the Bambu Lab A1 printer, but can be adapted for other printers. The complete print will take approximately 50 hours (6 hours for front pieces, 20 hours for each rear piece).

![Printing1](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/Printing1.jpg)

After printing, you'll need to:
1. Remove support structures
2. Clean up the prints
3. Use Gorilla Glue (gel recommended) to join the shell pieces

For those wanting individual STL files:
[CC_BYNCSA_STL_Files directory](https://github.com/MAVProxyUser/Gelblaster_Wingman/)

## Making a sentry gun

### Required plumbing
First remove all the supporting screws in the GelBlaster XL shell, then remove the cosmetic shroud & barrel tip. Once the shell is open, remove the batery. <br>
![RemoveBattery](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/RemoveBattery.jpg)
Remove the screws and prepare to remove the gearbox.<br> 
![RemoveGearbox](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/RemoveGearbox.jpg)
Next remove the shim for the ammo hopper.<br> 
![RemoveShim](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/RemoveShim.jpg)
Then remove the LED array, and associated cables. Do NOT disconnect them from the main PCB.<br> 
![RemoveLEDArray](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/RemoveLEDArray.jpg)
Do the same with the other switches, and wires.<br> 
![UnrouteWires](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/UnrouteWires.jpg)
Remove the screws holding down the PCB, and the barrel in place.<br> 
![RemovePCB](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/RemovePCB.jpg) 
Take the screws out of the barrel.<br>
![RemoveBarrel](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/RemoveBarrel.jpg)
Make sure to get the last few bits unscrewed.<br>
![RemoveFinalBits](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/RemoveFinalBits.jpg)
Eventually you will have extracted the Gearbox in full.<br>
![GearboxClean](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/GearboxClean.jpg)

### Transplant of GelBlaster XL electronics
Transplant all the parts to the shell (you will have some bits left over). Make sure to put the Logo & PCB in place before you start!<br> 
![LEDLogo1](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/LEDLogo1.jpg)
Use all the screws you kept from the teardown. The logo LED's use two short screws, you should have four total.
![LEDLogo2](https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/images/LEDLogo2.jpg)
Next start mounting the gearbox, and barrel. 

Connect the relay to the trigger

### More 3d printing
The turret base works fairly well, but there is always room for improvement. 

### Recommended additional parts

#### Motion Control
- [DYNAMIXEL XM430-W210](https://www.robotis.us/dynamixel-xm430-w210-t/) - Servo motor for turret movement
- [U2D2](https://www.robotis.us/u2d2/) - Dynamixel control board
- [U2D2 Power Hub](https://www.robotis.us/u2d2-power-hub-board-set/) - USB adapter for dynamixel
- Non-latching relay for trigger control

#### Computing
- [NVIDIA Jetson Xavier NX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-xavier-nx/) - Main compute module
- 19V DC to DC Converter - Power regulation
- 5A inline fuse - Circuit protection

#### Power System
- [Li-Time 50AH LiFePo4 Battery](https://www.lithiumion-batteries.com/products/50ah-lifepo4-battery/) - Main power source
- Battery Charger - For LiFePo4 battery
- 4-gang rocker panel - Power distribution


