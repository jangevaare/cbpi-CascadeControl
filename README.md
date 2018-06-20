# cbpi-CascadePID
## Introduction
This CraftBeerPi 3.0 plugin provides a new KettleController type called CascadePID. The main purpose of this plugin is for sophisticated mash temperature control within popular RIMS and HERMS-based breweries, however it may have other purposes in your brewery.

A SinglePID KettleController is also provided in this plugin.

Both KettleControllers provided are to be used with a GPIOPWM actor for your heating element.

## Typical Usage
Most brewers with HERMS or RIMS based breweries use trial and error to determine an appropriate differential between their mash temperature and hot liquor tank or RIMS tube temperature in order to maintain the set point. Especially for those who brew in non-temperature controlled environments, this can be inexact and tedious.

With a properly tuned cascade PID, this problem disappears. Your hot liquor tank or RIMS set point will automatically adjust on the basis of your mash tun set point and mash tun current temperature. Implicit in this tuning is the desired aggressiveness of action, so for those who wish to have *gentle* mash temperature control, they can still do that.

The potential benefit of cascade PID goes further. Many brewers with HERMS are told that step mashing is futile in their system. A properly tuned cascade PID unlocks this potential.

## License
This plugin is open source, and has an MIT License. Please read the included license if you are not already familiar with it.

## Safety and Disclaimers
* The most important method of improving temperature control in your HERMS or RIMS based brewery is by reducing lag time within the system. This is accomplished by proper mixing and recirculation conditions.
* This plugin is intended only for those knowledgable and comfortable with control systems.
* Improper tuning could lead to unpredictable results with this plugin. The user must closely monitor their brewery at all times of operation, especially during the tuning process. Conditions under tuning should mimic those during brewing, including pump speeds, volume of liquid, presence/absence of temperature stratification.
* This plugin should never be used in the absence of proper safety features, especially those related to element dry firing, properly rated hardware components, and GFCI protection.

## Control Loops and Cascade Control
With this plugin, we use only two control loops, which we will refer to as the inner loop and the outer loop.

With Cascade PID, two things happen:
* The outer loop controls the set point of the inner loop, and,
* The action of the inner loop ultimately controls the process variable in the outer loop.

The inner loop in a traditional HERMS brewery is the control loop for the temperature of the hot liquor tank. In a traditional RIMS brewery, this control loop is for the temperature of the RIMS tube.

The outer loop is the actual process variable you are interested in controlling. In the case of both RIMS and HERMS breweries, this is the mash temperature. The process variable in the outer loop is dependent upon the inner loop.

## Tuning
Tuning of these systems is non-trivial, but not impossible. You should approach tuning a cascade control starting with the innermost loop. The provided SinglePID KettleController can be used for these purposes, with the properties from a well tuned SinglePID KettleController copied over as the inner loop properties in a CascadePID KettleController.

For many homebreweries, it is sufficient to set the integral and derivative action parameters to 0 in the inner loop, as there should be minimal lag in this loop.

Some further information of the PID parameters is provided in their descriptions.
