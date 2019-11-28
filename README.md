# Notice: as of May 22, 2019 this plugin is no longer being developed or maintained. For my recent work with brewery control software, please see my open-source brewing dashboard, [brew2](https://github.com/jangevaare/brew2).

## cbpi-CascadeControl
### Introduction
This CraftBeerPi 3.0 plugin provides a several new `KettleController` types:

|Type|Sensors|Actor|Description|
|---|---|---|---|
|`AdvancedPID`|1|`GPIOPWM`|PID control with advanced settings|
|`AdvancedHysteresis`|1|`GPIOSIMPLE`|Hysteresis control with advanced settings|
|`CascadeHysteresis`|2|`GPIOSIMPLE`|PID controlled hysteresis|
|`CascadePID`|2|`GPIOPWM`|A series of 2 PIDs. The *outer loop PID* controls the set point of the *inner loop PID*|

The main purpose of this plugin is for sophisticated mash temperature control within popular RIMS and HERMS-based breweries, however it may have other purposes in your brewery. With the addition of hysteresis functionality to this plugin, it can now be used in settings where PWM was not possible, such as gas-fired HERMS or K-RIMS breweries.

### Motivation
Most brewers with HERMS or RIMS based breweries use trial and error to determine an appropriate differential between their mash temperature and hot liquor tank or RIMS tube temperature in order to maintain the set point. Especially for those who brew in non-temperature controlled environments, this can be inexact and tedious.

With a properly tuned cascade control algorithm, this problem is minimized. Your hot liquor tank or RIMS set point will automatically adjust on the basis of your mash tun set point and mash tun current temperature. Implicit in this tuning is the desired aggressiveness of action, so for those who wish to have *gentle* mash temperature control, they can still do that.

The potential benefit of cascade control goes further. Many brewers with HERMS are told that step mashing is futile in their system - a properly tuned cascade PID unlocks this potential by providing sufficiently aggressive heating to their HLT.

### License
This plugin is open source, and has an MIT License. Please read the included license if you are not already familiar with it.

### Safety and Disclaimers
* This plugin is intended only for those knowledgable and comfortable with control systems.
* Improper tuning could lead to unpredictable results with this plugin. The user must closely monitor their brewery at all times of operation. 
* This plugin should never be used in the absence of proper safety features, especially those related to element dry firing, stuck recirculation, properly rated hardware components, and GFCI protection.

### PID control
A PID control algorithm sets the output as the sum of 3 types of action:

* Proportional action: the larger the difference between the process variable and its set point (known as *error*), the higher the proportional action. Generally this may be used to set the overall aggressiveness of our controller.
* Integral action: the more error we have *accumulated* (magnitude * duration of error), the higher the integral action. Generally this is used to correct for steady-state error.
* Derivative action: the higher the rate of change in error (change of error magnitude/time), the higher the derivative action. Generally this is used to implement a dampening effect on our output to reduce overshoot under aggressive control scenarios.

If the parameters of a PID are tuned reasonably well, there can be precise control of a process variable. Tradeoffs are encountered in the tuning process for aggressiveness, overshooting, and generality.

In addition to the basic PID parameters, this plugin provides options to set:

* Maximum output %
* Initial integrator
* Update interval

### Hysteresis control
Hysteresis is a basic control algorithm where there are two output states, on and off, that are used to keep a process variable near its set point. Typically in systems utilizing hysteresis control it's not possible to incrementally control the output for mechanical reasons, and further, we may wish to minimize or otherwise constrain the switching between output states. For instance, perhaps a mechanical contactor is used, and it is limited physically by it's switching speed and we wish to reduce wear by preventing excessive switching. Or perhaps the thing we are controlling is a compressor in a glycol system, or solenoid controlled gas valve in a direct-fired brewery. All scenarios in which hysteresis would be used.

This plugin provides a hysteresis `KettleController` that can be positive (i.e. heating), or negative (i.e. cooling). It has the following settings which can be used to constrain it's behavior:

* Minimum on-time
* Maximum on-time
* Minimum off-time

As well as update interval.

### Control Loops and Cascade Control
With this plugin, we use two control loops, which we will refer to as the inner loop and the outer loop. With cascade control, two basic things happen:

* The outer loop controls the set point of the inner loop, and,
* The action of the inner loop ultimately controls the process variable in the outer loop.

The inner loop in a traditional HERMS brewery is the control loop for the temperature of the hot liquor tank. In a traditional RIMS brewery, this control loop is for the temperature of the RIMS tube.

This plugin offers PID-Hysteresis and PID-PID cascade control algorithms with `CascadeHysteresis` and `CascadePID` respectively.

In addition to the PID and Hysteresis settings listed above, both `KettleController`s have an option for a max inner loop set point.

### Tuning tips
Tuning of cascade control algorithms is non-trivial, but not impossible. In the future, autotuning methods *may* be implemented, until then, here are some tips:
* The most important method of improving temperature control in your HERMS or RIMS based brewery is by minimizing lag time within the system. This is accomplished by proper mixing and recirculation conditions.
* Read the descriptions above completely for PID control for a basic understanding of what each parameter does.
* Always approach tuning any cascade control algorithm starting with the innermost loop. The `AdvancedPID` and `AdvancedHysteresis` KettleControllers can be used to experiment with the innermost loop only to start.
* For many homebreweries, it is sufficient to set the integral and derivative action parameters to 0 in the inner loop in Cascade PID, as there should be minimal lag in this loop.
* Conditions under tuning should mimic those during brewing, including pump speeds, volume of liquid, presence/absence of temperature stratification.
* Some further information of the PID parameters is provided in their descriptions.
