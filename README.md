# cbpi-CascadePID
## Introduction
This CraftBeerPi 3.0 plugin provides a new KettleController type called CascadePID. The main purpose of this plugin is for sophisticated mash temperature control within popular RIMS and HERMS-based breweries, however it may have additional purposes.

Use of this plugin is technical, and it is not recommended for beginners.

## License
This plugin is open source, and has an MIT License. Please read the included license if you are not already familiar with it.

## Safety and Disclaimers
* The most effective way of improving temperature control in your RIMS or HERMS brewery is not by using this plugin, but rather by reducing lag time within the system. This is accomplished by proper mixing and recirculation conditions.
* This plugin is intended only for those knowledgable and comfortable with control systems.
* Improper tuning could lead to unpredictable results with this plugin. The user must closely monitor their brewery at all times of operation, especially during the tuning process.
* This plugin should never be used in the absence of proper safety features, especially those related to element dry firing, properly rated hardware components, and GFCI protection.

## Control Loops and Cascade Control
With this plugin, we use only two control loops, which we will refer to as the inner loop and the outer loop.

With Cascade PID, two things happen:
* The outer loop controls the set point of the inner loop, and,
* The action of the inner loop ultimately controls the process variable in the outer loop.

The inner loop in a traditional HERMS brewery is the control loop for the temperature of the hot liquor tank. In a traditional RIMS brewery, this control loop is for the temperature of the RIMS tube.

The outer loop is the actual process variable you are interested in controlling. In the case of both RIMS and HERMS breweries, this is the mash temperature. The process variable in the outer loop is dependent upon the inner loop.

## Tuning
Tuning of these systems is non-trivial, but not impossible. You should approach tuning a cascade control starting with the innermost loop.

For many homebreweries, it is entirely sufficient to set the integral and derivative action parameters to 0 in the inner loop, as there should be minimal lag in this loop. As such, 0 is the default values of these parameters.

Beyond this suggestion, tuning is responsibility of the user, and requires some knowledge of control systems.
