# **Circutree Usage Documentation**

# **LED Documentation**

## **Disabled - RSL Solid**

## Blue Swiping up
Starting up

## Blink blue followed by Solid Blue
Startup complete

## Rainbow
Idle

## Solid Red - After Holding User Button
Pivot motor on coast mode

## Solid Green - After Releasing User Button
Pivot motor returned to coast mode

## Blink orange - After Release User Button
Compressor enabled on startup setting toggled

#
## **Enabled - RSL Blinking**

## Solid Yellow
Operator wants cone

## Solid Purple
Operator wants cube

## Blinking Red - While Docking
Balancing but not engaged

## Fading Green - While Docking
Docked and engaged

## Blinking Red - While not Docking
Robot is tipping. Faster tipping means robot is at a greater angle

## Solid Red
Robot is sinificantly of ully tipped

## Blinking Swipe Down Fading Red
Final ten seconds of teleop

## Other
Idle

# **User Button Operation**

## Toggling compressor enable on startup setting
Quickly pressing and releasing the user button on the robrio will toggle the enable compressor on startup setting
### Expected Behavior
LED blink orange on release

## Setting Pivot Motor to Coast Mode
Pressing and holding the user button on the roborio will set the pivot motor to coast 
### Expected Behavior
LED solid red when pivot motor is set to coast mode

## Restoring Pivot Motor to Break Mode
Releasing the user button when the pivot motor is in coast mode will set it to break mode
### Expected Behavior
LED solid green, temporarly, when motor is set to break mode