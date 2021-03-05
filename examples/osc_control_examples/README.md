
# OSC control examples

OSC - opensoundcontrol.org is a simple message exchange protocol. Widely used in the Audio world it is being hailed as the successor to MIDI. But MIDI itself, and now OSC, has always been about sending and receiving fairly simple control messages, at medium speed, and that makes it well suited for the same kind of task in robotics or other control applications.

As a protocol it is simple, making implementation quite easy, and while binary, simple enough to be fairly "human readable", which aids in debugging and reduces errors.

The main advantage of using it is that there is a bunch of ready made software, and also libraries to use in your own programs, neaning you don't have to re-invent these things from scratch.

## TouchOSC

The first example shows how to set up control of a motor from TouchOSC. It's a super-fast way to set up a customizable GUI for your motor-project, that runs on your phone...

## purr-Data

The second example, "simplefoc\_osc\_esp32\_fullcontrol.ino" allows setting the variables and tuning the motor parameters, in the same way as the serial control but via OSC. The file "osc\_fullcontrol.pd" contains a sample GUI to go with that sketch, allowing the control and tuning of 2 BLDC motors.

Here a screenshot of what it looks like in purr-Data:

![Screenshot from pD](osc_fullcontrol_screenshot.png?raw=true "pD controlling 2 BLDC motors")


## Links to software used in examples

- OSC - opensoundcontrol.org
- pD - https://puredata.info
- TouchOSC - https://hexler.net/products/touchosc

