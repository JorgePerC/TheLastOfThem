# How we built this package

## Jetson's GPIO

First, you need to configure the PinMux on the Jetson, habilitating both of the PWM pins. 

You can do so by initiating 

    sudo /opt/nvidia/jetson-io/jetsion-io.py

I had problems initiating it, but [this]() post helped me resolve the issues. 

The encoder input pins doesn't require extra configuration.

### Motor node

The motor controller for the motor supports a PWM input signal from 1050-1950 freq. This basically vaires from -1, to +1 in relative max speed for the motor. 

### Encoder

For an accurate measurement of the enconder inputs, it's necesary to enable interruptions, the options so far are epoll, poll or select calls through sysfs. 