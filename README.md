# Raspberry pi for controller Chen's hand

This packages is running on the Ubuntu 22.04 with Humble version.

`./hand_server.py` is for desktop PC with ROS2 interface. 

It receives command by service interface.
All of them are used std_srvs::SetBool type

- `/left/motor` for left hand bigger motor
- `/left/pump` for left hand's pump
- `/left/smotor` for left hand smaller motor
- `/right/motor` for right hand bigger motor
- `/right/pump` for right hand's pump
- `/right/smotor` for right hand smaller motor
- `/dual/motor` for controlling both of the hand's bigger motor at the same time.
- `/dual/pump` for controlling both of the pump
- `/dual/smotro` for controlling both of the smaller motor