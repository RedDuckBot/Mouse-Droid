#Message expresses Xbox 360 controller buttons, triggers, and joysticks
#Joystick values have a range [-1,1], where (left,up) move towards -1, 
#and (right,down) move towards +1. 
#Both left & right trigger have a range [0,1], where zero is initial position

#Buttons
bool x 
bool y
bool b
bool a
bool left_bumper
bool right_bumper
bool back #Also called select button
bool start
bool mode #Big center button

#Joysticks & triggers
float32 left_joy_x 
float32 left_joy_y
float32 right_joy_x
float32 right_joy_y
float32 left_trigger
float32 right_trigger
