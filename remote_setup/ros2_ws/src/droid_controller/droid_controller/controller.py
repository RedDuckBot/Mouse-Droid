import rclpy 
from rclpy.node import Node 
from droid_interfaces.action import Drive
from droid_interfaces.msg import XboxController
from xbox360controller import Xbox360Controller
from rclpy.action.client import ClientGoalHandle
from rclpy.action import ActionClient
from enum import Enum

class ButtonCategories(Enum):
    SOUND = 0
    DISCO = 1
    ROTATE_180 = 2
    ROTATE_360 = 3

BUTTON_SOUNDS = ["button_x","button_y","button_a","button_b"] 

class XboxControllerNode(Node):
    def __init__(self):
        super().__init__("controller")
        self.controller_client_1 = ActionClient(self, Drive, "droid_buttons")
        self.controller_client_2 = ActionClient(self, Drive, "droid_motors")

        #Initialize and setup xbox controller
        self.Xcontroller_ = Xbox360Controller(0,axis_threshold=0.0)
        self.Xcontroller_.button_x.when_pressed = self.on_button_press 
        self.Xcontroller_.button_y.when_pressed = self.on_button_press 
        self.Xcontroller_.button_b.when_pressed = self.on_button_press 
        self.Xcontroller_.button_start.when_pressed = self.on_button_press
        self.Xcontroller_.button_select.when_pressed = self.on_button_press
        self.Xcontroller_.button_mode.when_pressed = self.on_button_press
        self.Xcontroller_.button_a.when_pressed = self.on_button_press 
        self.Xcontroller_.axis_r.when_moved = self.on_joy_move 
        self.Xcontroller_.axis_l.when_moved = self.on_joy_move 

        self.current_right_joy = 0.0
        self.current_left_joy = 0.0
        self.get_logger().info("Xbox controller node initialized.")

    def on_joy_move(self, axis): 
        controller = XboxController()
        goal = Drive.Goal()
        goal.is_sound_command = False
        goal.is_disco_command = False
        goal.is_rotate_command = False
        goal.is_joy_command = True

        if not (axis.y > -0.25 and axis.y < 0.25):
            if axis.name == "axis_r":
                controller.right_joy_y = axis.y
                self.current_right_joy = axis.y
                controller.left_joy_y = self.current_left_joy
            else:
                controller.left_joy_y = axis.y
                self.current_left_joy = axis.y
                controller.right_joy_y = self.current_right_joy
        else:
            if axis.name == "axis_r":
                controller.right_joy_y = 0.0
                self.current_right_joy = 0.0 
                controller.left_joy_y = self.current_left_joy
            else:
                controller.left_joy_y = 0.0
                self.current_left_joy = 0.0
                controller.right_joy_y = self.current_right_joy

        goal.controller = controller
        self.send_command(command=goal,command_type=2)

    def on_button_press(self, button):
        controller = XboxController() #message xbox controller
        goal = Drive.Goal()
        goal.is_joy_command = False

        if button.name in BUTTON_SOUNDS:
            button_type = ButtonCategories.SOUND

            if button.name == BUTTON_SOUNDS[0]: 
                controller.x = True
            elif button.name == BUTTON_SOUNDS[1]: 
                controller.y = True
            elif button.name == BUTTON_SOUNDS[2]:
                controller.b = True
            else: 
                controller.a = True
        elif button.name == "button_start":
            button_type = ButtonCategories.DISCO
            controller.start = True
        elif button.name == "button_select":
            button_type = ButtonCategories.ROTATE_180
            controller.back = True
        elif button.name == "button_mode":
            button_type = ButtonCategories.ROTATE_360
            controller.mode = True

        if button_type == ButtonCategories.SOUND:
            goal.is_sound_command = True
            goal.is_disco_command = False
            goal.is_rotate_command = False
        elif button_type == ButtonCategories.DISCO:
            goal.is_sound_command = False
            goal.is_rotate_command = False
            goal.is_disco_command = True
        elif (button_type == ButtonCategories.ROTATE_180) or  \
            (button_type == ButtonCategories.ROTATE_360):
            goal.is_sound_command = False
            goal.is_rotate_command = True
            goal.is_disco_command = False

        goal.controller = controller
        self.send_command(command=goal,command_type=1)

    #Purpose: send controller commands from Drive action
    #Parameters: command is a message containing controller input
    #            command_type is an int value eithr 1 or 2, i.e. client_1 or _2
    #Post: sent command (goal) to 'droid' server node
    def send_command(self, command, command_type):
        #Wait for sever 

        #Send goal (command) to one of action servers
        if command_type == 1:
            self.controller_client_1.wait_for_server()
            self.controller_client_1.send_goal_async(command). \
                add_done_callback(self.goal_response_callback)
        elif command_type == 2:
            self.controller_client_1.wait_for_server()
            self.controller_client_2.send_goal_async(command). \
                add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted: 
            self.goal_handle_.get_result_async().add_done_callback(
                self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        #self.get_logger().info("Result: " + result.reached_command)

def main():
    rclpy.init()
    controller_node = XboxControllerNode()
    rclpy.spin(controller_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
