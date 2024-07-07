#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "droid_interfaces/action/drive.hpp"
#include "droid_interfaces/msg/xbox_controller.hpp"
#include "sound_module/SoundPlayer.hpp"
#include "libgpio/MotorDriver.hpp"
#include "libgpio/DigitalOutput.hpp"
#include <chrono>
#include <cstdlib>
#include <unistd.h>
#include <pigpio.h>
#include "imu_module/RPi_Sensor.h"
#include "imu_module/RPi_BNO055.h"
#include "imu_module/utility/imumaths.h"
#include "controller_PID/PID.hpp"

using Drive = droid_interfaces::action::Drive;
using Xcontroller = droid_interfaces::msg::XboxController;
using DroidGoalHandle = rclcpp_action::ServerGoalHandle<Drive>;
using namespace std::placeholders;
using DY::Player;
using Controllers::PID;
using namespace libgpio;


class DroidServerNode: public rclcpp:: Node
{
    public:
        DroidServerNode(char * sound_port, 
        unsigned int in1_pin, unsigned int in2_pin, unsigned int enA_pin,
        unsigned int in3_pin, unsigned int in4_pin, unsigned int enB_pin,
        uint32_t gpio_disco_num): 
            Node("Droid_Server"), player_(sound_port), 
            leftMotors_(in3_pin,in4_pin,enB_pin),
            rightMotors_(in1_pin,in2_pin,enA_pin),
            discoPin_(gpio_disco_num)
        {
            int soundModuleReady;

            //Setup IMU sensor
            bno_ = Adafruit_BNO055();
            bno_._HandleBNO=i2cOpen(bno_._i2cChannel,BNO055_ADDRESS_A,0);
            if(!bno_.begin())
            {
                RCLCPP_INFO(this->get_logger(),
                    "No BNO055 detected ... Check your wiring or I2C ADDR!");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "IMU Module Ready");
                bno_.setExtCrystalUse(true);
            }

            //Set disco pin
            discoPin_.setOutput(false);

            //Setup threading groups
            cb_group_1_ = this->create_callback_group(rclcpp::
                CallbackGroupType::Reentrant); //Joy-stick group
            cb_group_2_ = this->create_callback_group(rclcpp::
                CallbackGroupType::MutuallyExclusive); //Button group

            //Setup action servers
            //_1, _2 <--- refers to # of arguments used in goal_callback
            droid_server_buttons_ = rclcpp_action::create_server<Drive>(
                this,
                "droid_buttons",
                std::bind(&DroidServerNode::goal_callback_buttons,this, _1, _2), 
                std::bind(&DroidServerNode::cancel_callback_buttons, this, _1),
                std::bind(&DroidServerNode::handle_accepted_callback_buttons, 
                    this, _1),
                rcl_action_server_get_default_options(),
                cb_group_1_
            );
            droid_server_joy_ = rclcpp_action::create_server<Drive>(
                this,
                "droid_motors",
                std::bind(&DroidServerNode::goal_callback_joys,this, _1, _2), 
                std::bind(&DroidServerNode::cancel_callback_joys, this, _1),
                std::bind(&DroidServerNode::handle_accepted_callback_joys, 
                    this, _1),
                rcl_action_server_get_default_options(),
                cb_group_2_
            );

            //Setup motors
            int frequency = 800; // 800 Hz
            gpioSetPWMfrequency(enA_pin, frequency);
            gpioSetPWMfrequency(enB_pin, frequency);
            leftMotors_.setDirection(MotorDirection::FORWARD);
            leftMotors_.setEffort_percent(0.0);
            rightMotors_.setDirection(MotorDirection::FORWARD);
            rightMotors_.setEffort_percent(0.0);

            //Setup sound board
            soundModuleReady = player_.begin();
            player_.setVolume(30);
            player_.setCycleMode(DY::PlayMode::OneOff);
            if (soundModuleReady == 0) 
            {
                RCLCPP_INFO(this->get_logger(), "Sound Module Ready");
                UART_filestream_number = player_.UART_filestream;
            }
            else 
            {
                RCLCPP_INFO(this->get_logger(), "Sound Module not Ready");
            }

            RCLCPP_INFO(this->get_logger(), "Droid servers started");
        }

        ~DroidServerNode()
        {
            if (UART_filestream_number != -1)
            {
                close(UART_filestream_number);
            }

            RCLCPP_INFO(this->get_logger(), "Cleaned up resources"); 
        }

    private:
        int UART_filestream_number = -1; //Used to close UART connection to sound module
        Player player_; //Sound player
        Adafruit_BNO055 bno_; //IMU
        MotorDriver leftMotors_, rightMotors_;
        DigitalOutput discoPin_;

        bool rotate_button_active = false;
        //PID controller for 180 degree rotation
        PID pid_180_ = PID(100.0 / 180.0, 0.0, 0.0); 

        rclcpp_action::Server<Drive>::SharedPtr droid_server_buttons_;
        rclcpp_action::Server<Drive>::SharedPtr droid_server_joy_;
        rclcpp::CallbackGroup::SharedPtr cb_group_1_;
        rclcpp::CallbackGroup::SharedPtr cb_group_2_;

        std::shared_ptr<DroidGoalHandle> goal_handle_joy_;
        std::shared_ptr<DroidGoalHandle> goal_handle_button_;
        std::mutex mutex_joy_;
        std::mutex mutex_button_;

        rclcpp_action::GoalUUID prempt_rotate_goal_id_;

        rclcpp_action::GoalResponse goal_callback_joys(
            const rclcpp_action::GoalUUID &uuid, 
            std::shared_ptr<const Drive::Goal> goal)
        {
            {
                //Policy: refuse new joy stick goal if one is active
                //Or if buttons for rotating is active
                std::lock_guard<std::mutex> lock(mutex_joy_);
                if (goal_handle_joy_)
                {
                    if (goal_handle_joy_ -> is_active())
                    {
                        RCLCPP_INFO(this->get_logger(), 
        "[Joy-Server]: A joy-stick command is active; reject new joy goal.");
                        return rclcpp_action::GoalResponse::REJECT;
                    }

                    if (rotate_button_active)
                    {
                        RCLCPP_INFO(this->get_logger(),
            "[Joy-Server]: joy goal rejected since rotate button is active.");
                        return rclcpp_action::GoalResponse::REJECT;
                    }
                }
            }

            (void) uuid;
            (void) goal;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::GoalResponse goal_callback_buttons(
            const rclcpp_action::GoalUUID &uuid, 
            std::shared_ptr<const Drive::Goal> goal)
        {
            (void) uuid;
            (void) goal;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse cancel_callback_buttons(
            const std::shared_ptr<DroidGoalHandle> goal_handle)
        {
            (void) goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        
        rclcpp_action::CancelResponse cancel_callback_joys(
            const std::shared_ptr<DroidGoalHandle> goal_handle)
        {
            (void) goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted_callback_joys(
            const std::shared_ptr<DroidGoalHandle> goal_handle)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_joy_);
                this-> goal_handle_joy_ = goal_handle;
            }

            auto goal = goal_handle->get_goal();

            if (goal->is_joy_command) 
            {
                execute_joy_goal(goal_handle);
            }
            send_goal_result(goal_handle);
        }
        
        void handle_accepted_callback_buttons(
            const std::shared_ptr<DroidGoalHandle> goal_handle)
        {
            auto goal = goal_handle->get_goal();

            if (goal->is_sound_command) 
            {
                execute_sound_goal(goal_handle);
            }

            if (goal->is_disco_command)
            {
                execute_disco_goal(goal_handle);
            }

            if (goal->is_rotate_command) 
            {
                if (rotate_button_active)
                {

                }
                else 
                {
                    {
                        std::lock_guard<std::mutex> lock(mutex_button_);
                        goal_handle_button_ = goal_handle;
                    }
                    rotate_button_active = true;
                    execute_rotate_goal(goal_handle);
                    rotate_button_active = false;

                }
            }
        }

        void execute_rotate_goal(const std::shared_ptr<DroidGoalHandle> 
            goal_handle)
        {
            auto rotate_goal = goal_handle -> get_goal();
            Xcontroller controller = rotate_goal -> controller;

            int heading_goal;
            imu::Vector<3> euler = 
                bno_.getVector(Adafruit_BNO055::VECTOR_EULER);

           if (controller.back == true) //button for 180 degree rotation
           {
                heading_goal = ((int) (euler.x() + 180.0)) % 360;
                pid_180_.set_heading_goal(heading_goal);
                handle_rotation(heading_goal, pid_180_);
           }
           else //Handle center button press (rotate 360 degrees)
        {
                heading_goal = ((int) (euler.x() + 180.0)) % 360;
                pid_180_.set_heading_goal(heading_goal);
                handle_rotation(heading_goal, pid_180_);

                sleep(0.5);
                imu::Vector<3> euler = 
                    bno_.getVector(Adafruit_BNO055::VECTOR_EULER);

                heading_goal = ((int) (euler.x() + 180.0)) % 360;
                pid_180_.set_heading_goal(heading_goal);
                handle_rotation(heading_goal, pid_180_);
           }

            send_goal_result(goal_handle);

        }

        void handle_rotation(double heading_goal, PID pid)
        {
            double tolerated_error = 5.0; //Degrees
            double current_heading;
            double motors_adjustment; 
            imu::Vector<3> euler;  

            leftMotors_.setEffort_percent(0.0);
            rightMotors_.setEffort_percent(0.0);

            sleep(0.5);

            std::cout << "\nHeading Goal: " << heading_goal << std::endl;

            //Only move clockwise
            leftMotors_.setDirection(MotorDirection::FORWARD);
            rightMotors_.setDirection(MotorDirection::BACKWARD);

            while (true)
            {
                euler = bno_.getVector(Adafruit_BNO055::VECTOR_EULER);
                current_heading = euler.x(); 

                motors_adjustment = pid.compute_adjustment(current_heading);
                current_heading = euler.x(); 

                if (abs(pid.get_error()) < tolerated_error )
                {
                    break;
                }

                //Constrain motor adjustment within [70,100.0]
                if (motors_adjustment > 100.0)
                {
                    motors_adjustment = 100.0;
                } else if (motors_adjustment < 70.0)
                {
                    motors_adjustment = 70.0;
                } else
                {
                    motors_adjustment = motors_adjustment;
                }

                leftMotors_.setEffort_percent(motors_adjustment);
                rightMotors_.setEffort_percent(motors_adjustment);

                sleep(0.5);
            }
            
            std::cout << "Final Heading: " << current_heading << std::endl;
            leftMotors_.setEffort_percent(0.0);
            rightMotors_.setEffort_percent(0.0);

        }

        void execute_disco_goal(const std::shared_ptr<DroidGoalHandle>
            goal_handle)
        {
            if (discoPin_.getState() == true)
            {
                discoPin_.setOutput(false);
            } 
            else
            {
                discoPin_.setOutput(true);
            }
            send_goal_result(goal_handle);
        }

        void execute_sound_goal(const std::shared_ptr<DroidGoalHandle> 
            goal_handle)
        {
            auto sound_goal = goal_handle -> get_goal();

            handle_sound_command(sound_goal->controller);
            send_goal_result(goal_handle);

        }

        void send_goal_result(const std::shared_ptr<DroidGoalHandle> goal_handle)
        {
            auto result = std::make_shared<Drive::Result>();
            result->reached_command = "Success";
            goal_handle->succeed(result);
        }

        void handle_sound_command(Xcontroller controller)
        {
            
           if (controller.x) 
           {
                player_.playSpecifiedDevicePath(DY::Device::Sd, "/chat2.wav");
           }
           else if (controller.y)
           {
                player_.playSpecifiedDevicePath(DY::Device::Sd, "/chirp2.wav");
           } 
           else if (controller.b)
           {
                player_.playSpecifiedDevicePath(DY::Device::Sd, "/chirp1.wav");
           } 
           else //Handle button press a
           {
                player_.playSpecifiedDevicePath(DY::Device::Sd, "/gen3.wav");
           }
        }

        void execute_joy_goal(const std::shared_ptr<DroidGoalHandle> 
            goal_handle)
        {
            auto joy_goal = goal_handle -> get_goal();
            Xcontroller controller = joy_goal -> controller;
            
            MotorDirection left_joy_dir;
            MotorDirection right_joy_dir;
            double new_left_motors_effort, new_right_motors_effort;
            
            //Set left joy stick direction
            if (controller.left_joy_y < 0)
            {
                left_joy_dir = MotorDirection(MotorDirection::FORWARD);
            } 
            else 
            {
                left_joy_dir = MotorDirection(MotorDirection::BACKWARD);
            }

            //Set right joy stick direction
            if (controller.right_joy_y < 0)
            {
                right_joy_dir = MotorDirection(MotorDirection::FORWARD);
            }
            else
            {
                right_joy_dir = MotorDirection(MotorDirection::BACKWARD);
            }

            //Check for change in motor directions
            if (leftMotors_.getDirection() != left_joy_dir)
            {
                leftMotors_.setDirection(left_joy_dir);
            }
            if (rightMotors_.getDirection() != right_joy_dir)
            {
                rightMotors_.setDirection(right_joy_dir);
            }

            new_left_motors_effort = controllerInput_to_motorEffort(
                abs(controller.left_joy_y));
            new_right_motors_effort = controllerInput_to_motorEffort(
                abs(controller.right_joy_y));

            leftMotors_.setEffort_percent(new_left_motors_effort);
            rightMotors_.setEffort_percent(new_right_motors_effort);

        }

        double controllerInput_to_motorEffort(double joy_stick_input) 
        {
            return joy_stick_input * 100;
        }
};

int main(int argc, char ** argv)
{
     //Right motors is controlled using to GPIO23 (pin 16), GPIO24 (pin 18), 
     //and GPIO25 (pin 22)
    static unsigned int IN1_PIN = 23; 
    static unsigned int IN2_PIN = 24; 
    static unsigned int ENA_PIN = 25;

    //Left motors is controlled using to GPIO17 (pin 11), GPIO27 (pin 13), and 
    //GPIO22 (pin 15)
    static unsigned int IN3_PIN = 17; 
    static unsigned int IN4_PIN = 27; 
    static unsigned int ENB_PIN = 22;

    //Disco pin GPIO5
    uint32_t disco_pin = 6;

    char sound_port[] = "/dev/serial0";
    rclcpp::init(argc,argv);
    auto droid_server_node = std::make_shared<DroidServerNode>(
        sound_port,
        IN1_PIN, IN2_PIN, ENA_PIN,
        IN3_PIN, IN4_PIN, ENB_PIN,
        disco_pin);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(droid_server_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
