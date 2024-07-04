// // // Includes
// // #include <rcl/rcl.h>
// // #include <rclc/rclc.h>
// // #include <rclc/executor.h>
// // #include <geometry_msgs/msg/twist.h>
// // #include <std_msgs/msg/float32.h>
// // #include <pico/stdlib.h>
// // #include <hardware/pwm.h>
// // #include <rmw_microros/rmw_microros.h>
// // #include "pico_uart_transports.h"
// // #include <math.h>

// // // Constants
// // #define PWM_RANGE 255
// // #define STATIC_PWM_VALUE 50

// // #define PWM_PIN_FR 16
// // #define DIR_PIN_FR 17

// // #define PWM_PIN_FL 18
// // #define DIR_PIN_FL 19

// // #define PWM_PIN_BL 20
// // #define DIR_PIN_BL 21

// // #define PWM_PIN_BR 2
// // #define DIR_PIN_BL 3

// // #define LED_PIN 25

// // #define UPDATE_INTERVAL_MS 10  // 10ms update interval

// // // Constants
// // const uint ENCODER_PIN_A = 4;
// // const uint ENCODER_PIN_B = 5;
// // const int TICKS_PER_REVOLUTION = 60; // Ticks per full rotation of the encoder
// // const float WHEEL_RADIUS_CM = 2.8; // Wheel radius in cm (56 mm)


// // // New constants for the movement sequence
// // const float FORWARD_DISTANCE_CM = 100.0;  // Distance to move forward
// // const float TURN_ANGLE_DEGREES = 90.0;    // Angle to turn (90 degrees for right turn)
// // const float FINAL_DISTANCE_CM = 50.0;     // Final forward distance after last turn


// // rcl_subscription_t subscriber;
// // rcl_publisher_t publisher;
// // geometry_msgs__msg__Twist cmd;
// // std_msgs__msg__Float32 msg;

// // // Global variables for encoder reading
// // uint8_t last_state;
// // float total_distance_cm = 0.0;
// // float distance_per_tick_cm;

// // // Function prototypes
// // int8_t read_encoder(uint8_t* last_state);
// // float calculate_distance_per_tick_cm(float radius_cm, int ticks_per_revolution);
// // void move_forward(float distance_cm);
// // void turn(float angle_degrees);
// // void perform_movement_sequence();

// // // function to set pwm and direction
// // void set_motor(uint gpio_pwm, uint gpio_dir, float speed)
// // {
// //     if(speed >= 0)
// //     {
// //         gpio_put(gpio_dir, 0); // forward
// //         pwm_set_gpio_level(gpio_pwm, STATIC_PWM_VALUE);
// //     }
// //     else
// //     {
// //         gpio_put(gpio_dir, 1); // backward
// //         pwm_set_gpio_level(gpio_pwm, STATIC_PWM_VALUE);
// //     }
// // }


// // // Function to read encoder and determine direction
// // int8_t read_encoder(uint8_t* last_state) {
// //     uint8_t current_state = (gpio_get(ENCODER_PIN_A) << 1) | gpio_get(ENCODER_PIN_B);
// //     int8_t direction = 0;
// //     uint8_t transition = (*last_state << 2) | current_state;

// //     switch (transition) {
// //         case 0b0001:
// //         case 0b0111:
// //         case 0b1110:
// //         case 0b1000:
// //             direction = 1; // Clockwise
// //             break;
// //         case 0b0010:
// //         case 0b0100:
// //         case 0b1101:
// //         case 0b1011:
// //             direction = -1; // Counterclockwise
// //             break;
// //     }

// //     *last_state = current_state; // Update last state
// //     return direction; // Return direction
// // }

// // // Function to calculate distance per tick in centimeters
// // float calculate_distance_per_tick_cm(float radius_cm, int ticks_per_revolution) {
// //     float circumference_cm = 2 * M_PI * radius_cm; // Circumference in cm
// //     return circumference_cm / ticks_per_revolution; // Distance per tick in cm
// // }

// // void move_forward(float distance_cm) {
// //     float start_distance = total_distance_cm;
// //     while (total_distance_cm - start_distance < distance_cm) {
// //         set_motor(PWM_PIN_FR, DIR_PIN_FR, 0.5);
// //         set_motor(PWM_PIN_FL, DIR_PIN_FL, 0.5);
// //         set_motor(PWM_PIN_BL, DIR_PIN_BL, 0.5);
// //         set_motor(PWM_PIN_BR, PWM_PIN_BR, 0.5);
        
// //         // Update encoder reading
// //         int8_t direction = read_encoder(&last_state);
// //         total_distance_cm += direction * distance_per_tick_cm;
        
// //         // Small delay to prevent busy-waiting
// //         sleep_ms(10);
// //     }
// //     // Stop motors
// //     set_motor(PWM_PIN_FR, DIR_PIN_FR, 0);
// //     set_motor(PWM_PIN_FL, DIR_PIN_FL, 0);
// //     set_motor(PWM_PIN_BL, DIR_PIN_BL, 0);
// //     set_motor(PWM_PIN_BR, PWM_PIN_BR, 0);
// // }

// // void turn(float angle_degrees) {
// //     // Assuming positive angle is right turn and negative is left turn
// //     float turn_time_ms = std::abs(angle_degrees) * 20; // Adjust this multiplier based on your robot's turning speed
// //     float turn_speed = (angle_degrees > 0) ? 0.3 : -0.3;
    
// //     set_motor(PWM_PIN_FR, DIR_PIN_FR, -turn_speed);
// //     set_motor(PWM_PIN_FL, DIR_PIN_FL, turn_speed);
// //     set_motor(PWM_PIN_BL, DIR_PIN_BL, -turn_speed);
// //     set_motor(PWM_PIN_BR, PWM_PIN_BR, turn_speed);
    
// //     sleep_ms(turn_time_ms);
    
// //     // Stop motors
// //     set_motor(PWM_PIN_FR, DIR_PIN_FR, 0);
// //     set_motor(PWM_PIN_FL, DIR_PIN_FL, 0);
// //     set_motor(PWM_PIN_BL, DIR_PIN_BL, 0);
// //     set_motor(PWM_PIN_BR, PWM_PIN_BR, 0);
// // }

// // void perform_movement_sequence() {
// //     move_forward(FORWARD_DISTANCE_CM);
// //     turn(TURN_ANGLE_DEGREES);  // Turn right
// //     move_forward(FORWARD_DISTANCE_CM);
// //     turn(-TURN_ANGLE_DEGREES); // Turn left
// //     move_forward(FINAL_DISTANCE_CM);
// // }



// // void cmd_vel_callback(const void * msgin)
// // {
// //     const geometry_msgs__msg__Twist * cmd = (const geometry_msgs__msg__Twist *)msgin;
// //     float Vx = cmd->linear.x;
// //     float Vy = cmd->linear.y;
// //     float W = cmd->angular.z;

// //     // Calculate wheel speeds based on received velocities
// //     float V4 = Vx - Vy + W;
// //     float V3 = Vx + Vy - W;
// //     float V2 = Vx + Vy + W;
// //     float V1 = Vx - Vy - W;

// //     // Set motor directions based on calculated speeds
// //     set_motor(PWM_PIN_FR, DIR_PIN_FR, V1);
// //     set_motor(PWM_PIN_FL, DIR_PIN_FL, V2);
// //     set_motor(PWM_PIN_BL, DIR_PIN_BL, V3);
// //     set_motor(PWM_PIN_BR, DIR_PIN_BL, V4);
// // }

// // int main()
// // {
// //     // Initialize GPIO and PWM
// //     stdio_init_all();

// //     // Set up micro-ROS transport
// //     rmw_uros_set_custom_transport(
// //         true,
// //         NULL,
// //         pico_serial_transport_open,
// //         pico_serial_transport_close,
// //         pico_serial_transport_write,
// //         pico_serial_transport_read
// //     );

// //     // Check if micro-ROS agent is reachable
// //     if (rmw_uros_ping_agent(1000, 120) != RCL_RET_OK) {
// //         printf("Unable to reach micro-ROS agent\n");
// //         return -1;
// //     }


// //     gpio_init(ENCODER_PIN_A);
// //     gpio_init(ENCODER_PIN_B);
// //     gpio_init(LED_PIN);
// //     gpio_set_dir(LED_PIN, GPIO_IN);
// //     gpio_put(LED_PIN, 0);
// //     gpio_set_dir(ENCODER_PIN_A, GPIO_IN);
// //     gpio_set_dir(ENCODER_PIN_B, GPIO_IN);
// //     gpio_pull_up(ENCODER_PIN_A);
// //     gpio_pull_up(ENCODER_PIN_B);

// //     gpio_init(PWM_PIN_FR);
// //     gpio_set_function(PWM_PIN_FR, GPIO_FUNC_PWM);
// //     gpio_init(DIR_PIN_FR);
// //     gpio_set_dir(DIR_PIN_FR, GPIO_OUT);

// //     gpio_init(PWM_PIN_FL);
// //     gpio_set_function(PWM_PIN_FL, GPIO_FUNC_PWM);
// //     gpio_init(DIR_PIN_FL);
// //     gpio_set_dir(DIR_PIN_FL, GPIO_OUT);

// //     gpio_init(PWM_PIN_BL);
// //     gpio_set_function(PWM_PIN_BL, GPIO_FUNC_PWM);
// //     gpio_init(DIR_PIN_BL);
// //     gpio_set_dir(DIR_PIN_BL, GPIO_OUT);

// //     gpio_init(PWM_PIN_BR);
// //     gpio_set_function(PWM_PIN_BR, GPIO_FUNC_PWM);
// //     gpio_init(DIR_PIN_BL);
// //     gpio_set_dir(DIR_PIN_BL, GPIO_OUT);

// //     // Initialize PWM
// //     uint slice_num1 = pwm_gpio_to_slice_num(PWM_PIN_FR);
// //     uint slice_num2 = pwm_gpio_to_slice_num(PWM_PIN_FL);
// //     uint slice_num3 = pwm_gpio_to_slice_num(PWM_PIN_BL);
// //     uint slice_num4 = pwm_gpio_to_slice_num(PWM_PIN_BR);

// //     pwm_set_wrap(slice_num1, PWM_RANGE);
// //     pwm_set_wrap(slice_num2, PWM_RANGE);
// //     pwm_set_wrap(slice_num3, PWM_RANGE);
// //     pwm_set_wrap(slice_num4, PWM_RANGE);

// //     pwm_set_clkdiv(slice_num1, 256.f);
// //     pwm_set_clkdiv(slice_num2, 256.f);
// //     pwm_set_clkdiv(slice_num3, 256.f);
// //     pwm_set_clkdiv(slice_num4, 256.f);

// //     pwm_set_gpio_level(PWM_PIN_FR, STATIC_PWM_VALUE);
// //     pwm_set_gpio_level(PWM_PIN_FL, STATIC_PWM_VALUE);
// //     pwm_set_gpio_level(PWM_PIN_BL, STATIC_PWM_VALUE);
// //     pwm_set_gpio_level(PWM_PIN_BR, STATIC_PWM_VALUE);

// //     pwm_set_enabled(slice_num1, true);
// //     pwm_set_enabled(slice_num2, true);
// //     pwm_set_enabled(slice_num3, true);
// //     pwm_set_enabled(slice_num4, true);

// //     // Initialize micro-ROS allocator
// //     rcl_allocator_t allocator = rcl_get_default_allocator();

// //     // create init_options
// //     rclc_support_t support;
// //     rclc_support_init(&support, 0, NULL, &allocator);

// //     // Create node
// //     rcl_node_t node;
// //     rclc_node_init_default(&node, "pico_node", "", &support);

// //     // Create subscriber
// //     rclc_subscription_init_default(
// //         &subscriber,
// //         &node,
// //         ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
// //         "/cmd_vel");

// //     rclc_publisher_init_default(
// //         &publisher, 
// //         &node, 
// //         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
// //         "pico_publisher");

// //     // Create executor
// //     rclc_executor_t executor;
// //     rclc_executor_init(&executor, &support.context, 2, &allocator);
// //     rclc_executor_add_subscription(&executor, &subscriber, &cmd, &cmd_vel_callback, ON_NEW_DATA);

// //     msg.data = 0.0;
// //     last_state = (gpio_get(ENCODER_PIN_A) << 1) | gpio_get(ENCODER_PIN_B);

// //     distance_per_tick_cm = calculate_distance_per_tick_cm(WHEEL_RADIUS_CM, TICKS_PER_REVOLUTION);
// //     total_distance_cm = 0.0;

// //     // Spin 
// //     while (true) {
// //         // Publish final position
// //         // set led pin to high
// //         gpio_put(LED_PIN, 0);
// //         int8_t direction = read_encoder(&last_state);
// //         total_distance_cm += direction * distance_per_tick_cm;
// //         msg.data = total_distance_cm;
// //         // if (rcl_publish(&publisher, &msg, NULL) != RCL_RET_OK) {
// //         //     printf("Error publishing message\n");
// //         // }
    
// //         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
// //         sleep_ms(UPDATE_INTERVAL_MS);
// //     }

// //     // Cleanup
// //     rcl_subscription_fini(&subscriber, &node);
// //     rcl_node_fini(&node);

// //     return 0;
// // }
// // paras
// // // Includes
// // #include <rcl/rcl.h>
// // #include <rclc/rclc.h>
// // #include <rclc/executor.h>
// // #include <geometry_msgs/msg/twist.h>
// // #include <pico/stdlib.h>
// // #include <hardware/pwm.h>
// // #include <rmw_microros/rmw_microros.h>
// // #include "pico_uart_transports.h"
// // #include <math.h>

// // // Constants
// // #define PWM_FREQ 1000

// // #define PWM_PIN_FR 16
// // #define DIR_PIN_FR 17

// // #define PWM_PIN_FL 18
// // #define DIR_PIN_FL 19

// // #define PWM_PIN_BL 20
// // #define DIR_PIN_BL 21

// // #define PWM_PIN_BR 2
// // #define DIR_PIN_BL 3

// // #define UPDATE_INTERVAL_MS 10  // 10ms update interval for smoother motion
// // #define ACCEL_RATE 0.1f  // Acceleration rate (adjust as needed)
// // #define LOW_PASS_ALPHA 0.2f  // Low-pass filter alpha (adjust as needed)

// // rcl_subscription_t subscriber;
// // geometry_msgs__msg__Twist cmd;

// // typedef struct {
// //     float current_speed;
// //     float target_speed;
// // } MotorState;

// // MotorState motors[4];

// // float low_pass_filter(float new_value, float old_value) {
// //     return LOW_PASS_ALPHA * new_value + (1 - LOW_PASS_ALPHA) * old_value;
// // }

// // void update_motor_speed(int motor_index) {
// //     MotorState* motor = &motors[motor_index];
// //     float diff = motor->target_speed - motor->current_speed;
// //     if (fabsf(diff) > ACCEL_RATE) {
// //         motor->current_speed += (diff > 0) ? ACCEL_RATE : -ACCEL_RATE;
// //     } else {
// //         motor->current_speed = motor->target_speed;
// //     }
// // }

// // // function to set pwm and direction
// // void set_motor(uint gpio_pwm, uint gpio_dir, float speed)
// // {
// //     // set direction
// //     if(speed >= 0)
// //     {
// //         gpio_put(gpio_dir, 0); // forward
// //     }
// //     else
// //     {
// //         gpio_put(gpio_dir, 1); // backward
// //         speed = -speed;
// //     }

// //     // Convert Speed to PWM duty cycle
// //     pwm_set_gpio_level(gpio_pwm, speed * 1250);
// // }

// // void cmd_vel_callback(const void * msgin)
// // {
// //     const geometry_msgs__msg__Twist * cmd = (const geometry_msgs__msg__Twist *)msgin;
// //     float Vx = cmd->linear.x;
// //     float Vy = cmd->linear.y;
// //     float W = cmd->angular.z;

// //     // Calculate the wheel speeds
// //     float V1 = Vx - Vy - W;
// //     float V2 = Vx + Vy + W;
// //     float V3 = Vx + Vy - W;
// //     float V4 = Vx - Vy + W;

// //     // Update target speeds with low-pass filter
// //     motors[0].target_speed = low_pass_filter(-V1, motors[0].target_speed);
// //     motors[1].target_speed = low_pass_filter(-V2, motors[1].target_speed);
// //     motors[2].target_speed = low_pass_filter(-V3, motors[2].target_speed);
// //     motors[3].target_speed = low_pass_filter(-V4, motors[3].target_speed);
// // }

// // int main()
// // {
// //     // Initialize GPIO and PWM
// //     stdio_init_all();

// //     // Set up micro-ROS transport
// //     rmw_uros_set_custom_transport(
// //         true,
// //         NULL,
// //         pico_serial_transport_open,
// //         pico_serial_transport_close,
// //         pico_serial_transport_write,
// //         pico_serial_transport_read
// //     );

// //     // Check if micro-ROS agent is reachable
// //     if (rmw_uros_ping_agent(1000, 120) != RCL_RET_OK) {
// //         printf("Unable to reach micro-ROS agent\n");
// //         return -1;
// //     }

// //     gpio_init(PWM_PIN_FR);
// //     gpio_set_function(PWM_PIN_FR, GPIO_FUNC_PWM);
// //     gpio_init(DIR_PIN_FR);
// //     gpio_set_dir(DIR_PIN_FR, GPIO_OUT);

// //     gpio_init(PWM_PIN_FL);
// //     gpio_set_function(PWM_PIN_FL, GPIO_FUNC_PWM);
// //     gpio_init(DIR_PIN_FL);
// //     gpio_set_dir(DIR_PIN_FL, GPIO_OUT);

// //     gpio_init(PWM_PIN_BL);
// //     gpio_set_function(PWM_PIN_BL, GPIO_FUNC_PWM);
// //     gpio_init(DIR_PIN_BL);
// //     gpio_set_dir(DIR_PIN_BL, GPIO_OUT);

// //     gpio_init(PWM_PIN_BR);
// //     gpio_set_function(PWM_PIN_BR, GPIO_FUNC_PWM);
// //     gpio_init(DIR_PIN_BL);
// //     gpio_set_dir(DIR_PIN_BL, GPIO_OUT);

// //     // Initialize PWM
// //     uint slice_num1 = pwm_gpio_to_slice_num(PWM_PIN_FR);
// //     uint slice_num2 = pwm_gpio_to_slice_num(PWM_PIN_FL);
// //     uint slice_num3 = pwm_gpio_to_slice_num(PWM_PIN_BL);
// //     uint slice_num4 = pwm_gpio_to_slice_num(PWM_PIN_BR);

// //     pwm_set_wrap(slice_num1, 1250);
// //     pwm_set_wrap(slice_num2, 1250);
// //     pwm_set_wrap(slice_num3, 1250);
// //     pwm_set_wrap(slice_num4, 1250);

// //     pwm_set_clkdiv(slice_num1, 64.f);
// //     pwm_set_clkdiv(slice_num2, 64.f);
// //     pwm_set_clkdiv(slice_num3, 64.f);
// //     pwm_set_clkdiv(slice_num4, 64.f);

// //     pwm_set_gpio_level(PWM_PIN_FR, 0);
// //     pwm_set_gpio_level(PWM_PIN_FL, 0);
// //     pwm_set_gpio_level(PWM_PIN_BL, 0);
// //     pwm_set_gpio_level(PWM_PIN_BR, 0);

// //     pwm_set_enabled(slice_num1, true);
// //     pwm_set_enabled(slice_num2, true);
// //     pwm_set_enabled(slice_num3, true);
// //     pwm_set_enabled(slice_num4, true);

// //     // Initialize motor states
// //     for (int i = 0; i < 4; i++) {
// //         motors[i].current_speed = 0;
// //         motors[i].target_speed = 0;
// //     }

// //     // Initialize micro-ROS allocator
// //     rcl_allocator_t allocator = rcl_get_default_allocator();

// //     // create init_options
// //     rclc_support_t support;
// //     rclc_support_init(&support, 0, NULL, &allocator);

// //     // Create node
// //     rcl_node_t node;
// //     rclc_node_init_default(&node, "pico_node", "", &support);

// //     // Create subscriber
// //     rclc_subscription_init_default(
// //         &subscriber,
// //         &node,
// //         ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
// //         "/cmd_vel");

// //     // Create executor
// //     rclc_executor_t executor;
// //     rclc_executor_init(&executor, &support.context, 1, &allocator);
// //     rclc_executor_add_subscription(&executor, &subscriber, &cmd, &cmd_vel_callback, ON_NEW_DATA);

// //     // Spin 
// //     while (true) {
// //         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        
// //         // Update motor speeds
// //         for (int i = 0; i < 4; i++) {
// //             update_motor_speed(i);
// //         }

// //         // Set motor speeds
// //         set_motor(PWM_PIN_FR, DIR_PIN_FR, motors[0].current_speed);
// //         set_motor(PWM_PIN_FL, DIR_PIN_FL, motors[1].current_speed);
// //         set_motor(PWM_PIN_BL, DIR_PIN_BL, motors[2].current_speed);
// //         set_motor(PWM_PIN_BR, DIR_PIN_BL, motors[3].current_speed);

// //         sleep_ms(UPDATE_INTERVAL_MS);
// //     }

// //     // Cleanup
// //     rcl_subscription_fini(&subscriber, &node);
// //     rcl_node_fini(&node);

// //     return 0;
// // }

// // Includes
// #include <stdio.h>
// #include <math.h>
// #include <rcl/rcl.h>
// #include <rcl/error_handling.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <std_msgs/msg/float32.h>
// #include <rmw_microros/rmw_microros.h>
// #include "pico/stdlib.h"
// #include "pico_uart_transports.h"

// // Constants
// const uint ENCODER_PIN_A = 4;
// const uint ENCODER_PIN_B = 5;
// const int TICKS_PER_REVOLUTION = 60; // Ticks per full rotation of the encoder
// const float WHEEL_RADIUS_CM = 2.8; // Wheel radius in cm (56 mm)

// // Global Variables pub/sub/service
// rcl_publisher_t publisher;
// std_msgs__msg__Float32 msg;

// // Function to read encoder and determine direction
// int8_t read_encoder(uint8_t* last_state) {
//     uint8_t current_state = (gpio_get(ENCODER_PIN_A) << 1) | gpio_get(ENCODER_PIN_B);
//     int8_t direction = 0;
//     uint8_t transition = (*last_state << 2) | current_state;

//     switch (transition) {
//         case 0b0001:
//         case 0b0111:
//         case 0b1110:
//         case 0b1000:
//             direction = 1; // Clockwise
//             break;
//         case 0b0010:
//         case 0b0100:
//         case 0b1101:
//         case 0b1011:
//             direction = -1; // Counterclockwise
//             break;
//     }

//     *last_state = current_state; // Update last state
//     return direction; // Return direction
// }

// // Function to calculate distance per tick in centimeters
// float calculate_distance_per_tick_cm(float radius_cm, int ticks_per_revolution) {
//     float circumference_cm = 2 * M_PI * radius_cm; // Circumference in cm
//     return ((circumference_cm / ticks_per_revolution)*(1)); // Distance per tick in cm
// }

// // Main function
// int main() {
//     stdio_init_all();
//     rmw_uros_set_custom_transport(
//         true,
//         NULL,
//         pico_serial_transport_open,
//         pico_serial_transport_close,
//         pico_serial_transport_write,
//         pico_serial_transport_read
//     );

//     rcl_node_t node;
//     rcl_allocator_t allocator = rcl_get_default_allocator();
//     rclc_support_t support;
//     rclc_executor_t executor;

//     gpio_init(ENCODER_PIN_A);
//     gpio_init(ENCODER_PIN_B);
//     gpio_set_dir(ENCODER_PIN_A, GPIO_IN);
//     gpio_set_dir(ENCODER_PIN_B, GPIO_IN);
//     gpio_pull_up(ENCODER_PIN_A);
//     gpio_pull_up(ENCODER_PIN_B);

//     // Check if micro-ROS agent is reachable
//     if (rmw_uros_ping_agent(1000, 120) != RCL_RET_OK) {
//         return -1; // Exit if unreachable
//     }

//     // Initialize support, node, and publisher
//     rclc_support_init(&support, 0, NULL, &allocator);
//     rclc_node_init_default(&node, "pico_node", "", &support);
//     rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "pico_publisher");
//     rclc_executor_init(&executor, &support.context, 1, &allocator);

//     msg.data = 0.0;
//     uint8_t last_state = (gpio_get(ENCODER_PIN_A) << 1) | gpio_get(ENCODER_PIN_B);

//     float distance_per_tick_cm = calculate_distance_per_tick_cm(WHEEL_RADIUS_CM, TICKS_PER_REVOLUTION);
//     float total_distance_cm = 0.0;

//     // Main loop
//     while (true) {
//         int8_t direction = read_encoder(&last_state);
//         total_distance_cm += direction * distance_per_tick_cm;
//         msg.data = total_distance_cm;

//         // Publish message
//         if (rcl_publish(&publisher, &msg, NULL) != RCL_RET_OK) {
//             printf("Error publishing message\n");
//         }

//         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//     }

//     // Cleanup
//     rclc_executor_fini(&executor);
//     rcl_publisher_fini(&publisher, &node);
//     rcl_node_fini(&node);
//     rclc_support_fini(&support);

//     return 0;
// }


// Includes
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>
#include <math.h>
#include "pico_uart_transports.h"

// Constants
#define PWM_FREQ 1000

#define PWM_PIN_FR 16
#define DIR_PIN_FR 17

#define PWM_PIN_FL 18
#define DIR_PIN_FL 19

#define PWM_PIN_BL 20
#define DIR_PIN_BL 21

#define PWM_PIN_BR 6
#define DIR_PIN_BR 7

// const uint ENCODER_PIN_A = 6;
// const uint ENCODER_PIN_B = 7;

// Speed Scaling factor to reduce motor speed
#define SPEED_SCALING_FACTOR 0.5 // 0.5 for half speed

const int TICKS_PER_REVOLUTION = 80;  // Ticks per full rotation of the encoder
const float WHEEL_RADIUS_CM = 2.8;    // Wheel radius in cm
const int TARGET_TICKS = 500;   
// const float M_PI = 3.14159265358979323846;

// New constants for the movement sequence
const float FORWARD_DISTANCE_CM = 100.0;  // Distance to move forward
const float TURN_ANGLE_DEGREES = 90.0;    // Angle to turn (90 degrees for right turn)
const float FINAL_DISTANCE_CM = 50.0;     // Final forward distance after last turn

float distance_per_tick_cm;
float total_distance_cm;

rcl_subscription_t subscriber_line;
rcl_subscription_t subscriber_ball;
rcl_publisher_t publisher;
geometry_msgs__msg__Twist cmd;
std_msgs__msg__Float32 msg;

int encoder_count = 0;
bool target_reached = false;

// Function to read encoder and determine direction
// int8_t read_encoder(uint8_t* last_state) {
//     uint8_t current_state = (gpio_get(ENCODER_PIN_A) << 1) | gpio_get(ENCODER_PIN_B);
//     int8_t direction = 0;
//     uint8_t transition = (*last_state << 2) | current_state;
    
//     switch (transition) {
//         case 0b0001:
//         case 0b0111:
//         case 0b1110:
//         case 0b1000:
//             direction = 1; // Clockwise
//             break;
//         case 0b0010:
//         case 0b0100:
//         case 0b1101:
//         case 0b1011:
//             direction = -1; // Counterclockwise
//             break;
//     }
    
//     *last_state = current_state;
//     return direction;
// }

// Function to calculate distance per tick in centimeters
// float calculate_distance_per_tick_cm(float radius_cm, int ticks_per_revolution) {
//     float circumference_cm = 2 * M_PI * radius_cm; // Circumference in cm
//     return ((circumference_cm / ticks_per_revolution)*(1)); // Distance per tick in cm
// }

// function to set pwm and direction
void set_motor(uint gpio_pwm, uint gpio_dir, float speed)
{
    // Apply the Speed Scaling Factor
    // speed *= SPEED_SCALING_FACTOR;

    // set direction
    if(speed >= 0)
    {
        gpio_put(gpio_dir, 0); // forward
    }
    else
    {
        gpio_put(gpio_dir, 1); // backward
        speed = -speed;
    }

    // Convert Speed to PWM duty cycle
    // uint16_t duty_cycle = (uint16_t)(speed * 65535);
    pwm_set_gpio_level(gpio_pwm, speed);
}

// void move_forward(float distance_cm) {
//     float start_distance = total_distance_cm;
//     while (total_distance_cm - start_distance < distance_cm) {
//         set_motor(PWM_PIN_FL, DIR_PIN_FL, 0.5);
//         set_motor(PWM_PIN_FR, DIR_PIN_FR, 0.5);
//         set_motor(PWM_PIN_BL, DIR_PIN_BL, 0.5);
//         set_motor(PWM_PIN_BR, DIR_PIN_BR, 0.5);
        
//         // Update encoder reading
//         // int8_t direction = read_encoder(&last_state);
//         // total_distance_cm += direction * distance_per_tick_cm;
        
//         // Small delay to prevent busy-waiting
//         sleep_ms(10);
//     }
//     // Stop motors
//     set_motor(PWM_PIN_FL, DIR_PIN_FL, 0);
//     set_motor(PWM_PIN_FR, DIR_PIN_FR, 0);
//     set_motor(PWM_PIN_BL, DIR_PIN_BL, 0);
//     set_motor(PWM_PIN_BR, DIR_PIN_BR, 0);
// }

void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * cmd = (const geometry_msgs__msg__Twist *)msgin;
    // float Vx = cmd->linear.x;
    // float Vy = cmd->linear.y;
    // float W = cmd->angular.z;

    // // Calculate the wheel speeds
    // float V4 = Vx - Vy - W;
    // float V3 = Vx + Vy + W;
    // float V2 = Vx + Vy - W;
    // float V1 = Vx - Vy + W;

    double linear_x = cmd->linear.x;
    double linear_y = cmd->linear.y;
    double angular_z = cmd->angular.z;

    int val1, val2, val3, val4 = 0;

    if(linear_x > 0.0)
    {
        val1 = 300;
        val2 = 300;
        val3 = 300;
        val4 = 300;
    }

    else if(linear_x < 0.0)
    {
        val1 = -300;
        val2 = -300;
        val3 = -300;
        val4 = -300;
    }

    else if(angular_z > 0.0)
    {
        val1 = 300;
        val2 = -300;
        val3 = -300;
        val4 = 300;
    }

    else if(angular_z < 0.0)
    {
        val1 = -300;
        val2 = 300;
        val3 = 300;
        val4 = -300;
    }

    else
    {
        val1 = 0;
        val2 = 0;
        val3 = 0;
        val4 = 0;
    }


    // Set Motor Speed and Directions
    set_motor(PWM_PIN_FR, DIR_PIN_FR, val1);
    set_motor(PWM_PIN_FL, DIR_PIN_FL, val2);
    set_motor(PWM_PIN_BL, DIR_PIN_BL, val3);
    set_motor(PWM_PIN_BR, DIR_PIN_BR, val4);
}

void cmd_vel_callback_ball(const void * msgin)
{
    const geometry_msgs__msg__Twist * cmd = (const geometry_msgs__msg__Twist *)msgin;
    float Vx = cmd->linear.x;
    float Vy = cmd->linear.y;
    float W = cmd->angular.z;

    // Calculate the wheel speeds
    float V1 = Vx - Vy - W;
    float V2 = Vx + Vy + W;
    float V3 = Vx + Vy - W;
    float V4 = Vx - Vy + W;

    // Set Motor Speed and Directions
    set_motor(PWM_PIN_FR, DIR_PIN_FR, V1);
    set_motor(PWM_PIN_FL, DIR_PIN_FL, V2);
    set_motor(PWM_PIN_BL, DIR_PIN_BL, V3);
    set_motor(PWM_PIN_BR, DIR_PIN_BR, V4);
}

int main()
{
    // Initialize GPIO and PWM
    stdio_init_all();

    // Set up micro-ROS transport
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Check if micro-ROS agent is reachable
    if (rmw_uros_ping_agent(1000, 120) != RCL_RET_OK) {
        printf("Unable to reach micro-ROS agent\n");
        return -1;
    }

    // Initialize GPIO and PWM
    // gpio_init(ENCODER_PIN_A);
    // gpio_init(ENCODER_PIN_B);
    // gpio_set_dir(ENCODER_PIN_A, GPIO_IN);
    // gpio_set_dir(ENCODER_PIN_B, GPIO_IN);
    // gpio_pull_up(ENCODER_PIN_A);
    // gpio_pull_up(ENCODER_PIN_B);

    gpio_init(PWM_PIN_FR);
    gpio_set_function(PWM_PIN_FR, GPIO_FUNC_PWM);
    gpio_init(DIR_PIN_FR);
    gpio_set_dir(DIR_PIN_FR, GPIO_OUT);

    gpio_init(PWM_PIN_FL);
    gpio_set_function(PWM_PIN_FL, GPIO_FUNC_PWM);
    gpio_init(DIR_PIN_FL);
    gpio_set_dir(DIR_PIN_FL, GPIO_OUT);

    gpio_init(PWM_PIN_BL);
    gpio_set_function(PWM_PIN_BL, GPIO_FUNC_PWM);
    gpio_init(DIR_PIN_BL);
    gpio_set_dir(DIR_PIN_BL, GPIO_OUT);

    gpio_init(PWM_PIN_BR);
    gpio_set_function(PWM_PIN_BR, GPIO_FUNC_PWM);
    gpio_init(DIR_PIN_BR);
    gpio_set_dir(DIR_PIN_BR, GPIO_OUT);

    // Initialize PWM
    uint slice_num1 = pwm_gpio_to_slice_num(PWM_PIN_FR);
    uint slice_num2 = pwm_gpio_to_slice_num(PWM_PIN_FL);
    uint slice_num3 = pwm_gpio_to_slice_num(PWM_PIN_BL);
    uint slice_num4 = pwm_gpio_to_slice_num(PWM_PIN_BR);

    pwm_set_wrap(slice_num1, 1250);
    pwm_set_wrap(slice_num2, 1250);
    pwm_set_wrap(slice_num3, 1250);
    pwm_set_wrap(slice_num4, 1250);

    pwm_set_clkdiv(slice_num1, 64.f);
    pwm_set_clkdiv(slice_num2, 64.f);
    pwm_set_clkdiv(slice_num3, 64.f);
    pwm_set_clkdiv(slice_num4, 64.f);

    pwm_set_gpio_level(PWM_PIN_FR, 0);
    pwm_set_gpio_level(PWM_PIN_FL, 0);
    pwm_set_gpio_level(PWM_PIN_BL, 0);
    pwm_set_gpio_level(PWM_PIN_BR, 0);

    pwm_set_enabled(slice_num1, true);
    pwm_set_enabled(slice_num2, true);
    pwm_set_enabled(slice_num3, true);
    pwm_set_enabled(slice_num4, true);

    // Initialize micro-ROS allocator
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // create init_options
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    // Create node
    rcl_node_t node;
    rclc_node_init_default(&node, "pico_node", "", &support);

    // Create subscriber
    rclc_subscription_init_default(
        &subscriber_line,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel/line");
    
    rclc_subscription_init_default(
        &subscriber_ball,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel/ball");

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/encoder");

    // Create executor
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber_line, &cmd, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &subscriber_ball, &cmd, &cmd_vel_callback_ball, ON_NEW_DATA);

    msg.data = 0.0;
    // uint8_t last_state = (gpio_get(ENCODER_PIN_A) << 1) | gpio_get(ENCODER_PIN_B);

    // Spin 
    while (true) {

        // int8_t direction = read_encoder(&last_state);
        // encoder_count += direction;
                
        // Publish encoder count
        // msg.data = (float)encoder_count;
        rcl_publish(&publisher, &msg, NULL);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        // sleep_ms(100);
    //     set_motor(PWM_PIN_FR, DIR_PIN_FR, -100);
    //     set_motor(PWM_PIN_FL, DIR_PIN_FL, -100);
    //     set_motor(PWM_PIN_BL, DIR_PIN_BL, -100);
    //     set_motor(PWM_PIN_BR, DIR_PIN_BR, -100);
    }

    // Cleanup
    rcl_subscription_fini(&subscriber_line, &node);
    rcl_subscription_fini(&subscriber_ball, &node); 
    rcl_node_fini(&node);

    return 0;
}