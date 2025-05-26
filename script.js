document.addEventListener('DOMContentLoaded', function() {
    // DOM Elements
    const userInput = document.getElementById('user-input');
    const sendBtn = document.getElementById('send-btn');
    const messageContainer = document.getElementById('message-container');
    const themeToggle = document.querySelector('.theme-toggle');
    const themeIcon = document.getElementById('theme-icon');
    const codeBtn = document.getElementById('code-btn');
    const hardwareBtn = document.getElementById('hardware-btn');
    const rosBtn = document.getElementById('ros-btn');
    const clearBtn = document.getElementById('clear-btn');
    const codeModal = document.getElementById('code-modal');
    const closeModal = document.querySelector('.close-modal');
    const generateCodeBtn = document.getElementById('generate-code-btn');
    const exampleBtns = document.querySelectorAll('.example-btn');
    const rosCommandsBtn = document.getElementById('ros-commands');
    const embeddedSnippetsBtn = document.getElementById('embedded-snippets');
    
    // Theme toggle
    themeToggle.addEventListener('click', toggleTheme);
    
    // Send message on button click or Enter key
    sendBtn.addEventListener('click', sendMessage);
    userInput.addEventListener('keydown', function(e) {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            sendMessage();
        }
    });
    
    // Tool buttons
    codeBtn.addEventListener('click', openCodeModal);
    hardwareBtn.addEventListener('click', () => askAboutHardware());
    rosBtn.addEventListener('click', () => askAboutROS());
    clearBtn.addEventListener('click', clearChat);
    
    // Example buttons
    exampleBtns.forEach(btn => {
        btn.addEventListener('click', function() {
            const exampleType = this.getAttribute('data-example');
            loadExample(exampleType);
        });
    });
    
    // Quick tools
    rosCommandsBtn.addEventListener('click', () => askAboutROSCommands());
    embeddedSnippetsBtn.addEventListener('click', () => askAboutEmbeddedSnippets());
    
    // Code modal
    closeModal.addEventListener('click', () => codeModal.style.display = 'none');
    generateCodeBtn.addEventListener('click', generateCode);
    
    // Close modal when clicking outside
    window.addEventListener('click', function(e) {
        if (e.target === codeModal) {
            codeModal.style.display = 'none';
        }
    });
    
    // Functions
    function toggleTheme() {
        document.body.classList.toggle('light-theme');
        const isLight = document.body.classList.contains('light-theme');
        themeIcon.className = isLight ? 'fas fa-sun' : 'fas fa-moon';
        localStorage.setItem('theme', isLight ? 'light' : 'dark');
    }
    
    function sendMessage() {
        const message = userInput.value.trim();
        if (message === '') return;
        
        // Add user message to chat
        addMessage(message, 'user');
        userInput.value = '';
        
        // Show typing indicator
        showTypingIndicator();
        
        // Simulate API call with timeout
        setTimeout(() => {
            removeTypingIndicator();
            const response = getBotResponse(message);
            addMessage(response, 'bot');
        }, 1000 + Math.random() * 2000); // Random delay between 1-3 seconds
    }
    
    function addMessage(content, sender) {
        const messageDiv = document.createElement('div');
        messageDiv.className = `message ${sender}-message`;
        
        const headerDiv = document.createElement('div');
        headerDiv.className = 'message-header';
        
        if (sender === 'bot') {
            headerDiv.innerHTML = '<i class="fas fa-robot"></i><span>RoboEmbed</span>';
        } else {
            headerDiv.innerHTML = '<i class="fas fa-user"></i><span>You</span>';
        }
        
        const contentDiv = document.createElement('div');
        contentDiv.className = 'message-content';
        
        // Check if content contains code blocks and format them
        const formattedContent = formatCodeBlocks(content);
        contentDiv.innerHTML = formattedContent;
        
        messageDiv.appendChild(headerDiv);
        messageDiv.appendChild(contentDiv);
        messageContainer.appendChild(messageDiv);
        
        // Scroll to bottom
        messageContainer.scrollTop = messageContainer.scrollHeight;
        
        // Highlight any code blocks
        document.querySelectorAll('pre code').forEach(block => {
            hljs.highlightElement(block);
        });
    }
    
    function formatCodeBlocks(text) {
        // Simple code block formatting (in a real app, use Markdown or similar)
        return text.replace(/```(\w*)\n([\s\S]*?)\n```/g, function(match, lang, code) {
            return `<pre><code class="language-${lang || 'plaintext'}">${escapeHtml(code)}</code></pre>`;
        });
    }
    
    function escapeHtml(unsafe) {
        return unsafe
            .replace(/&/g, "&amp;")
            .replace(/</g, "&lt;")
            .replace(/>/g, "&gt;")
            .replace(/"/g, "&quot;")
            .replace(/'/g, "&#039;");
    }
    
    function showTypingIndicator() {
        const typingDiv = document.createElement('div');
        typingDiv.className = 'message bot-message typing-indicator';
        typingDiv.id = 'typing-indicator';
        typingDiv.innerHTML = `
            <div class="message-header">
                <i class="fas fa-robot"></i>
                <span>RoboEmbed</span>
            </div>
            <div class="typing-dots">
                <span></span>
                <span></span>
                <span></span>
            </div>
        `;
        messageContainer.appendChild(typingDiv);
        messageContainer.scrollTop = messageContainer.scrollHeight;
    }
    
    function removeTypingIndicator() {
        const typingIndicator = document.getElementById('typing-indicator');
        if (typingIndicator) {
            typingIndicator.remove();
        }
    }
    
    function openCodeModal() {
        codeModal.style.display = 'flex';
    }
    
    function generateCode() {
        const language = document.getElementById('code-language').value;
        const framework = document.getElementById('code-framework').value;
        const description = document.getElementById('code-description').value.trim();
        
        if (description === '') {
            alert('Please enter a description of the code you need');
            return;
        }
        
        codeModal.style.display = 'none';
        addMessage(`I need code in ${language}${framework !== 'none' ? ' with ' + framework : ''}: ${description}`, 'user');
        
        showTypingIndicator();
        
        setTimeout(() => {
            removeTypingIndicator();
            const response = generateCodeResponse(language, framework, description);
            addMessage(response, 'bot');
        }, 1500 + Math.random() * 2500);
    }
    
    function askAboutHardware() {
        const question = "Can you recommend hardware for my robotics/embedded project?";
        addMessage(question, 'user');
        
        showTypingIndicator();
        
        setTimeout(() => {
            removeTypingIndicator();
            const response = getHardwareRecommendation();
            addMessage(response, 'bot');
        }, 1000 + Math.random() * 2000);
    }
    
    function askAboutROS() {
        const question = "Can you help me with ROS?";
        addMessage(question, 'user');
        
        showTypingIndicator();
        
        setTimeout(() => {
            removeTypingIndicator();
            const response = getROSHelp();
            addMessage(response, 'bot');
        }, 1000 + Math.random() * 2000);
    }
    
    function askAboutROSCommands() {
        const question = "Show me common ROS commands";
        addMessage(question, 'user');
        
        showTypingIndicator();
        
        setTimeout(() => {
            removeTypingIndicator();
            const response = getROSCommands();
            addMessage(response, 'bot');
        }, 1000 + Math.random() * 2000);
    }
    
    function askAboutEmbeddedSnippets() {
        const question = "Show me useful embedded systems code snippets";
        addMessage(question, 'user');
        
        showTypingIndicator();
        
        setTimeout(() => {
            removeTypingIndicator();
            const response = getEmbeddedSnippets();
            addMessage(response, 'bot');
        }, 1000 + Math.random() * 2000);
    }
    
    function clearChat() {
        if (confirm('Are you sure you want to clear the chat?')) {
            messageContainer.innerHTML = '';
            // Add initial bot message back
            addMessage("Hello! I'm your expert assistant for robot programming and embedded systems. How can I help you today?", 'bot');
        }
    }
    
    function loadExample(type) {
        let question = '';
        let response = '';
        
        switch(type) {
            case 'pid':
                question = "How do I implement a PID controller in C for a STM32 microcontroller?";
                response = `Here's a basic PID implementation for STM32 using HAL:

\`\`\`c
#include "stm32f4xx_hal.h"

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float output_limit;
} PID_Controller;

void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd, float limit) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output_limit = limit;
}

float PID_Update(PID_Controller* pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    
    // Proportional term
    float P = pid->Kp * error;
    
    // Integral term with anti-windup
    pid->integral += error * dt;
    if (pid->output_limit != 0) {
        // Clamp the integral term
        pid->integral = (pid->integral > pid->output_limit) ? pid->output_limit : 
                       (pid->integral < -pid->output_limit) ? -pid->output_limit : pid->integral;
    }
    float I = pid->Ki * pid->integral;
    
    // Derivative term
    float D = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // Calculate output
    float output = P + I + D;
    
    // Apply output limits
    if (pid->output_limit != 0) {
        output = (output > pid->output_limit) ? pid->output_limit : 
                (output < -pid->output_limit) ? -pid->output_limit : output;
    }
    
    return output;
}
\`\`\`

To use this PID controller:
1. Initialize it with your gains and output limits
2. Call PID_Update() in your control loop with a consistent time step (dt)
3. Tune the Kp, Ki, Kd parameters for your specific application

For real-time performance:
- Consider using fixed-point arithmetic if floating-point is too slow
- Use a hardware timer for precise loop timing
- For motor control, combine with PWM output`;
                break;
                
            case 'rtos':
                question = "How do I set up tasks in FreeRTOS?";
                response = `Here's how to set up tasks in FreeRTOS:

\`\`\`c
#include "FreeRTOS.h"
#include "task.h"

// Task function prototypes
void vTask1(void *pvParameters);
void vTask2(void *pvParameters);

int main(void) {
    // Create Task 1 with higher priority
    xTaskCreate(vTask1,       // Task function
                "Task1",      // Task name
                128,         // Stack size (words)
                NULL,         // Parameters
                2,           // Priority (higher number = higher priority)
                NULL);       // Task handle
    
    // Create Task 2 with lower priority
    xTaskCreate(vTask2, "Task2", 128, NULL, 1, NULL);
    
    // Start the scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    while(1);
}

void vTask1(void *pvParameters) {
    while(1) {
        // Task 1 code here
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100ms
    }
}

void vTask2(void *pvParameters) {
    while(1) {
        // Task 2 code here
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500ms
    }
}
\`\`\`

Key considerations:
1. Stack size depends on task requirements (watch for stack overflows)
2. Priorities determine execution order (0 = lowest)
3. Use vTaskDelay() instead of busy waits to save power
4. For inter-task communication, use:
   - Queues (xQueueCreate, xQueueSend, xQueueReceive)
   - Semaphores (xSemaphoreCreateBinary, xSemaphoreTake, xSemaphoreGive)
   - Mutexes for resource protection`;
                break;
                
            case 'sensor':
                question = "How can I implement sensor fusion for an IMU?";
                response = `For IMU sensor fusion (combining accelerometer, gyroscope, and optionally magnetometer data), a common approach is the Complementary Filter or Kalman Filter. Here's a basic Complementary Filter implementation:

\`\`\`cpp
#include <cmath> // for atan2, sqrt functions

// Simple complementary filter for pitch and roll
void updateOrientation(float ax, float ay, float az, float gx, float gy, float gz, 
                      float dt, float alpha, float& pitch, float& roll) {
    // Calculate accelerometer angles
    float acc_pitch = atan2(ay, sqrt(ax * ax + az * az));
    float acc_roll = atan2(-ax, sqrt(ay * ay + az * az));
    
    // Integrate gyroscope data
    pitch += gy * dt;
    roll += gx * dt;
    
    // Complementary filter: combine accelerometer and gyro
    pitch = alpha * (pitch) + (1 - alpha) * acc_pitch;
    roll = alpha * (roll) + (1 - alpha) * acc_roll;
}

// Madgwick filter is more advanced (example initialization)
// #include "MadgwickAHRS.h"
// Madgwick filter;
// filter.begin(100); // sample rate in Hz

// In your main loop:
// filter.updateIMU(gx, gy, gz, ax, ay, az);
// float roll = filter.getRoll();
// float pitch = filter.getPitch();
// float yaw = filter.getYaw();
\`\`\`

For better results:
1. Calibrate your IMU (measure bias for gyro when stationary)
2. Use a proper sampling rate (typically 100-200Hz)
3. For the complementary filter, alpha is typically 0.96-0.98
4. Consider more advanced filters like:
   - Madgwick filter (simpler, good for most applications)
   - Mahony filter
   - Kalman filter (more computationally intensive)
   
For ROS, you can use the IMU message and robot_localization package for advanced sensor fusion.`;
                break;
        }
        
        addMessage(question, 'user');
        setTimeout(() => {
            addMessage(response, 'bot');
        }, 500);
    }
    
    // Mock API responses
    function getBotResponse(message) {
        const lowerMessage = message.toLowerCase();
        
        // ROS responses
        if (lowerMessage.includes('ros') || lowerMessage.includes('robot operating system')) {
            if (lowerMessage.includes('node') || lowerMessage.includes('create')) {
                return `To create a ROS node in Python:

\`\`\`python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Received: %s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
\`\`\`

For C++:
\`\`\`cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

void callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, callback);
    ros::spin();
    return 0;
}
\`\`\``;
            } else if (lowerMessage.includes('topic') || lowerMessage.includes('publish')) {
                return `To publish to a ROS topic:

Python:
\`\`\`python
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('chatter', String, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    msg = String()
    msg.data = "Hello ROS"
    pub.publish(msg)
    rate.sleep()
\`\`\`

C++:
\`\`\`cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);
    
    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "Hello ROS";
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
\`\`\``;
            } else {
                return getROSHelp();
            }
        }
        
        // Embedded systems responses
        if (lowerMessage.includes('embedded') || lowerMessage.includes('microcontroller')) {
            if (lowerMessage.includes('stm32') || lowerMessage.includes('arm')) {
                return `For STM32 development:

1. **Toolchain Options**:
   - STM32CubeIDE (official IDE based on Eclipse)
   - PlatformIO with VS Code
   - Keil MDK or IAR EWARM (commercial options)

2. **Key HAL Functions**:
\`\`\`c
// Initialize peripherals
HAL_GPIO_Init();
HAL_UART_Init();
HAL_TIM_Base_Start_IT(); // Timer with interrupt

// Common patterns
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Set pin
HAL_UART_Transmit(&huart2, (uint8_t*)data, length, timeout); // UART send
HAL_ADC_Start(&hadc1); // Start ADC conversion
\`\`\`

3. **Debugging Tips**:
   - Use ST-Link debugger
   - Implement ITM printf for SWO trace
   - Use STM32CubeMonitor for runtime monitoring`;
            } else if (lowerMessage.includes('rtos') || lowerMessage.includes('freeRTOS') || lowerMessage.includes('real-time')) {
                return `Real-Time Operating Systems (RTOS) are crucial for complex embedded systems. Key options:

1. **FreeRTOS** (most popular, open source):
   - Task scheduling with priorities
   - Queues, semaphores, mutexes
   - Memory management options

2. **Zephyr RTOS** (growing in popularity):
   - Modern, modular design
   - Excellent hardware support
   - Built-in networking stack

3. **Key Concepts**:
   - Deterministic timing
   - Priority inversion prevention
   - Resource sharing with mutexes
   - Inter-task communication with queues

Example FreeRTOS task creation:
\`\`\`c
xTaskCreate(myTask, "TaskName", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
\`\`\``;
            } else {
                return `Embedded systems require attention to:
1. **Resource Constraints**: Limited memory, CPU power
2. **Real-time Requirements**: Predictable timing
3. **Power Management**: Especially for battery-powered devices
4. **Hardware Interfaces**: GPIO, ADC, PWM, I2C, SPI, UART

Would you like specific information about:
- Microcontroller selection
- Low-power design
- Real-time operating systems
- Peripheral interfacing?`;
            }
        }
        
        // Robot programming responses
        if (lowerMessage.includes('robot') || lowerMessage.includes('kinematics') || lowerMessage.includes('control')) {
            if (lowerMessage.includes('pid') || lowerMessage.includes('control')) {
                return `PID (Proportional-Integral-Derivative) control is fundamental in robotics. Here's a basic implementation:

\`\`\`python
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        
    def update(self, measurement, dt):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output
\`\`\`

Tuning Tips:
1. Start with Kp only (set Ki and Kd to 0)
2. Increase Kp until the response is fast but starts to oscillate
3. Add Kd to reduce oscillations
4. Add Ki to eliminate steady-state error
5. Consider anti-windup for integral term`;
            } else if (lowerMessage.includes('moveit') || lowerMessage.includes('motion planning')) {
                return `MoveIt is the most widely used motion planning framework in ROS. Key concepts:

1. **Setup**:
\`\`\`xml
<!-- In your URDF -->
<ros_control>
  <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
</ros_control>
\`\`\`

2. **Basic Usage**:
\`\`\`python
from moveit_commander import MoveGroupCommander

group = MoveGroupCommander("arm_group")
group.set_pose_target(pose)  # Set target pose
plan = group.plan()         # Generate plan
group.execute(plan)         # Execute plan
\`\`\`

3. **Key Features**:
   - Inverse kinematics
   - Collision avoidance
   - Trajectory optimization
   - Integration with perception`;
            } else {
                return `Robot programming involves several key areas:
1. **Perception**: Computer vision, LiDAR, sensor fusion
2. **Localization**: SLAM, odometry, GPS
3. **Planning**: Path planning, motion planning
4. **Control**: PID, MPC, force control

Would you like specific information about:
- Robot kinematics/dynamics
- Sensor integration
- Motion planning algorithms
- ROS navigation stack?`;
            }
        }
        
        // Default response
        return `I'm an expert in robot programming and embedded systems. I can help with:

1. **Robot Programming**:
   - ROS/ROS2
   - Motion planning
   - Control algorithms
   - Sensor integration

2. **Embedded Systems**:
   - Microcontroller programming
   - Real-time operating systems
   - Low-level hardware interfacing
   - Power optimization

Please ask a specific question about robotics or embedded systems, or use the quick tools on the left for common tasks.`;
    }
    
    function generateCodeResponse(language, framework, description) {
        let response = `Here's the code you requested in ${language}`;
        if (framework !== 'none') {
            response += ` using ${framework}`;
        }
        response += `:\n\n`;
        
        if (language === 'cpp' && framework === 'ros') {
            response += `\`\`\`cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "${description.toLowerCase().split(' ')[0]}_node");
    ros::NodeHandle nh;
    
    // Create publisher
    ros::Publisher pub = nh.advertise<std_msgs::String>("output_topic", 10);
    
    // Create subscriber
    ros::Subscriber sub = nh.subscribe("input_topic", 10, callbackFunction);
    
    // Main loop
    ros::Rate rate(10); // 10Hz
    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "Hello from ROS node";
        pub.publish(msg);
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}

void callbackFunction(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received: %s", msg->data.c_str());
}
\`\`\`

Remember to:
1. Add dependencies to package.xml
2. Update CMakeLists.txt
3. Build with \`catkin_make\``;
        } else if (language === 'python' && framework === 'ros2') {
            response += `\`\`\`python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ${capitalizeFirstLetter(description.split(' ')[0])}Node(Node):
    def __init__(self):
        super().__init__('${description.toLowerCase().split(' ')[0]}_node')
        self.publisher = self.create_publisher(String, 'output_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.listener_callback,
            10)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from ROS2 node'
        self.publisher.publish(msg)
        
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = ${capitalizeFirstLetter(description.split(' ')[0])}Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
\`\`\`

To run:
1. Add dependencies to package.xml
2. Update setup.py
3. Install with \`colcon build\`
4. Source and run`;
        } else if (language === 'c' && framework === 'stm32hal') {
            response += `\`\`\`c
#include "stm32f4xx_hal.h"

// Peripheral handles
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM3_Init();
    
    // Start PWM on TIM3 Channel 1
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    
    char msg[] = "STM32 HAL running\\r\\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    
    uint16_t duty = 0;
    while (1) {
        // Fade LED
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
        duty = (duty + 10) % 1000;
        HAL_Delay(50);
    }
}

// Generated initialization code would be here...
\`\`\`

Key points:
1. Use STM32CubeMX to generate initialization code
2. HAL provides hardware abstraction
3. Implement HAL_UART_MspInit etc. for low-level setup`;
        } else {
            response += `I've generated code based on your request for ${description}.

\`\`\`${language}
// ${capitalizeFirstLetter(description)}
// TODO: Implement based on your specific requirements

// Basic structure in ${language}
function main() {
    // Initialize system
    init();
    
    // Main loop
    while(true) {
        processInput();
        updateState();
        renderOutput();
    }
}

// Remember to:
// 1. Handle errors appropriately
// 2. Consider real-time constraints
// 3. Optimize for embedded systems if needed
\`\`\`

Would you like me to provide a more specific implementation for your use case?`;
        }
        
        return response;
    }
    
    function getHardwareRecommendation() {
        return `For robotics and embedded systems, hardware selection depends on your requirements:

1. **Microcontrollers**:
   - **Beginner**: Arduino (Uno, Mega) or STM32 (Blue Pill, Black Pill)
   - **Mid-range**: ESP32 (with WiFi/BT), STM32F4 series
   - **High-end**: STM32H7, Teensy 4.1

2. **Single Board Computers**:
   - **Low-power**: Raspberry Pi Pico (RP2040)
   - **General purpose**: Raspberry Pi 4/5
   - **Robotics**: NVIDIA Jetson series (for AI/vision)
   - **Real-time**: BeagleBone with PRUs

3. **Sensors**:
   - **IMU**: MPU6050 (basic), BMI088 (higher quality)
   - **Distance**: HC-SR04 (ultrasonic), VL53L0X (ToF)
   - **Position**: GPS modules (NEO-6M, NEO-M9N)

4. **Motor Control**:
   - **DC Motors**: L298N, TB6612FNG drivers
   - **Steppers**: A4988, DRV8825 drivers
   - **Servos**: PCA9685 PWM controller

What specific application are you working on? I can provide more targeted recommendations.`;
    }
    
    function getROSHelp() {
        return `The Robot Operating System (ROS) is a flexible framework for robotics development. Key concepts:

1. **Core Concepts**:
   - Nodes: Single-purpose processes
   - Topics: Asynchronous pub/sub communication
   - Services: Synchronous request/reply
   - Parameters: Configuration storage

2. **Common Tools**:
   - \`roscore\`: Master process
   - \`rosrun\`: Run a single node
   - \`roslaunch\`: Start multiple nodes
   - \`rviz\`: 3D visualization
   - \`rqt\`: GUI tool suite

3. **ROS2 Improvements**:
   - DDS-based communication
   - Improved real-time support
   - Better cross-platform compatibility
   - Enhanced security features

Would you like specific information about:
- Creating nodes
- Setting up packages
- Using common message types
- Debugging techniques?`;
    }
    
    function getROSCommands() {
        return `Here are essential ROS commands:

1. **Package Management**:
\`\`\`bash
# Create a new package (ROS1)
catkin_create_pkg <pkg_name> [deps]

# Create a new package (ROS2)
ros2 pkg create <pkg_name> --build-type ament_cmake
ros2 pkg create <pkg_name> --build-type ament_python
\`\`\`

2. **Building**:
\`\`\`bash
# ROS1
catkin_make
catkin_make install

# ROS2
colcon build
colcon build --symlink-install
\`\`\`

3. **Node Operations**:
\`\`\`bash
# ROS1
rosrun <pkg_name> <node_name>
roslaunch <pkg_name> <launch_file.launch>

# ROS2
ros2 run <pkg_name> <node_name>
ros2 launch <pkg_name> <launch_file.py>
\`\`\`

4. **Information**:
\`\`\`bash
# ROS1
rostopic list
rosservice list
rosparam list

# ROS2
ros2 topic list
ros2 service list
ros2 param list
\`\`\`

5. **Debugging**:
\`\`\`bash
rosbag record -a
rqt_graph
rosrun rviz rviz
\`\`\``;
    }
    
    function getEmbeddedSnippets() {
        return `Here are useful embedded systems code snippets:

1. **GPIO Toggle (STM32 HAL)**:
\`\`\`c
HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Toggle PA5
HAL_Delay(500);                         // 500ms delay
\`\`\`

2. **UART Communication**:
\`\`\`c
char msg[] = "Hello UART\\r\\n";
HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

uint8_t rx_data[10];
HAL_UART_Receive(&huart2, rx_data, 10, HAL_MAX_DELAY);
\`\`\`

3. **PWM Generation (50% duty)**:
\`\`\`c
htim3.Instance->CCR1 = htim3.Instance->ARR / 2;  // Set 50% duty
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
\`\`\`

4. **ADC Reading**:
\`\`\`c
HAL_ADC_Start(&hadc1);
if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
    uint32_t value = HAL_ADC_GetValue(&hadc1);
}
\`\`\`

5. **Interrupt Handler**:
\`\`\`c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BUTTON_Pin) {
        // Handle button press
    }
}
\`\`\`

6. **RTOS Task**:
\`\`\`c
void vTaskFunction(void *pvParameters) {
    while(1) {
        // Task code
        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms delay
    }
}
\`\`\``;
    }
    
    function capitalizeFirstLetter(string) {
        return string.charAt(0).toUpperCase() + string.slice(1);
    }
    
    // Check for saved theme preference
    if (localStorage.getItem('theme') === 'light') {
        document.body.classList.add('light-theme');
        themeIcon.className = 'fas fa-sun';
    }
});