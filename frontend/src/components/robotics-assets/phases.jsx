import PropTypes from "prop-types";

const LineComponent = ({title, description}) => {
    return (
      <div className='flex'>
        <h3 className="font-semibold text-md flex-1 mb-2 gap-2">
          {title}:   
        </h3>
        <p className='mb-2 w-[80%]'>
          {description}
        </p>
      </div>
    );
  };
  
  LineComponent.propTypes = {
    title: PropTypes.string.isRequired,
    description: PropTypes.string.isRequired
  };

export const phases = {
    0: {
      title: "General Introduction to Robotics",
      description: "Learn the basics of robotics and its applications",
      lessons: [
        {
          title: "What is Robotics?",
          content: `
            Robotics is the interdisciplinary branch of engineering and science that includes mechanical engineering, electrical engineering, computer science, and others. Robotics deals with the design, construction, operation, and use of robots, as well as computer systems for their control, sensory feedback, and information processing.
          `
        },
        {
          title: "Applications of Robotics",
          content: (
            <>
            <h3>Robotics has a wide range of applications, including: </h3>
            <ul className='list-inside list-disc pl-12 pb-3'>
              <li>Industrial automation</li>
              <li>Medical robotics</li>
              <li>Autonomous vehicles</li>
              <li>Military applications</li>
              <li>Space exploration</li>
              <li>Home automation</li>
            </ul>
            </>
          )
        },
        {
          title: "Robot Components",
          content: (
            <>
              <LineComponent
                title="Manipulator/rover"
                description="The main body of the robot. It consists of links, joints, and other structural elements of the robot." 
              />
              <LineComponent
                title="Actuators"
                description="The components that provide motion to the robot. Examples include motors, pneumatic cylinders, and hydraulic actuators."
              /> 
              <LineComponent
                title="Sensors"
                description="Devices that provide feedback to the robot about its environment. Examples include cameras, ultrasonic sensors, and encoders."
              />
              <LineComponent
                title="Controller"
                description="The brain of the robot. It processes sensor data and sends commands to the actuators to control the robot's behavior."
              />
              <LineComponent
                title="Power Supply"
                description="Provides electrical power to the robot's components. It can be a battery, power adapter, or other power source."
              />
              <LineComponent
                title="End Effector"
                description="The tool or device attached to the robot's manipulator to perform specific tasks. Examples include grippers, welding torches, and sensors."
              />
            </>
          )
        },
        {
          title: "Robotics Intro in a Video",
          content: (
            <>
              <h3 className='pt-3'> 
                Watch this video to get a general introduction to robotics:
              </h3>
              <div className='flex justify-center pt-6'>
                <iframe width="560" height="315" src="https://www.youtube.com/embed/pwwVOpXrazs?si=bD918wvcbmTpviQh" title="YouTube video player" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerPolicy="strict-origin-when-cross-origin" allowFullScreen></iframe>
              </div>
            </>
          )
        }
      ],
      quiz: [
        {
          question: "What is the main body of a robot called?",
          options: [
            "Actuator",
            "Controller",
            "Manipulator",
            "Sensor"
          ],
          correct: 2
        },
        {
          question: "From the next options, Which is NOT a part of a robot?",
          options: [
            "Power Supply",
            "End Effector",
            "Controller",
            "Spark"
          ],
          correct: 3
        },
        {
          question: "What is the purpose of sensors in a robot?",
          options: [
            "To provide motion to the robot",
            "To process sensor data",
            "To provide feedback about the environment",
            "To control the robot's behavior"
          ],
          correct: 2
        },
        {
          question: "Which of the following is NOT an application of robotics?",
          options: [
            "Industrial automation",
            "Medical robotics",
            "Software development",
            "Agriculture"
          ],
          correct: 2
        },
        {
          question: "What is the main function of the controller in a robot?",
          options: [
            "To provide power to the robot",
            "To send data to the sensors",
            "To move the robot's body",
            "To control the robot's behavior"
          ],
          correct: 3
        }
      ] 
    },
    1: {
      title: "Resources and Tools for Robotics",
      description: "Discover the tools and platforms used in robotics",
      lessons: [
        {
          title: "Introduction to Wokwi",
          content: (
            <>
              <h3>What is Wokwi?</h3>
              <p>
                Wokwi is an online platform for learning and simulating electronics and robotics. With Wokwi, you can:
              </p>
              <ul className='list-inside list-disc pl-8 pb-3'>
                <li>Create and simulate circuits</li>
                <li>Program Arduino and other micro-controllers</li>
                <li>Learn electronics and robotics concepts</li>
                <li>Share your projects with others</li>
              </ul>
              <p>
                Let's start by creating an account on Wokwi and exploring the platform.
              </p>
              <p> Go to this link to create an account: <a
              className='text-blue-500 font-bold pl-3' href="https://wokwi.com/" target="_blank">Wokwi
              </a></p>
            </>
          )
        },
        {
          title: "Getting Started with Arduino IDE",
          content: (
            <>
              <h3>What is Arduino IDE?</h3>
              <p>
                Arduino IDE is the official software for programming Arduino micro-controllers. With Arduino IDE, you can:
              </p>
              <ul className='list-inside list-disc pl-8 pb-3'>
                <li>Write and upload code to Arduino boards</li>
                <li>Use libraries for additional functionality</li>
                <li>Monitor serial output for debugging</li>
                <li>Create projects with Arduino boards</li>
              </ul>
              <p>
                Let's start by downloading and installing Arduino IDE on your computer.
              </p>
              <p>go to this link to download the software: <a
              className='text-blue-500 font-bold pl-3' href="https://www.arduino.cc/en/donate/newsletter" target="_blank">Arduino IDE
              </a></p>

            </>
          )
        },
        {
          title: "(Optional) Robotics Materials and Kits for a better learning experience",
          content: (
            <>
              <h3>Robotics Materials and Kits</h3>
              <p>
                To get hands-on experience with robotics, you can purchase robotics kits and materials. Some popular kits include:
              </p>
              <div className='pl-8 pb-3'>
                  <LineComponent
                    title=" Arduino Uno"
                    description="A popular micro-controller board for beginners. It is easy to use and has a large community."
                  />
                  <LineComponent
                    title="Breaboard"
                    description="A prototyping board for building circuits. It allows you to quickly test and modify circuits."
                  />
                  <LineComponent
                    title="Jumper Wires"
                    description="Wires used to connect components in a circuit. They come in different lengths and colors."
                  />
                  <LineComponent
                    title="LEDs and Resistors"
                    description="Basic components for learning electronics. LEDs emit light, while resistors control current flow."
                  />
                  <LineComponent
                    title="DC Motors"
                    description="Components for building moving robots. Motors provide motion."
                  />
                  <LineComponent
                    title="Motor Drivers"
                    description="Circuits that control the speed and direction of motors. They are essential for robot movement."
                  />
                  <LineComponent
                    title="Ultrasonic Sensors"
                    description="Sensors that measure distance using sound waves. They are perfect for obstacle avoidance."
                  />
                  <LineComponent
                    title="Servo Motors"
                    description="Motors that can rotate to a specific angle. They are ideal for robotic arms and wheels."
                  />
                  <LineComponent
                    title="Power Supply (Two 18650 Batteries) and a Battery Holder"
                    description="Provides electrical power to the robot. It can be a battery pack or power adapter."
                  />
              </div>
              <div>
                <p>Here are two links where you can purchase robotics kits and materials:</p>
                <ul className='list-inside list-disc pl-8 pb-3'>
                  <li>
                    <a className='text-blue-500 font-bold pl-3' href="https://tuni-smart-innovation.com/" target="_blank">Tuni Smart Innovation
                    </a>
                  </li>
                  <li>
                    <a className='text-blue-500 font-bold pl-3' href="https://www.little-son.com/" target="_blank">Little Son
                    </a>
                  </li>
                  <li>
                    <a className='text-blue-500 font-bold pl-3' href="https://2btrading.tn/" target="_blank">2B Trading
                    </a>
                  </li>
                </ul>
              </div>
            </>
          )
        }
      ],
    },
    2: {
      title: "Introduction to Electronics",
      description: "Learn the basics of electronics and circuits",
      lessons: [
        {
          title: "Basic Electronics Components",
          content: (
            <>
              <div>
                <p>Let's start with the fundamental components you'll need in robotics:</p>
                <ul>
                  <li>Resistors - Control current flow</li>
                  <li>Capacitors - Store electrical charge</li>
                  <li>LEDs - Light-emitting diodes</li>
                  <li>Switches - Control circuit completion</li>
                </ul>
                <p>Practice Task: Build a simple LED circuit on Wokwi with a resistor and switch.</p>
              </div>
              <div>

              </div>
            </>
          )

        },
        {
          title: "Understanding Voltage, Current, and Resistance",
          content: `
            Key concepts:
            - Voltage (V): Electrical pressure
            - Current (I): Flow of electricity
            - Resistance (Ω): Opposition to current flow
            
            Ohm's Law: V = I × R
            
            Practice: Calculate current in different circuit scenarios.
          `
        }
      ],
      quiz: [
        {
          question: "What is the purpose of a resistor in an LED circuit?",
          options: [
            "To make the circuit look complex",
            "To limit current and protect the LED",
            "To increase the voltage",
            "To store energy"
          ],
          correct: 1
        },
        {
          question: "Using Ohm's Law, if voltage is 5V and resistance is 250Ω, what is the current?",
          options: [
            "0.02A (20mA)",
            "1.25A",
            "125mA",
            "0.5A"
          ],
          correct: 0
        }
      ]
    },
    3: {
      title: "Introduction to Arduino",
      description: "Getting started with Arduino programming",
      lessons: [
        {
          title: "Arduino Basics and Setup",
          content: `
            Arduino is a micro-controller platform perfect for robotics:
            
            1. Download Arduino IDE
            2. Understanding digital vs analog pins
            3. Basic structure: setup() and loop()
            4. Your first program: Blinking LED
            
            Practice: Create a blinking LED program on Wokwi.
          `
        },
        {
          title: "Digital Input/Output",
          content: `
            Learn to:
            1. Read button states
            2. Control multiple LEDs
            3. Use pinMode() and digitalRead()/digitalWrite()
            
            Project: Create a traffic light system with buttons.
          `
        }
      ],
      quiz: [
        {
          question: "Which function runs repeatedly in an Arduino program?",
          options: [
            "setup()",
            "loop()",
            "main()",
            "repeat()"
          ],
          correct: 1
        },
        {
          question: "What is the correct way to configure a pin as an input?",
          options: [
            "pinMode(pin, INPUT);",
            "setMode(pin, INPUT);",
            "pinInput(pin);",
            "digitalInput(pin);"
          ],
          correct: 0
        }
      ]
    },
    4: {
      title: "Sensors and Actuators",
      description: "Learn about different types of sensors and how to use them",
      lessons: [
        {
          title: "Digital Sensors",
          content: `
            Common digital sensors in robotics:
            
            1. IR Proximity Sensors
            - Detection range: 2-30cm
            - Perfect for line following robots
            - Example code for IR sensor reading:
            
            int irPin = 7;
            void setup() {
              pinMode(irPin, INPUT);
            }
            void loop() {
              if(digitalRead(irPin) == LOW) {
                // Object detected
              }
            }
            
            2. Ultrasonic Distance Sensors (HC-SR04)
            - Range: 2-400cm
            - Uses echo location principle
            - Requires 2 pins: Trigger and Echo
            
            Practice Task: Build a distance-measuring device using HC-SR04 on Wokwi
            `
        },
        {
          title: "Analog Sensors",
          content: `
            Essential analog sensors:
            
            1. Light Dependent Resistors (LDR)
            - Measures ambient light
            - Connect to analog pins
            - Perfect for light-following robots
            
            2. Temperature Sensors (LM35)
            - Accurate temperature readings
            - Linear output
            - 10mV per degree Celsius
            
            Practice Project: Create a temperature-controlled LED system
            - LED changes color based on temperature
            - Use map() function for value conversion
            - Implement threshold-based actions
          `
        }
      ],
      quiz: [
        {
          question: "What is the typical range of an HC-SR04 ultrasonic sensor?",
          options: [
            "0-10cm",
            "2-400cm",
            "1-1000cm",
            "5-50cm"
          ],
          correct: 1
        },
        {
          question: "How much voltage does an LM35 temperature sensor output per degree Celsius?",
          options: [
            "1mV",
            "5mV",
            "10mV",
            "100mV"
          ],
          correct: 2
        }
      ]
    },
    5: {
      title: "Motors and Motion Control",
      description: "Master different types of motors and motion control techniques",
      lessons: [
        {
          title: "DC Motors and Motor Drivers",
          content: `
            Understanding DC Motor Control:
            
            1. Basic DC Motors
            - Operating voltage: Usually 3-12V
            - Requires H-Bridge for direction control
            - Popular driver: L298N
            
            Motor Driver Connection:
            - IN1, IN2: Direction control
            - ENA: Speed control (PWM)
            
            Example Code for Basic Movement:
            
            const int IN1 = 7;
            const int IN2 = 8;
            const int ENA = 9;
            
            void forward(int speed) {
              digitalWrite(IN1, HIGH);
              digitalWrite(IN2, LOW);
              analogWrite(ENA, speed);
            }
            
            Practice: Create a motor control system with variable speed
            `
        },
        {
          title: "Servo Motors and Position Control",
          content: `
            Working with Servo Motors:
            
            1. Standard Servo Motors
            - 0-180 degree rotation
            - Built-in position control
            - Perfect for robotic arms
            
            2. Continuous Rotation Servos
            - Full 360-degree rotation
            - Speed control instead of position
            - Ideal for wheel drive
            
            Advanced Concepts:
            - PWM signal control
            - Position feedback
            - Multiple servo coordination
            
            Project: Build a 2-DOF robotic arm
            - Use two servo motors
            - Implement position control
            - Add button/potentiometer control
          `
        }
      ],
      quiz: [
        {
          question: "What component is necessary to control a DC motor's direction?",
          options: [
            "Resistor",
            "Capacitor",
            "H-Bridge",
            "Transistor"
          ],
          correct: 2
        },
        {
          question: "What is the typical rotation range of a standard servo motor?",
          options: [
            "0-90 degrees",
            "0-180 degrees",
            "0-270 degrees",
            "0-360 degrees"
          ],
          correct: 1
        }
      ]
    },
    6: {
      title: "Advanced Robot Programming",
      description: "Master complex algorithms and robot behavior programming",
      lessons: [
        {
          title: "PID Control Systems",
          content: `
            Understanding PID Control:
            
            1. PID Theory
            - Proportional: Direct proportion to error
            - Integral: Sum of past errors
            - Derivative: Rate of error change
            
            Implementation Example:
            
            class PIDController {
              private:
                float kp, ki, kd;
                float prevError = 0;
                float integral = 0;
                unsigned long lastTime = 0;
              
              public:
                PIDController(float p, float i, float d) : kp(p), ki(i), kd(d) {}
                
                float calculate(float setpoint, float measured) {
                  unsigned long now = millis();
                  float dt = (now - lastTime) / 1000.0;
                  lastTime = now;
                  
                  float error = setpoint - measured;
                  integral += error * dt;
                  float derivative = (error - prevError) / dt;
                  prevError = error;
                  
                  return kp * error + ki * integral + kd * derivative;
                }
            };
            
            Practice Project: 
            - Implement PID control for a line-following robot
            - Tune PID parameters for optimal performance
            - Add telemetry for debugging
            `
        },
        {
          title: "Robot Kinematics and Path Planning",
          content: `
            Advanced Motion Planning:
            
            1. Forward Kinematics
            - Calculate end-effector position
            - DH parameters
            - Transformation matrices
            
            2. Inverse Kinematics
            - Calculate joint angles
            - Multiple solutions handling
            - Singularity avoidance
            
            Example implementation for 2R planar arm:
            
            struct Point2D {
              float x, y;
            };
            
            class RoboticArm {
              private:
                float l1, l2; // Link lengths
                
              public:
                Point2D forwardKinematics(float theta1, float theta2) {
                  Point2D pos;
                  pos.x = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
                  pos.y = l1 * sin(theta1) + l2 * sin(theta1 + theta2);
                  return pos;
                }
                
                bool inverseKinematics(Point2D target, float& theta1, float& theta2) {
                  float d = sqrt(target.x * target.x + target.y * target.y);
                  if (d > (l1 + l2)) return false; // Point unreachable
                  
                  // Calculate angles using cosine law
                  float cos_theta2 = (d*d - l1*l1 - l2*l2) / (2*l1*l2);
                  theta2 = acos(cos_theta2);
                  theta1 = atan2(target.y, target.x) - 
                          atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
                  return true;
                }
            };
            
            Advanced Project:
            - Implement a pick-and-place robot
            - Add obstacle avoidance
            - Create smooth trajectories
            `
        }
      ],
      quiz: [
        {
          question: "Which PID term helps reduce steady-state error?",
          options: [
            "Proportional",
            "Integral",
            "Derivative",
            "None of the above"
          ],
          correct: 1
        },
        {
          question: "What is a singularity in robot kinematics?",
          options: [
            "A software bug",
            "A mechanical failure",
            "A configuration where the robot loses one or more degrees of freedom",
            "A power surge"
          ],
          correct: 2
        }
      ]
    },
    7: {
      title: "Robot Intelligence and Integration",
      description: "Implement computer vision and advanced sensing capabilities",
      lessons: [
        {
          title: "Computer Vision Integration",
          content: `
            Advanced Vision Processing:
            
            1. OpenCV with Arduino
            - Serial communication protocol
            - Image processing pipeline
            - Object detection and tracking
            
            Example Processing Sketch (Python with OpenCV):
            
            import cv2
            import numpy as np
            import serial
            
            class RobotVision:
              def __init__(self, port='/dev/ttyUSB0'):
                self.serial = serial.Serial(port, 115200)
                self.cap = cv2.VideoCapture(0)
                
              def detect_object(self, lower_color, upper_color):
                ret, frame = self.cap.read()
                if not ret: return None
                
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower_color, upper_color)
                
                contours, _ = cv2.findContours(
                  mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                
                if contours:
                  largest = max(contours, key=cv2.contourArea)
                  M = cv2.moments(largest)
                  if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return (cx, cy)
                return None
                
              def send_coordinates(self, coords):
                if coords:
                  cmd = f"X{coords[0]}Y{coords[1]}\\n"
                  self.serial.write(cmd.encode())
            
            Practice Project:
            - Create a color-tracking robot
            - Implement object following behavior
            - Add gesture recognition
            `
        },
        {
          title: "Multi-Sensor Fusion and AI",
          content: `
            Implementing Sensor Fusion:
            
            1. Kalman Filter Implementation
            - State estimation
            - Sensor noise handling
            - Prediction and update steps
            
            Example Kalman Filter:
            
            class KalmanFilter {
              private:
                float Q = 0.1;  // Process noise
                float R = 1.0;  // Measurement noise
                float P = 1.0;  // Estimation error
                float K = 0.0;  // Kalman gain
                float x = 0.0;  // State estimate
                
              public:
                float update(float measurement) {
                  // Prediction
                  P = P + Q;
                  
                  // Update
                  K = P / (P + R);
                  x = x + K * (measurement - x);
                  P = (1 - K) * P;
                  
                  return x;
                }
            };
            
            Final Integration Project:
            - Build an autonomous navigation robot
            - Implement SLAM using multiple sensors
            - Use machine learning for obstacle recognition
            - Create a web interface for monitoring
            
            Requirements:
            1. Multiple sensor types (IMU, encoders, vision)
            2. Real-time sensor fusion
            3. Autonomous decision making
            4. Remote monitoring capabilities
            `
        }
      ],
      quiz: [
        {
          question: "What is the primary purpose of a Kalman filter?",
          options: [
            "Image processing",
            "Motor control",
            "State estimation from noisy measurements",
            "Wireless communication"
          ],
          correct: 2
        },
        {
          question: "Which of these is NOT a common step in computer vision processing?",
          options: [
            "Color space conversion",
            "Voltage regulation",
            "Contour detection",
            "Thresholding"
          ],
          correct: 1
        }
      ]
    }   
  };