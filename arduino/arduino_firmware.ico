#include <Servo.h>
#include <Arduino.h>

// Define servo pins
#define SERVO1_PIN 3
#define SERVO2_PIN 5
#define SERVO3_PIN 6
#define SERVO4_PIN 9
#define SERVO5_PIN 10
#define GRIPPER_PIN 11

// Create servo objects
Servo servo1;  // Base rotation
Servo servo2;  // Shoulder
Servo servo3;  // Elbow
Servo servo4;  // Wrist roll
Servo servo5;  // Wrist pitch
Servo gripper; // Gripper

// Current joint positions in degrees
float joint1Pos = 0.0;
float joint2Pos = 0.0;
float joint3Pos = 0.0;
float joint4Pos = 0.0;
float joint5Pos = 0.0;
float gripperPos = 0.0;

// Target joint positions
float joint1Target = 0.0;
float joint2Target = 0.0;
float joint3Target = 0.0;
float joint4Target = 0.0;
float joint5Target = 0.0;
float gripperTarget = 0.0;

// Movement speed (degrees per update)
float jointSpeed = 2.0;

// Time tracking
unsigned long lastUpdate = 0;
const int updateInterval = 20; // 50Hz update rate

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Attach servos to pins
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);
  gripper.attach(GRIPPER_PIN);
  
  // Move servos to initial position
  updateServos();
  
  Serial.println("BCN3D Moveo Arduino controller initialized");
}

void loop() {
  // Check for commands from ROS
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
  }
  
  // Update servo positions at regular intervals
  unsigned long currentTime = millis();
  if (currentTime - lastUpdate >= updateInterval) {
    updateJointPositions();
    updateServos();
    sendJointStates();
    lastUpdate = currentTime;
  }
}

void parseCommand(String command) {
  if (command.startsWith("J:")) {
    // Joint command format: J:angle1,angle2,angle3,angle4,angle5
    command.remove(0, 2); // Remove "J:"
    
    int commaIndex = command.indexOf(',');
    if (commaIndex != -1) {
      joint1Target = command.substring(0, commaIndex).toFloat();
      command.remove(0, commaIndex + 1);
      
      commaIndex = command.indexOf(',');
      if (commaIndex != -1) {
        joint2Target = command.substring(0, commaIndex).toFloat();
        command.remove(0, commaIndex + 1);
        
        commaIndex = command.indexOf(',');
        if (commaIndex != -1) {
          joint3Target = command.substring(0, commaIndex).toFloat();
          command.remove(0, commaIndex + 1);
          
          commaIndex = command.indexOf(',');
          if (commaIndex != -1) {
            joint4Target = command.substring(0, commaIndex).toFloat();
            joint5Target = command.substring(commaIndex + 1).toFloat();
          }
        }
      }
    }
    
    Serial.print("Received joint command: ");
    Serial.print(joint1Target);
    Serial.print(", ");
    Serial.print(joint2Target);
    Serial.print(", ");
    Serial.print(joint3Target);
    Serial.print(", ");
    Serial.print(joint4Target);
    Serial.print(", ");
    Serial.println(joint5Target);
  }
  else if (command.startsWith("G:")) {
    // Gripper command format: G:position (0-100)
    command.remove(0, 2); // Remove "G:"
    gripperTarget = command.toFloat();
    
    Serial.print("Received gripper command: ");
    Serial.println(gripperTarget);
  }
}

void updateJointPositions() {
  // Gradually move joints toward target positions
  joint1Pos = moveToward(joint1Pos, joint1Target, jointSpeed);
  joint2Pos = moveToward(joint2Pos, joint2Target, jointSpeed);
  joint3Pos = moveToward(joint3Pos, joint3Target, jointSpeed);
  joint4Pos = moveToward(joint4Pos, joint4Target, jointSpeed);
  joint5Pos = moveToward(joint5Pos, joint5Target, jointSpeed);
  gripperPos = moveToward(gripperPos, gripperTarget, jointSpeed);
}

float moveToward(float current, float target, float step) {
  if (current < target) {
    return min(current + step, target);
  } else if (current > target) {
    return max(current - step, target);
  }
  return current;
}

void updateServos() {
  // Convert joint angles to servo positions (0-180)
  // Note: You may need to adjust these mappings based on your servo configuration
  
  // Map joint1 (-180 to 180 degrees) to servo range (0-180)
  int servo1Pos = map(constrain(joint1Pos, -180, 180), -180, 180, 0, 180);
  
  // Map joint2 (-90 to 90 degrees) to servo range (0-180)
  int servo2Pos = map(constrain(joint2Pos, -90, 90), -90, 90, 0, 180);
  
  // Map joint3 (-180 to 180 degrees) to servo range (0-180)
  int servo3Pos = map(constrain(joint3Pos, -180, 180), -180, 180, 0, 180);
  
  // Map joint4 (-180 to 180 degrees) to servo range (0-180)
  int servo4Pos = map(constrain(joint4Pos, -180, 180), -180, 180, 0, 180);
  
  // Map joint5 (-90 to 90 degrees) to servo range (0-180)
  int servo5Pos = map(constrain(joint5Pos, -90, 90), -90, 90, 0, 180);
  
  // Map gripper (0 to 100 percent) to servo range (0-180)
  int gripperServoPos = map(constrain(gripperPos, 0, 100), 0, 100, 0, 180);
  
  // Write positions to servos
  servo1.write(servo1Pos);
  servo2.write(servo2Pos);
  servo3.write(servo3Pos);
  servo4.write(servo4Pos);
  servo5.write(servo5Pos);
  gripper.write(gripperServoPos);
}

void sendJointStates() {
  // Send current joint positions back to ROS
  Serial.print("JOINTS:");
  Serial.print(joint1Pos);
  Serial.print(",");
  Serial.print(joint2Pos);
  Serial.print(",");
  Serial.print(joint3Pos);
  Serial.print(",");
  Serial.print(joint4Pos);
  Serial.print(",");
  Serial.print(joint5Pos);
  Serial.print(",");
  Serial.println(gripperPos);
}
