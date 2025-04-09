import { rosConnection } from "./ros-connection"

// Define types for robot control
export interface JointPositions {
  joint1: number
  joint2: number
  joint3: number
  joint4: number
  joint5: number
  gripper: number
}

export interface CartesianPosition {
  x: number
  y: number
  z: number
  roll: number
  pitch: number
  yaw: number
}

// Convert Euler angles (roll, pitch, yaw) to quaternion
function eulerToQuaternion(roll: number, pitch: number, yaw: number) {
  // Convert degrees to radians
  const rollRad = (roll * Math.PI) / 180
  const pitchRad = (pitch * Math.PI) / 180
  const yawRad = (yaw * Math.PI) / 180

  const cy = Math.cos(yawRad * 0.5)
  const sy = Math.sin(yawRad * 0.5)
  const cp = Math.cos(pitchRad * 0.5)
  const sp = Math.sin(pitchRad * 0.5)
  const cr = Math.cos(rollRad * 0.5)
  const sr = Math.sin(rollRad * 0.5)

  const w = cy * cp * cr + sy * sp * sr
  const x = cy * cp * sr - sy * sp * cr
  const y = sy * cp * sr + cy * sp * cr
  const z = sy * cp * cr - cy * sp * sr

  return { x, y, z, w }
}

// Convert quaternion to Euler angles (roll, pitch, yaw)
function quaternionToEuler(x: number, y: number, z: number, w: number) {
  // Roll (x-axis rotation)
  const sinr_cosp = 2 * (w * x + y * z)
  const cosr_cosp = 1 - 2 * (x * x + y * y)
  const roll = Math.atan2(sinr_cosp, cosr_cosp)

  // Pitch (y-axis rotation)
  const sinp = 2 * (w * y - z * x)
  let pitch
  if (Math.abs(sinp) >= 1) {
    pitch = (Math.sign(sinp) * Math.PI) / 2 // Use 90 degrees if out of range
  } else {
    pitch = Math.asin(sinp)
  }

  // Yaw (z-axis rotation)
  const siny_cosp = 2 * (w * z + x * y)
  const cosy_cosp = 1 - 2 * (y * y + z * z)
  const yaw = Math.atan2(siny_cosp, cosy_cosp)

  // Convert radians to degrees
  return {
    roll: (roll * 180) / Math.PI,
    pitch: (pitch * 180) / Math.PI,
    yaw: (yaw * 180) / Math.PI,
  }
}

// Robot control class
export class RobotControl {
  // Send joint positions to the robot
  public static sendJointPositions(positions: JointPositions): void {
    const posArray = [positions.joint1, positions.joint2, positions.joint3, positions.joint4, positions.joint5]

    // Send joint positions to ROS
    rosConnection.sendJointCommand(posArray)

    // Send gripper position separately
    rosConnection.sendGripperCommand(positions.gripper)
  }

  // Send cartesian position to the robot
  public static sendCartesianPosition(position: CartesianPosition): void {
    // Convert Euler angles to quaternion
    const quaternion = eulerToQuaternion(position.roll, position.pitch, position.yaw)

    // Send cartesian position to ROS
    rosConnection.sendCartesianCommand(
      {
        x: position.x / 1000, // Convert from mm to meters
        y: position.y / 1000,
        z: position.z / 1000,
      },
      quaternion,
    )
  }

  // Send home command to the robot
  public static sendHomeCommand(): void {
    rosConnection.sendHomeCommand()
  }

  // Get current joint positions
  public static getCurrentJointPositions(): JointPositions {
    const state = rosConnection.getState()
    const jointState = state.jointState

    // Default joint positions
    const positions: JointPositions = {
      joint1: 0,
      joint2: 0,
      joint3: 0,
      joint4: 0,
      joint5: 0,
      gripper: state.gripperState,
    }

    // Map joint state to joint positions
    jointState.name.forEach((name, index) => {
      if (name === "joint1") positions.joint1 = jointState.position[index]
      if (name === "joint2") positions.joint2 = jointState.position[index]
      if (name === "joint3") positions.joint3 = jointState.position[index]
      if (name === "joint4") positions.joint4 = jointState.position[index]
      if (name === "joint5") positions.joint5 = jointState.position[index]
      if (name === "gripper") positions.gripper = jointState.position[index]
    })

    return positions
  }

  // Get current cartesian position
  public static getCurrentCartesianPosition(): CartesianPosition {
    const state = rosConnection.getState()
    const cartesianPose = state.cartesianPose

    // Convert quaternion to Euler angles
    const euler = quaternionToEuler(
      cartesianPose.orientation.x,
      cartesianPose.orientation.y,
      cartesianPose.orientation.z,
      cartesianPose.orientation.w,
    )

    return {
      x: cartesianPose.position.x * 1000, // Convert from meters to mm
      y: cartesianPose.position.y * 1000,
      z: cartesianPose.position.z * 1000,
      roll: euler.roll,
      pitch: euler.pitch,
      yaw: euler.yaw,
    }
  }

  // Check if robot is connected
  public static isConnected(): boolean {
    return rosConnection.getStatus() === "connected"
  }
}
