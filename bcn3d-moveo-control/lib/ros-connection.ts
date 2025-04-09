import ROSLIB from "roslib"

// Define types for our ROS connection
export type RosConnectionStatus = "disconnected" | "connecting" | "connected" | "error"

export interface JointState {
  name: string[]
  position: number[]
  velocity: number[]
  effort: number[]
}

export interface RobotState {
  jointState: JointState
  cartesianPose: {
    position: { x: number; y: number; z: number }
    orientation: { x: number; y: number; z: number; w: number }
  }
  isConnected: boolean
  gripperState: number // 0-100 percentage
}

// Default robot state
export const defaultRobotState: RobotState = {
  jointState: {
    name: ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper"],
    position: [0, 0, 0, 0, 0, 0],
    velocity: [0, 0, 0, 0, 0, 0],
    effort: [0, 0, 0, 0, 0, 0],
  },
  cartesianPose: {
    position: { x: 0, y: 0, z: 0 },
    orientation: { x: 0, y: 0, z: 0, w: 1 },
  },
  isConnected: false,
  gripperState: 0,
}

// ROS Connection class to manage the connection to ROS
export class RosConnection {
  private ros: ROSLIB.Ros | null = null
  private jointStateSubscriber: ROSLIB.Topic | null = null
  private cartesianPoseSubscriber: ROSLIB.Topic | null = null
  private jointCommandPublisher: ROSLIB.Topic | null = null
  private cartesianCommandPublisher: ROSLIB.Topic | null = null
  private gripperCommandPublisher: ROSLIB.Topic | null = null

  private statusListeners: ((status: RosConnectionStatus) => void)[] = []
  private stateListeners: ((state: RobotState) => void)[] = []

  private currentStatus: RosConnectionStatus = "disconnected"
  private currentState: RobotState = { ...defaultRobotState }
  private connectionUrl = ""

  // Connect to ROS
  public connect(url: string): Promise<void> {
    return new Promise((resolve, reject) => {
      if (this.ros) {
        this.disconnect()
      }

      this.connectionUrl = url
      this.updateStatus("connecting")

      this.ros = new ROSLIB.Ros({
        url: this.connectionUrl,
      })

      this.ros.on("connection", () => {
        console.log("Connected to ROS bridge server")
        this.updateStatus("connected")
        this.setupSubscribers()
        this.setupPublishers()
        resolve()
      })

      this.ros.on("error", (error) => {
        console.error("Error connecting to ROS bridge server:", error)
        this.updateStatus("error")
        reject(error)
      })

      this.ros.on("close", () => {
        console.log("Connection to ROS bridge server closed")
        this.updateStatus("disconnected")
      })
    })
  }

  // Disconnect from ROS
  public disconnect(): void {
    if (this.ros) {
      this.ros.close()
      this.ros = null
      this.jointStateSubscriber = null
      this.cartesianPoseSubscriber = null
      this.jointCommandPublisher = null
      this.cartesianCommandPublisher = null
      this.gripperCommandPublisher = null
      this.updateStatus("disconnected")
      this.currentState = { ...defaultRobotState }
      this.notifyStateListeners()
    }
  }

  // Set up subscribers to ROS topics
  private setupSubscribers(): void {
    if (!this.ros) return

    // Subscribe to joint states
    this.jointStateSubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: "/joint_states",
      messageType: "sensor_msgs/JointState",
    })

    this.jointStateSubscriber.subscribe((message: any) => {
      this.currentState.jointState = {
        name: message.name,
        position: message.position,
        velocity: message.velocity,
        effort: message.effort,
      }
      this.currentState.isConnected = true
      this.notifyStateListeners()
    })

    // Subscribe to cartesian pose
    this.cartesianPoseSubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: "/moveo/cartesian_pose",
      messageType: "geometry_msgs/PoseStamped",
    })

    this.cartesianPoseSubscriber.subscribe((message: any) => {
      this.currentState.cartesianPose = {
        position: message.pose.position,
        orientation: message.pose.orientation,
      }
      this.notifyStateListeners()
    })
  }

  // Set up publishers to ROS topics
  private setupPublishers(): void {
    if (!this.ros) return

    // Publisher for joint commands
    this.jointCommandPublisher = new ROSLIB.Topic({
      ros: this.ros,
      name: "/moveo/joint_command",
      messageType: "sensor_msgs/JointState",
    })

    // Publisher for cartesian commands
    this.cartesianCommandPublisher = new ROSLIB.Topic({
      ros: this.ros,
      name: "/moveo/cartesian_command",
      messageType: "geometry_msgs/PoseStamped",
    })

    // Publisher for gripper commands
    this.gripperCommandPublisher = new ROSLIB.Topic({
      ros: this.ros,
      name: "/moveo/gripper_command",
      messageType: "std_msgs/Float32",
    })
  }

  // Send a joint command to the robot
  public sendJointCommand(positions: number[]): void {
    if (!this.jointCommandPublisher) return

    const jointNames = ["joint1", "joint2", "joint3", "joint4", "joint5"]

    // Create the joint command message
    const jointCommand = new ROSLIB.Message({
      header: {
        stamp: {
          sec: 0,
          nanosec: 0,
        },
        frame_id: "",
      },
      name: jointNames,
      position: positions.slice(0, 5), // Only send the first 5 positions (excluding gripper)
      velocity: [],
      effort: [],
    })

    // Publish the joint command
    this.jointCommandPublisher.publish(jointCommand)

    // Update the local state to reflect the command
    // This provides immediate feedback before the robot actually moves
    for (let i = 0; i < Math.min(positions.length, 5); i++) {
      const jointIndex = this.currentState.jointState.name.indexOf(jointNames[i])
      if (jointIndex !== -1) {
        this.currentState.jointState.position[jointIndex] = positions[i]
      }
    }

    this.notifyStateListeners()
  }

  // Send a gripper command to the robot
  public sendGripperCommand(position: number): void {
    if (!this.gripperCommandPublisher) return

    // Create the gripper command message (0-100 percentage)
    const gripperCommand = new ROSLIB.Message({
      data: position,
    })

    // Publish the gripper command
    this.gripperCommandPublisher.publish(gripperCommand)

    // Update the local state
    this.currentState.gripperState = position

    // Also update the joint state if 'gripper' is in the joint names
    const gripperIndex = this.currentState.jointState.name.indexOf("gripper")
    if (gripperIndex !== -1) {
      this.currentState.jointState.position[gripperIndex] = position
    }

    this.notifyStateListeners()
  }

  // Send a cartesian command to the robot
  public sendCartesianCommand(
    position: { x: number; y: number; z: number },
    orientation: { x: number; y: number; z: number; w: number },
  ): void {
    if (!this.cartesianCommandPublisher) return

    // Create the cartesian command message
    const cartesianCommand = new ROSLIB.Message({
      header: {
        stamp: {
          sec: 0,
          nanosec: 0,
        },
        frame_id: "base_link",
      },
      pose: {
        position: position,
        orientation: orientation,
      },
    })

    // Publish the cartesian command
    this.cartesianCommandPublisher.publish(cartesianCommand)

    // Update the local state
    this.currentState.cartesianPose = {
      position,
      orientation,
    }

    this.notifyStateListeners()
  }

  // Send a home command to the robot
  public sendHomeCommand(): void {
    // Home position is typically all joints at 0
    const homePositions = [0, 0, 0, 0, 0]
    this.sendJointCommand(homePositions)
  }

  // Add a listener for connection status changes
  public addStatusListener(listener: (status: RosConnectionStatus) => void): void {
    this.statusListeners.push(listener)
    // Immediately notify the new listener of the current status
    listener(this.currentStatus)
  }

  // Remove a status listener
  public removeStatusListener(listener: (status: RosConnectionStatus) => void): void {
    this.statusListeners = this.statusListeners.filter((l) => l !== listener)
  }

  // Add a listener for robot state changes
  public addStateListener(listener: (state: RobotState) => void): void {
    this.stateListeners.push(listener)
    // Immediately notify the new listener of the current state
    listener(this.currentState)
  }

  // Remove a state listener
  public removeStateListener(listener: (state: RobotState) => void): void {
    this.stateListeners = this.stateListeners.filter((l) => l !== listener)
  }

  // Update the connection status and notify listeners
  private updateStatus(status: RosConnectionStatus): void {
    this.currentStatus = status
    this.notifyStatusListeners()
  }

  // Notify all status listeners of the current status
  private notifyStatusListeners(): void {
    this.statusListeners.forEach((listener) => {
      listener(this.currentStatus)
    })
  }

  // Notify all state listeners of the current robot state
  private notifyStateListeners(): void {
    this.stateListeners.forEach((listener) => {
      listener(this.currentState)
    })
  }

  // Get the current connection status
  public getStatus(): RosConnectionStatus {
    return this.currentStatus
  }

  // Get the current robot state
  public getState(): RobotState {
    return this.currentState
  }
}

// Create a singleton instance of the ROS connection
export const rosConnection = new RosConnection()
