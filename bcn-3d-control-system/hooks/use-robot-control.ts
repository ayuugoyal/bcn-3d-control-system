"use client"

import { useState, useEffect } from "react"
import { useRos } from "@/contexts/ros-context"
import { RobotControl, type JointPositions, type CartesianPosition } from "@/lib/robot-control"

// Hook for controlling the robot
export function useRobotControl() {
  const { status, robotState } = useRos()
  const [jointPositions, setJointPositions] = useState<JointPositions>({
    joint1: 0,
    joint2: 0,
    joint3: 0,
    joint4: 0,
    joint5: 0,
    gripper: 0,
  })

  const [cartesianPosition, setCartesianPosition] = useState<CartesianPosition>({
    x: 0,
    y: 0,
    z: 0,
    roll: 0,
    pitch: 0,
    yaw: 0,
  })

  // Update joint positions when robot state changes
  useEffect(() => {
    if (status === "connected") {
      const currentJointPositions = RobotControl.getCurrentJointPositions()
      setJointPositions(currentJointPositions)

      const currentCartesianPosition = RobotControl.getCurrentCartesianPosition()
      setCartesianPosition(currentCartesianPosition)
    }
  }, [robotState, status])

  // Send joint positions to the robot
  const sendJointPositions = (positions: JointPositions) => {
    if (status === "connected") {
      RobotControl.sendJointPositions(positions)
      setJointPositions(positions)
    }
  }

  // Send cartesian position to the robot
  const sendCartesianPosition = (position: CartesianPosition) => {
    if (status === "connected") {
      RobotControl.sendCartesianPosition(position)
      setCartesianPosition(position)
    }
  }

  // Update a single joint position
  const updateJointPosition = (joint: keyof JointPositions, value: number) => {
    const newPositions = { ...jointPositions, [joint]: value }
    setJointPositions(newPositions)
    sendJointPositions(newPositions)
  }

  // Update a single cartesian position component
  const updateCartesianPosition = (component: keyof CartesianPosition, value: number) => {
    const newPosition = { ...cartesianPosition, [component]: value }
    setCartesianPosition(newPosition)
    sendCartesianPosition(newPosition)
  }

  // Send home command to the robot
  const sendHomeCommand = () => {
    if (status === "connected") {
      RobotControl.sendHomeCommand()
    }
  }

  // Reset all joint positions to zero
  const resetJointPositions = () => {
    const zeroPositions: JointPositions = {
      joint1: 0,
      joint2: 0,
      joint3: 0,
      joint4: 0,
      joint5: 0,
      gripper: 0,
    }
    sendJointPositions(zeroPositions)
  }

  return {
    isConnected: status === "connected",
    jointPositions,
    cartesianPosition,
    sendJointPositions,
    sendCartesianPosition,
    updateJointPosition,
    updateCartesianPosition,
    sendHomeCommand,
    resetJointPositions,
  }
}
