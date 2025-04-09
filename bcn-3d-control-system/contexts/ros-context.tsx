"use client"

import { createContext, useContext, useEffect, useState, type ReactNode } from "react"
import { rosConnection, type RosConnectionStatus, type RobotState, defaultRobotState } from "@/lib/ros-connection"

// Define the context type
interface RosContextType {
  status: RosConnectionStatus
  robotState: RobotState
  connect: (url: string) => Promise<void>
  disconnect: () => void
  statusMessage: string
}

// Create the context with default values
const RosContext = createContext<RosContextType>({
  status: "disconnected",
  robotState: defaultRobotState,
  connect: () => Promise.resolve(),
  disconnect: () => {},
  statusMessage: "Disconnected",
})

// Hook to use the ROS context
export const useRos = () => useContext(RosContext)

// Provider component
export const RosProvider = ({ children }: { children: ReactNode }) => {
  const [status, setStatus] = useState<RosConnectionStatus>("disconnected")
  const [robotState, setRobotState] = useState<RobotState>(defaultRobotState)
  const [statusMessage, setStatusMessage] = useState<string>("Disconnected")

  // Set up listeners when the component mounts
  useEffect(() => {
    // Add status listener
    const handleStatusChange = (newStatus: RosConnectionStatus) => {
      setStatus(newStatus)

      // Update status message based on connection status
      switch (newStatus) {
        case "connected":
          setStatusMessage("Connected to ROS2 on Raspberry Pi")
          break
        case "connecting":
          setStatusMessage("Connecting to ROS2...")
          break
        case "disconnected":
          setStatusMessage("Disconnected from ROS2")
          break
        case "error":
          setStatusMessage("Error connecting to ROS2")
          break
      }
    }

    // Add state listener
    const handleStateChange = (newState: RobotState) => {
      setRobotState(newState)
    }

    rosConnection.addStatusListener(handleStatusChange)
    rosConnection.addStateListener(handleStateChange)

    // Clean up listeners when the component unmounts
    return () => {
      rosConnection.removeStatusListener(handleStatusChange)
      rosConnection.removeStateListener(handleStateChange)
    }
  }, [])

  // Connect to ROS
  const connect = async (url: string) => {
    try {
      await rosConnection.connect(url)
    } catch (error) {
      console.error("Failed to connect to ROS:", error)
      setStatusMessage(`Connection error: ${error instanceof Error ? error.message : String(error)}`)
    }
  }

  // Disconnect from ROS
  const disconnect = () => {
    rosConnection.disconnect()
  }

  // Context value
  const value = {
    status,
    robotState,
    connect,
    disconnect,
    statusMessage,
  }

  return <RosContext.Provider value={value}>{children}</RosContext.Provider>
}
