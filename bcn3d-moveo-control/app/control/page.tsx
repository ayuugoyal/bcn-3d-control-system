"use client"

import { useState } from "react"
import Link from "next/link"
import { Slider } from "@/components/ui/slider"
import { Button } from "@/components/ui/button"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { ArrowLeft, Save, Play, Pause, RotateCcw, Home, Zap, AlertTriangle } from "lucide-react"
import { motion } from "framer-motion"
import { useRos } from "@/contexts/ros-context"
import { useRobotControl } from "@/hooks/use-robot-control"
import { RobotVisualization } from "@/components/robot-visualization"

export default function ControlPanel() {
  const [activeTab, setActiveTab] = useState("joint-control")
  const [rosEndpoint, setRosEndpoint] = useState("ws://raspberry-pi-ip:9090")
  const { status, connect, disconnect, statusMessage } = useRos()
  const {
    jointPositions,
    cartesianPosition,
    updateJointPosition,
    updateCartesianPosition,
    sendHomeCommand,
    resetJointPositions,
  } = useRobotControl()

  // Handle joint position change
  const handleJointChange = (joint, value) => {
    updateJointPosition(joint, value[0])
  }

  // Handle cartesian position change
  const handleCartesianChange = (component, value) => {
    updateCartesianPosition(component, value[0])
  }

  // Reset all joints to zero position
  const handleReset = () => {
    resetJointPositions()
  }

  // Home all joints
  const handleHome = () => {
    sendHomeCommand()
  }

  // Connect to ROS
  const handleConnect = async () => {
    try {
      await connect(rosEndpoint)
    } catch (error) {
      console.error("Failed to connect to ROS:", error)
    }
  }

  // Disconnect from ROS
  const handleDisconnect = () => {
    disconnect()
  }

  return (
    <div className="flex flex-col min-h-screen bg-gradient-to-br from-slate-900 to-slate-800 text-white">
      <header className="border-b border-slate-700 bg-slate-900/80 backdrop-blur-sm sticky top-0 z-10">
        <div className="container mx-auto px-4 py-4 flex justify-between items-center">
          <div className="flex items-center space-x-2">
            <Link href="/" className="flex items-center space-x-2 text-slate-300 hover:text-cyan-400 transition-colors">
              <ArrowLeft size={20} />
              <span>Back to Home</span>
            </Link>
          </div>
          <h1 className="text-xl font-bold bg-gradient-to-r from-cyan-400 to-purple-500 text-transparent bg-clip-text">
            Control Panel
          </h1>
          <div className="flex items-center">
            <motion.span
              className={`inline-block w-3 h-3 rounded-full mr-2 ${status === "connected" ? "bg-emerald-500" : "bg-red-500"}`}
              animate={{ scale: status === "connected" ? [1, 1.2, 1] : 1 }}
              transition={{
                duration: 1,
                repeat: status === "connected" ? Number.POSITIVE_INFINITY : 0,
                repeatDelay: 1,
              }}
            ></motion.span>
            <span className="text-sm text-slate-300">{statusMessage}</span>
          </div>
        </div>
      </header>

      <main className="flex-grow container mx-auto px-4 py-8">
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          <div className="lg:col-span-2">
            <Tabs defaultValue="joint-control" value={activeTab} onValueChange={setActiveTab} className="w-full">
              <TabsList className="grid w-full grid-cols-3 bg-slate-800 border border-slate-700">
                <TabsTrigger
                  value="joint-control"
                  className="data-[state=active]:bg-gradient-to-r data-[state=active]:from-cyan-500 data-[state=active]:to-cyan-600 data-[state=active]:text-white"
                >
                  Joint Control
                </TabsTrigger>
                <TabsTrigger
                  value="cartesian-control"
                  className="data-[state=active]:bg-gradient-to-r data-[state=active]:from-purple-500 data-[state=active]:to-purple-600 data-[state=active]:text-white"
                >
                  Cartesian Control
                </TabsTrigger>
                <TabsTrigger
                  value="programs"
                  className="data-[state=active]:bg-gradient-to-r data-[state=active]:from-teal-500 data-[state=active]:to-teal-600 data-[state=active]:text-white"
                >
                  Saved Programs
                </TabsTrigger>
              </TabsList>

              <TabsContent value="joint-control" className="mt-4">
                <Card className="bg-slate-800/50 border-slate-700 backdrop-blur-sm">
                  <CardHeader className="pb-2">
                    <CardTitle className="text-cyan-400 flex items-center">
                      <Zap className="mr-2 h-5 w-5 text-cyan-400" />
                      Joint Control
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-6">
                    <div className="space-y-4">
                      <div className="flex items-center space-x-4">
                        <span className="w-24 text-slate-300">Base</span>
                        <Slider
                          value={[jointPositions.joint1]}
                          min={-180}
                          max={180}
                          step={1}
                          onValueChange={(value) => handleJointChange("joint1", value)}
                          className="flex-grow"
                          disabled={status !== "connected"}
                        />
                        <span className="w-16 text-right font-mono bg-slate-900 px-2 py-1 rounded text-cyan-400">
                          {jointPositions.joint1}°
                        </span>
                      </div>

                      <div className="flex items-center space-x-4">
                        <span className="w-24 text-slate-300">Shoulder</span>
                        <Slider
                          value={[jointPositions.joint2]}
                          min={-90}
                          max={90}
                          step={1}
                          onValueChange={(value) => handleJointChange("joint2", value)}
                          className="flex-grow"
                          disabled={status !== "connected"}
                        />
                        <span className="w-16 text-right font-mono bg-slate-900 px-2 py-1 rounded text-cyan-400">
                          {jointPositions.joint2}°
                        </span>
                      </div>

                      <div className="flex items-center space-x-4">
                        <span className="w-24 text-slate-300">Elbow</span>
                        <Slider
                          value={[jointPositions.joint3]}
                          min={-180}
                          max={180}
                          step={1}
                          onValueChange={(value) => handleJointChange("joint3", value)}
                          className="flex-grow"
                          disabled={status !== "connected"}
                        />
                        <span className="w-16 text-right font-mono bg-slate-900 px-2 py-1 rounded text-cyan-400">
                          {jointPositions.joint3}°
                        </span>
                      </div>

                      <div className="flex items-center space-x-4">
                        <span className="w-24 text-slate-300">Wrist Roll</span>
                        <Slider
                          value={[jointPositions.joint4]}
                          min={-180}
                          max={180}
                          step={1}
                          onValueChange={(value) => handleJointChange("joint4", value)}
                          className="flex-grow"
                          disabled={status !== "connected"}
                        />
                        <span className="w-16 text-right font-mono bg-slate-900 px-2 py-1 rounded text-cyan-400">
                          {jointPositions.joint4}°
                        </span>
                      </div>

                      <div className="flex items-center space-x-4">
                        <span className="w-24 text-slate-300">Wrist Pitch</span>
                        <Slider
                          value={[jointPositions.joint5]}
                          min={-90}
                          max={90}
                          step={1}
                          onValueChange={(value) => handleJointChange("joint5", value)}
                          className="flex-grow"
                          disabled={status !== "connected"}
                        />
                        <span className="w-16 text-right font-mono bg-slate-900 px-2 py-1 rounded text-cyan-400">
                          {jointPositions.joint5}°
                        </span>
                      </div>

                      <div className="flex items-center space-x-4">
                        <span className="w-24 text-slate-300">Gripper</span>
                        <Slider
                          value={[jointPositions.gripper]}
                          min={0}
                          max={100}
                          step={1}
                          onValueChange={(value) => handleJointChange("gripper", value)}
                          className="flex-grow"
                          disabled={status !== "connected"}
                        />
                        <span className="w-16 text-right font-mono bg-slate-900 px-2 py-1 rounded text-cyan-400">
                          {jointPositions.gripper}%
                        </span>
                      </div>
                    </div>

                    <div className="flex space-x-4 pt-4">
                      <Button
                        onClick={handleHome}
                        className="flex items-center space-x-2 bg-gradient-to-r from-cyan-500 to-cyan-600 hover:from-cyan-400 hover:to-cyan-500 border-0"
                        disabled={status !== "connected"}
                      >
                        <Home size={16} />
                        <span>Home</span>
                      </Button>
                      <Button
                        onClick={handleReset}
                        variant="outline"
                        className="flex items-center space-x-2 border-cyan-500 text-cyan-400 hover:bg-cyan-500/10"
                        disabled={status !== "connected"}
                      >
                        <RotateCcw size={16} />
                        <span>Reset</span>
                      </Button>
                      <Button
                        className="flex items-center space-x-2 ml-auto bg-gradient-to-r from-purple-500 to-purple-600 hover:from-purple-400 hover:to-purple-500 border-0"
                        disabled={status !== "connected"}
                      >
                        <Save size={16} />
                        <span>Save Position</span>
                      </Button>
                    </div>
                  </CardContent>
                </Card>
              </TabsContent>

              <TabsContent value="cartesian-control" className="mt-4">
                <Card className="bg-slate-800/50 border-slate-700 backdrop-blur-sm">
                  <CardHeader className="pb-2">
                    <CardTitle className="text-purple-400 flex items-center">
                      <Zap className="mr-2 h-5 w-5 text-purple-400" />
                      Cartesian Control
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="grid grid-cols-2 gap-6">
                      <div className="space-y-4">
                        <div className="flex items-center space-x-4">
                          <span className="w-12 text-slate-300">X</span>
                          <Slider
                            value={[cartesianPosition.x]}
                            min={-300}
                            max={300}
                            step={1}
                            className="flex-grow"
                            onValueChange={(value) => handleCartesianChange("x", value[0])}
                            disabled={status !== "connected"}
                          />
                          <span className="w-20 text-right font-mono bg-slate-900 px-2 py-1 rounded text-purple-400">
                            {cartesianPosition.x} mm
                          </span>
                        </div>

                        <div className="flex items-center space-x-4">
                          <span className="w-12 text-slate-300">Y</span>
                          <Slider
                            value={[cartesianPosition.y]}
                            min={-300}
                            max={300}
                            step={1}
                            className="flex-grow"
                            onValueChange={(value) => handleCartesianChange("y", value[0])}
                            disabled={status !== "connected"}
                          />
                          <span className="w-20 text-right font-mono bg-slate-900 px-2 py-1 rounded text-purple-400">
                            {cartesianPosition.y} mm
                          </span>
                        </div>

                        <div className="flex items-center space-x-4">
                          <span className="w-12 text-slate-300">Z</span>
                          <Slider
                            value={[cartesianPosition.z]}
                            min={0}
                            max={400}
                            step={1}
                            className="flex-grow"
                            onValueChange={(value) => handleCartesianChange("z", value[0])}
                            disabled={status !== "connected"}
                          />
                          <span className="w-20 text-right font-mono bg-slate-900 px-2 py-1 rounded text-purple-400">
                            {cartesianPosition.z} mm
                          </span>
                        </div>
                      </div>

                      <div className="space-y-4">
                        <div className="flex items-center space-x-4">
                          <span className="w-12 text-slate-300">Roll</span>
                          <Slider
                            value={[cartesianPosition.roll]}
                            min={-180}
                            max={180}
                            step={1}
                            className="flex-grow"
                            onValueChange={(value) => handleCartesianChange("roll", value[0])}
                            disabled={status !== "connected"}
                          />
                          <span className="w-20 text-right font-mono bg-slate-900 px-2 py-1 rounded text-purple-400">
                            {cartesianPosition.roll}°
                          </span>
                        </div>

                        <div className="flex items-center space-x-4">
                          <span className="w-12 text-slate-300">Pitch</span>
                          <Slider
                            value={[cartesianPosition.pitch]}
                            min={-90}
                            max={90}
                            step={1}
                            className="flex-grow"
                            onValueChange={(value) => handleCartesianChange("pitch", value[0])}
                            disabled={status !== "connected"}
                          />
                          <span className="w-20 text-right font-mono bg-slate-900 px-2 py-1 rounded text-purple-400">
                            {cartesianPosition.pitch}°
                          </span>
                        </div>

                        <div className="flex items-center space-x-4">
                          <span className="w-12 text-slate-300">Yaw</span>
                          <Slider
                            value={[cartesianPosition.yaw]}
                            min={-180}
                            max={180}
                            step={1}
                            className="flex-grow"
                            onValueChange={(value) => handleCartesianChange("yaw", value[0])}
                            disabled={status !== "connected"}
                          />
                          <span className="w-20 text-right font-mono bg-slate-900 px-2 py-1 rounded text-purple-400">
                            {cartesianPosition.yaw}°
                          </span>
                        </div>
                      </div>
                    </div>

                    <div className="flex space-x-4 pt-6">
                      <Button
                        className="flex items-center space-x-2 bg-gradient-to-r from-purple-500 to-purple-600 hover:from-purple-400 hover:to-purple-500 border-0"
                        disabled={status !== "connected"}
                      >
                        <Play size={16} />
                        <span>Move to Position</span>
                      </Button>
                      <Button
                        variant="outline"
                        className="flex items-center space-x-2 ml-auto border-purple-500 text-purple-400 hover:bg-purple-500/10"
                        disabled={status !== "connected"}
                      >
                        <Save size={16} />
                        <span>Save Position</span>
                      </Button>
                    </div>
                  </CardContent>
                </Card>
              </TabsContent>

              <TabsContent value="programs" className="mt-4">
                <Card className="bg-slate-800/50 border-slate-700 backdrop-blur-sm">
                  <CardHeader className="pb-2">
                    <CardTitle className="text-teal-400 flex items-center">
                      <Zap className="mr-2 h-5 w-5 text-teal-400" />
                      Saved Programs
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-4">
                      <motion.div
                        className="bg-slate-900/80 p-4 rounded-lg border border-slate-700 hover:border-teal-500/50 transition-colors"
                        whileHover={{ scale: 1.01 }}
                        transition={{ type: "spring", stiffness: 400, damping: 10 }}
                      >
                        <div className="flex justify-between items-center mb-2">
                          <h3 className="font-medium text-teal-400">Pick and Place Demo</h3>
                          <div className="flex space-x-2">
                            <Button
                              size="sm"
                              variant="outline"
                              className="h-8 w-8 p-0 border-teal-500 text-teal-400 hover:bg-teal-500/10"
                              disabled={status !== "connected"}
                            >
                              <Play size={16} />
                            </Button>
                            <Button
                              size="sm"
                              variant="outline"
                              className="h-8 w-8 p-0 border-teal-500 text-teal-400 hover:bg-teal-500/10"
                              disabled={status !== "connected"}
                            >
                              <Pause size={16} />
                            </Button>
                          </div>
                        </div>
                        <p className="text-sm text-slate-400">Simple pick and place sequence with 5 waypoints</p>
                      </motion.div>

                      <motion.div
                        className="bg-slate-900/80 p-4 rounded-lg border border-slate-700 hover:border-teal-500/50 transition-colors"
                        whileHover={{ scale: 1.01 }}
                        transition={{ type: "spring", stiffness: 400, damping: 10 }}
                      >
                        <div className="flex justify-between items-center mb-2">
                          <h3 className="font-medium text-teal-400">Drawing Circle</h3>
                          <div className="flex space-x-2">
                            <Button
                              size="sm"
                              variant="outline"
                              className="h-8 w-8 p-0 border-teal-500 text-teal-400 hover:bg-teal-500/10"
                              disabled={status !== "connected"}
                            >
                              <Play size={16} />
                            </Button>
                            <Button
                              size="sm"
                              variant="outline"
                              className="h-8 w-8 p-0 border-teal-500 text-teal-400 hover:bg-teal-500/10"
                              disabled={status !== "connected"}
                            >
                              <Pause size={16} />
                            </Button>
                          </div>
                        </div>
                        <p className="text-sm text-slate-400">Draws a circle on a flat surface</p>
                      </motion.div>

                      <motion.div
                        className="bg-slate-900/80 p-4 rounded-lg border border-slate-700 hover:border-teal-500/50 transition-colors"
                        whileHover={{ scale: 1.01 }}
                        transition={{ type: "spring", stiffness: 400, damping: 10 }}
                      >
                        <div className="flex justify-between items-center mb-2">
                          <h3 className="font-medium text-teal-400">Wave Sequence</h3>
                          <div className="flex space-x-2">
                            <Button
                              size="sm"
                              variant="outline"
                              className="h-8 w-8 p-0 border-teal-500 text-teal-400 hover:bg-teal-500/10"
                              disabled={status !== "connected"}
                            >
                              <Play size={16} />
                            </Button>
                            <Button
                              size="sm"
                              variant="outline"
                              className="h-8 w-8 p-0 border-teal-500 text-teal-400 hover:bg-teal-500/10"
                              disabled={status !== "connected"}
                            >
                              <Pause size={16} />
                            </Button>
                          </div>
                        </div>
                        <p className="text-sm text-slate-400">Makes the robot arm wave back and forth</p>
                      </motion.div>
                    </div>

                    <div className="mt-6">
                      <Button
                        className="bg-gradient-to-r from-teal-500 to-teal-600 hover:from-teal-400 hover:to-teal-500 border-0"
                        disabled={status !== "connected"}
                      >
                        Create New Program
                      </Button>
                    </div>
                  </CardContent>
                </Card>
              </TabsContent>
            </Tabs>
          </div>

          <div>
            <Card className="mb-6 bg-slate-800/50 border-slate-700 backdrop-blur-sm">
              <CardHeader className="pb-2">
                <CardTitle className="text-cyan-400">Robot Visualization</CardTitle>
              </CardHeader>
              <CardContent className="flex justify-center">
                <RobotVisualization />
              </CardContent>
            </Card>

            <Card className="bg-slate-800/50 border-slate-700 backdrop-blur-sm">
              <CardHeader className="pb-2">
                <CardTitle className="text-purple-400">Connection Settings</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="space-y-4">
                  <div>
                    <label className="block text-sm font-medium mb-1 text-slate-300">ROS Bridge Endpoint</label>
                    <input
                      type="text"
                      value={rosEndpoint}
                      onChange={(e) => setRosEndpoint(e.target.value)}
                      className="w-full px-3 py-2 border rounded-md bg-slate-900 border-slate-700 text-slate-300 focus:border-cyan-500 focus:ring-1 focus:ring-cyan-500"
                    />
                  </div>

                  <div className="flex space-x-2">
                    <Button
                      onClick={handleConnect}
                      className="flex-1 bg-gradient-to-r from-cyan-500 to-purple-600 hover:from-cyan-400 hover:to-purple-500 border-0"
                      disabled={status === "connected" || status === "connecting"}
                    >
                      Connect
                    </Button>
                    <Button
                      variant="outline"
                      onClick={handleDisconnect}
                      className="flex-1 border-slate-600 text-slate-300 hover:bg-slate-700"
                      disabled={status !== "connected"}
                    >
                      Disconnect
                    </Button>
                  </div>
                </div>
              </CardContent>
            </Card>

            <Card className="mt-6 bg-slate-800/50 border-slate-700 backdrop-blur-sm">
              <CardHeader className="pb-2">
                <CardTitle className="text-teal-400">System Health</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="space-y-3">
                  <div className="flex justify-between items-center">
                    <span className="text-slate-300">CPU Usage</span>
                    <div className="w-32 h-2 bg-slate-700 rounded-full overflow-hidden">
                      <div className="h-full bg-gradient-to-r from-teal-500 to-teal-400" style={{ width: "25%" }}></div>
                    </div>
                    <span className="text-teal-400 font-mono">25%</span>
                  </div>

                  <div className="flex justify-between items-center">
                    <span className="text-slate-300">Memory</span>
                    <div className="w-32 h-2 bg-slate-700 rounded-full overflow-hidden">
                      <div
                        className="h-full bg-gradient-to-r from-purple-500 to-purple-400"
                        style={{ width: "42%" }}
                      ></div>
                    </div>
                    <span className="text-purple-400 font-mono">42%</span>
                  </div>

                  <div className="flex justify-between items-center">
                    <span className="text-slate-300">Temperature</span>
                    <div className="w-32 h-2 bg-slate-700 rounded-full overflow-hidden">
                      <div className="h-full bg-gradient-to-r from-cyan-500 to-cyan-400" style={{ width: "38%" }}></div>
                    </div>
                    <span className="text-cyan-400 font-mono">38°C</span>
                  </div>

                  <div className="mt-4 p-2 bg-slate-900 rounded border border-amber-500/30 flex items-center">
                    <AlertTriangle size={16} className="text-amber-500 mr-2" />
                    <span className="text-sm text-amber-400">Servo power supply voltage low</span>
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>
        </div>
      </main>
    </div>
  )
}
