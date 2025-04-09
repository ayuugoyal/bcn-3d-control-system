"use client"

import { useState } from "react"
import Link from "next/link"
import { Button } from "@/components/ui/button"
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { Input } from "@/components/ui/input"
import { Switch } from "@/components/ui/switch"
import { Label } from "@/components/ui/label"
import { Slider } from "@/components/ui/slider"
import { ArrowLeft, Save, RefreshCw, Download, Terminal, Shield, Cpu, Wifi } from "lucide-react"

export default function Settings() {
  const [robotSettings, setRobotSettings] = useState({
    maxSpeed: 50,
    acceleration: 30,
    safetyEnabled: true,
    workspaceLimit: true,
    collisionDetection: true,
    jointLimits: true,
  })

  const [networkSettings, setNetworkSettings] = useState({
    rosIp: "192.168.1.100",
    rosPort: "9090",
    webserverPort: "3000",
    autoConnect: true,
  })

  const handleRobotSettingChange = (setting, value) => {
    setRobotSettings((prev) => ({
      ...prev,
      [setting]: value,
    }))
  }

  const handleNetworkSettingChange = (setting, value) => {
    setNetworkSettings((prev) => ({
      ...prev,
      [setting]: value,
    }))
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
            Settings
          </h1>
          <div></div>
        </div>
      </header>

      <main className="flex-grow container mx-auto px-4 py-8">
        <Tabs defaultValue="robot">
          <TabsList className="grid w-full grid-cols-3 max-w-md mx-auto mb-8 bg-slate-800 border border-slate-700">
            <TabsTrigger
              value="robot"
              className="data-[state=active]:bg-gradient-to-r data-[state=active]:from-cyan-500 data-[state=active]:to-cyan-600 data-[state=active]:text-white"
            >
              <Shield className="w-4 h-4 mr-2" />
              Robot
            </TabsTrigger>
            <TabsTrigger
              value="network"
              className="data-[state=active]:bg-gradient-to-r data-[state=active]:from-purple-500 data-[state=active]:to-purple-600 data-[state=active]:text-white"
            >
              <Wifi className="w-4 h-4 mr-2" />
              Network
            </TabsTrigger>
            <TabsTrigger
              value="system"
              className="data-[state=active]:bg-gradient-to-r data-[state=active]:from-teal-500 data-[state=active]:to-teal-600 data-[state=active]:text-white"
            >
              <Cpu className="w-4 h-4 mr-2" />
              System
            </TabsTrigger>
          </TabsList>

          <TabsContent value="robot">
            <Card className="bg-slate-800/50 border-slate-700 backdrop-blur-sm">
              <CardHeader className="pb-2">
                <CardTitle className="text-cyan-400">Robot Settings</CardTitle>
                <CardDescription className="text-slate-400">
                  Configure the behavior and safety settings of your BCN3D Moveo
                </CardDescription>
              </CardHeader>
              <CardContent className="space-y-6">
                <div className="space-y-4">
                  <div>
                    <div className="flex justify-between items-center mb-2">
                      <Label htmlFor="max-speed" className="text-slate-300">
                        Maximum Speed
                      </Label>
                      <span className="text-sm font-mono bg-slate-900 px-2 py-1 rounded text-cyan-400">
                        {robotSettings.maxSpeed}%
                      </span>
                    </div>
                    <Slider
                      id="max-speed"
                      value={[robotSettings.maxSpeed]}
                      min={10}
                      max={100}
                      step={1}
                      onValueChange={(value) => handleRobotSettingChange("maxSpeed", value[0])}
                    />
                    <p className="text-xs text-slate-400 mt-1">Controls the maximum speed of all robot movements</p>
                  </div>

                  <div>
                    <div className="flex justify-between items-center mb-2">
                      <Label htmlFor="acceleration" className="text-slate-300">
                        Acceleration
                      </Label>
                      <span className="text-sm font-mono bg-slate-900 px-2 py-1 rounded text-cyan-400">
                        {robotSettings.acceleration}%
                      </span>
                    </div>
                    <Slider
                      id="acceleration"
                      value={[robotSettings.acceleration]}
                      min={10}
                      max={100}
                      step={1}
                      onValueChange={(value) => handleRobotSettingChange("acceleration", value[0])}
                    />
                    <p className="text-xs text-slate-400 mt-1">
                      Controls how quickly the robot accelerates to its target speed
                    </p>
                  </div>
                </div>

                <div className="space-y-4 pt-4">
                  <div className="flex items-center justify-between p-3 rounded-lg bg-slate-900/50 border border-slate-700 hover:border-cyan-500/30 transition-colors">
                    <div>
                      <Label htmlFor="safety-enabled" className="font-medium text-slate-300">
                        Safety Mode
                      </Label>
                      <p className="text-sm text-slate-400">Enables additional safety checks during operation</p>
                    </div>
                    <Switch
                      id="safety-enabled"
                      checked={robotSettings.safetyEnabled}
                      onCheckedChange={(checked) => handleRobotSettingChange("safetyEnabled", checked)}
                      className="data-[state=checked]:bg-cyan-500"
                    />
                  </div>

                  <div className="flex items-center justify-between p-3 rounded-lg bg-slate-900/50 border border-slate-700 hover:border-cyan-500/30 transition-colors">
                    <div>
                      <Label htmlFor="workspace-limit" className="font-medium text-slate-300">
                        Workspace Limits
                      </Label>
                      <p className="text-sm text-slate-400">Restricts movement to a predefined workspace</p>
                    </div>
                    <Switch
                      id="workspace-limit"
                      checked={robotSettings.workspaceLimit}
                      onCheckedChange={(checked) => handleRobotSettingChange("workspaceLimit", checked)}
                      className="data-[state=checked]:bg-cyan-500"
                    />
                  </div>

                  <div className="flex items-center justify-between p-3 rounded-lg bg-slate-900/50 border border-slate-700 hover:border-cyan-500/30 transition-colors">
                    <div>
                      <Label htmlFor="collision-detection" className="font-medium text-slate-300">
                        Collision Detection
                      </Label>
                      <p className="text-sm text-slate-400">Stops movement when unexpected resistance is detected</p>
                    </div>
                    <Switch
                      id="collision-detection"
                      checked={robotSettings.collisionDetection}
                      onCheckedChange={(checked) => handleRobotSettingChange("collisionDetection", checked)}
                      className="data-[state=checked]:bg-cyan-500"
                    />
                  </div>

                  <div className="flex items-center justify-between p-3 rounded-lg bg-slate-900/50 border border-slate-700 hover:border-cyan-500/30 transition-colors">
                    <div>
                      <Label htmlFor="joint-limits" className="font-medium text-slate-300">
                        Joint Limits
                      </Label>
                      <p className="text-sm text-slate-400">Enforces software limits on joint movements</p>
                    </div>
                    <Switch
                      id="joint-limits"
                      checked={robotSettings.jointLimits}
                      onCheckedChange={(checked) => handleRobotSettingChange("jointLimits", checked)}
                      className="data-[state=checked]:bg-cyan-500"
                    />
                  </div>
                </div>
              </CardContent>
              <CardFooter>
                <Button className="w-full bg-gradient-to-r from-cyan-500 to-cyan-600 hover:from-cyan-400 hover:to-cyan-500 border-0">
                  <Save className="w-4 h-4 mr-2" />
                  Save Robot Settings
                </Button>
              </CardFooter>
            </Card>
          </TabsContent>

          <TabsContent value="network">
            <Card className="bg-slate-800/50 border-slate-700 backdrop-blur-sm">
              <CardHeader className="pb-2">
                <CardTitle className="text-purple-400">Network Settings</CardTitle>
                <CardDescription className="text-slate-400">
                  Configure the network connection between the web interface and ROS2
                </CardDescription>
              </CardHeader>
              <CardContent className="space-y-6">
                <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                  <div>
                    <Label htmlFor="ros-ip" className="text-slate-300">
                      ROS Bridge IP Address
                    </Label>
                    <Input
                      id="ros-ip"
                      value={networkSettings.rosIp}
                      onChange={(e) => handleNetworkSettingChange("rosIp", e.target.value)}
                      className="mt-1 bg-slate-900 border-slate-700 text-slate-300 focus:border-purple-500 focus:ring-1 focus:ring-purple-500"
                    />
                    <p className="text-xs text-slate-400 mt-1">IP address of the Raspberry Pi running ROS2</p>
                  </div>

                  <div>
                    <Label htmlFor="ros-port" className="text-slate-300">
                      ROS Bridge Port
                    </Label>
                    <Input
                      id="ros-port"
                      value={networkSettings.rosPort}
                      onChange={(e) => handleNetworkSettingChange("rosPort", e.target.value)}
                      className="mt-1 bg-slate-900 border-slate-700 text-slate-300 focus:border-purple-500 focus:ring-1 focus:ring-purple-500"
                    />
                    <p className="text-xs text-slate-400 mt-1">Port for the ROSBridge WebSocket server</p>
                  </div>

                  <div>
                    <Label htmlFor="webserver-port" className="text-slate-300">
                      Web Server Port
                    </Label>
                    <Input
                      id="webserver-port"
                      value={networkSettings.webserverPort}
                      onChange={(e) => handleNetworkSettingChange("webserverPort", e.target.value)}
                      className="mt-1 bg-slate-900 border-slate-700 text-slate-300 focus:border-purple-500 focus:ring-1 focus:ring-purple-500"
                    />
                    <p className="text-xs text-slate-400 mt-1">Port for the Next.js web server</p>
                  </div>

                  <div className="flex items-center space-x-4">
                    <div className="flex-grow pt-6">
                      <div className="flex items-center space-x-2">
                        <Switch
                          id="auto-connect"
                          checked={networkSettings.autoConnect}
                          onCheckedChange={(checked) => handleNetworkSettingChange("autoConnect", checked)}
                          className="data-[state=checked]:bg-purple-500"
                        />
                        <Label htmlFor="auto-connect" className="text-slate-300">
                          Auto-connect on startup
                        </Label>
                      </div>
                    </div>
                  </div>
                </div>
              </CardContent>
              <CardFooter>
                <Button className="w-full bg-gradient-to-r from-purple-500 to-purple-600 hover:from-purple-400 hover:to-purple-500 border-0">
                  <Save className="w-4 h-4 mr-2" />
                  Save Network Settings
                </Button>
              </CardFooter>
            </Card>
          </TabsContent>

          <TabsContent value="system">
            <Card className="bg-slate-800/50 border-slate-700 backdrop-blur-sm">
              <CardHeader className="pb-2">
                <CardTitle className="text-teal-400">System Settings</CardTitle>
                <CardDescription className="text-slate-400">
                  Configure system-level settings for the Raspberry Pi controller
                </CardDescription>
              </CardHeader>
              <CardContent className="space-y-6">
                <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                  <div>
                    <h3 className="text-lg font-medium mb-2 text-teal-400">System Information</h3>
                    <div className="bg-slate-900/80 p-4 rounded-lg space-y-2 border border-slate-700">
                      <div className="flex justify-between">
                        <span className="text-slate-400">Hostname:</span>
                        <span className="font-mono text-teal-400">bcn3d-moveo</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-slate-400">IP Address:</span>
                        <span className="font-mono text-teal-400">192.168.1.100</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-slate-400">ROS2 Version:</span>
                        <span className="font-mono text-teal-400">Humble</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-slate-400">CPU Usage:</span>
                        <span className="font-mono text-teal-400">23%</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-slate-400">Memory Usage:</span>
                        <span className="font-mono text-teal-400">512MB / 2GB</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-slate-400">Uptime:</span>
                        <span className="font-mono text-teal-400">2d 4h 37m</span>
                      </div>
                    </div>
                  </div>

                  <div>
                    <h3 className="text-lg font-medium mb-2 text-teal-400">System Actions</h3>
                    <div className="space-y-3">
                      <Button
                        variant="outline"
                        className="w-full justify-start border-teal-500/50 text-teal-400 hover:bg-teal-500/10"
                      >
                        <RefreshCw className="w-4 h-4 mr-2" />
                        Restart ROS2 Services
                      </Button>
                      <Button
                        variant="outline"
                        className="w-full justify-start border-teal-500/50 text-teal-400 hover:bg-teal-500/10"
                      >
                        <RefreshCw className="w-4 h-4 mr-2" />
                        Update ROS2 Packages
                      </Button>
                      <Button
                        variant="outline"
                        className="w-full justify-start border-teal-500/50 text-teal-400 hover:bg-teal-500/10"
                      >
                        <Save className="w-4 h-4 mr-2" />
                        Backup Configuration
                      </Button>
                      <Button
                        variant="outline"
                        className="w-full justify-start border-teal-500/50 text-teal-400 hover:bg-teal-500/10"
                      >
                        <RefreshCw className="w-4 h-4 mr-2" />
                        Restore Default Settings
                      </Button>
                      <Button
                        variant="destructive"
                        className="w-full justify-start bg-red-600 hover:bg-red-700 border-0"
                      >
                        <RefreshCw className="w-4 h-4 mr-2" />
                        Reboot Raspberry Pi
                      </Button>
                    </div>
                  </div>
                </div>

                <div className="pt-4">
                  <h3 className="text-lg font-medium mb-2 text-teal-400 flex items-center">
                    <Terminal className="w-5 h-5 mr-2" />
                    Logs
                  </h3>
                  <div className="bg-slate-950 text-emerald-400 p-4 rounded-lg font-mono text-sm h-48 overflow-y-auto border border-slate-700">
                    <p>[INFO] [1682345678.123456] [robot_controller]: Robot controller started</p>
                    <p>[INFO] [1682345679.234567] [moveit_controller]: MoveIt controller initialized</p>
                    <p>[INFO] [1682345680.345678] [rosbridge_server]: WebSocket server started on port 9090</p>
                    <p>[INFO] [1682345681.456789] [joint_state_publisher]: Publishing joint states</p>
                    <p>[INFO] [1682345682.567890] [robot_state_publisher]: Publishing robot state</p>
                    <p>[INFO] [1682345683.678901] [moveit_controller]: Planning scene initialized</p>
                    <p>[INFO] [1682345684.789012] [robot_controller]: Robot ready for commands</p>
                    <p>[INFO] [1682345685.890123] [web_interface]: Web interface connected</p>
                    <p>[INFO] [1682345686.901234] [robot_controller]: Received joint state command</p>
                    <p>[INFO] [1682345687.012345] [robot_controller]: Executing joint state command</p>
                  </div>
                </div>
              </CardContent>
              <CardFooter>
                <Button variant="outline" className="w-full border-teal-500/50 text-teal-400 hover:bg-teal-500/10">
                  <Download className="w-4 h-4 mr-2" />
                  Download System Logs
                </Button>
              </CardFooter>
            </Card>
          </TabsContent>
        </Tabs>
      </main>
    </div>
  )
}
