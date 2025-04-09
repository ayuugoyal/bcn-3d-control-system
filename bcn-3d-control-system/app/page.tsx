import Image from "next/image"
import Link from "next/link"
import { Button } from "@/components/ui/button"
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card"

export default function Home() {
  return (
    <div className="flex flex-col min-h-screen bg-gradient-to-br from-slate-900 to-slate-800 text-white">
      <header className="border-b border-slate-700 bg-slate-900/80 backdrop-blur-sm sticky top-0 z-10">
        <div className="container mx-auto px-4 py-4 flex justify-between items-center">
          <div className="flex items-center space-x-2">
            <div className="bg-white h-10 rounded-lg flex items-center justify-center">
              <Image src="/logo.svg" alt="BCN3D Moveo Controller" width={100} height={30} />
            </div>
            <h1 className="text-xl font-bold bg-gradient-to-r from-cyan-400 to-purple-500 text-transparent bg-clip-text">
              BCN3D Moveo Controller
            </h1>
          </div>
          <nav>
            <ul className="flex space-x-4">
              <li>
                <Link href="/" className="font-medium text-cyan-400 hover:text-cyan-300 transition-colors">
                  Home
                </Link>
              </li>
              <li>
                <Link href="/control" className="font-medium text-slate-300 hover:text-cyan-300 transition-colors">
                  Control Panel
                </Link>
              </li>
              <li>
                <Link href="/settings" className="font-medium text-slate-300 hover:text-cyan-300 transition-colors">
                  Settings
                </Link>
              </li>
            </ul>
          </nav>
        </div>
      </header>

      <main className="flex-grow container mx-auto px-4 py-8">
        <section className="mb-12">
          <div className="text-center mb-8">
            <h2 className="text-4xl font-bold mb-4 bg-gradient-to-r from-cyan-400 via-purple-500 to-cyan-400 text-transparent bg-clip-text">
              BCN3D Moveo Robot Arm Control System
            </h2>
            <p className="text-xl text-slate-300 max-w-3xl mx-auto">
              A modern web interface to control your open-source 3D printed robot arm using ROS2 and Raspberry Pi
            </p>
          </div>

          <div className="flex justify-center">
            <div className="relative rounded-lg overflow-hidden shadow-2xl shadow-cyan-500/20 border border-slate-700">
              <div className="absolute inset-0 bg-gradient-to-r from-cyan-500/20 to-purple-600/20 z-0"></div>
              <Image
                src="/image.png"
                alt="BCN3D Moveo Robot Arm"
                width={600}
                height={400}
                className="relative z-10"
              />
            </div>
          </div>
        </section>

        <section className="grid grid-cols-1 md:grid-cols-3 gap-6 mb-12">
          <Card className="bg-slate-800/50 border-slate-700 backdrop-blur-sm hover:shadow-lg hover:shadow-cyan-500/10 transition-all">
            <CardHeader className="pb-2">
              <CardTitle className="text-cyan-400">Control Panel</CardTitle>
              <CardDescription className="text-slate-400">
                Operate your robot arm with an intuitive interface
              </CardDescription>
            </CardHeader>
            <CardContent>
              <p className="text-slate-300">
                Access joint controls, inverse kinematics, and pre-programmed movements for your BCN3D Moveo.
              </p>
            </CardContent>
            <CardFooter>
              <Button
                asChild
                className="w-full bg-gradient-to-r from-cyan-500 to-purple-600 hover:from-cyan-400 hover:to-purple-500 text-white border-0"
              >
                <Link href="/control">Open Control Panel</Link>
              </Button>
            </CardFooter>
          </Card>

          <Card className="bg-slate-800/50 border-slate-700 backdrop-blur-sm hover:shadow-lg hover:shadow-purple-500/10 transition-all">
            <CardHeader className="pb-2">
              <CardTitle className="text-purple-400">System Status</CardTitle>
              <CardDescription className="text-slate-400">Monitor your robot and Raspberry Pi</CardDescription>
            </CardHeader>
            <CardContent>
              <p className="text-slate-300">
                View real-time data about your robot's position, system health, and connection status.
              </p>
            </CardContent>
            <CardFooter>
              <Button
                asChild
                variant="outline"
                className="w-full border-purple-500 text-purple-400 hover:bg-purple-500/10"
              >
                <Link href="/status">Check Status</Link>
              </Button>
            </CardFooter>
          </Card>

          <Card className="bg-slate-800/50 border-slate-700 backdrop-blur-sm hover:shadow-lg hover:shadow-teal-500/10 transition-all">
            <CardHeader className="pb-2">
              <CardTitle className="text-teal-400">Programming</CardTitle>
              <CardDescription className="text-slate-400">Create custom movement sequences</CardDescription>
            </CardHeader>
            <CardContent>
              <p className="text-slate-300">
                Program and save movement sequences for your robot to perform repetitive tasks.
              </p>
            </CardContent>
            <CardFooter>
              <Button asChild variant="outline" className="w-full border-teal-500 text-teal-400 hover:bg-teal-500/10">
                <Link href="/programming">Start Programming</Link>
              </Button>
            </CardFooter>
          </Card>
        </section>

        <section className="bg-slate-800/30 rounded-lg p-6 border border-slate-700">
          <h2 className="text-2xl font-bold mb-4 text-cyan-400">Getting Started</h2>
          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
            <div>
              <h3 className="text-xl font-semibold mb-2 text-purple-400">System Requirements</h3>
              <ul className="list-disc pl-5 space-y-1 text-slate-300">
                <li>Assembled BCN3D Moveo robot arm</li>
                <li>Raspberry Pi 4 (recommended) or 3B+</li>
                <li>Arduino board (as per BCN3D Moveo specifications)</li>
                <li>Power supply for Raspberry Pi and servos</li>
                <li>Network connection for the Raspberry Pi</li>
              </ul>
            </div>
            <div>
              <h3 className="text-xl font-semibold mb-2 text-teal-400">Software Stack</h3>
              <ul className="list-disc pl-5 space-y-1 text-slate-300">
                <li>Ubuntu Server 22.04 LTS for Raspberry Pi</li>
                <li>ROS2 Humble Hawksbill</li>
                <li>Next.js web interface</li>
                <li>ROSBridge for WebSocket communication</li>
                <li>Arduino firmware for motor control</li>
              </ul>
            </div>
          </div>
        </section>
      </main>

      <footer className="bg-slate-900 border-t border-slate-700">
        <div className="container mx-auto px-4 py-6">
          <p className="text-center text-slate-400">
            BCN3D Moveo Controller © {new Date().getFullYear()} | Open Source Project
          </p>
        </div>
      </footer>
    </div>
  )
}
