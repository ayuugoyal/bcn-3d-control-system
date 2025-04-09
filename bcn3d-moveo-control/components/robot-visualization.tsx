"use client"

import { useEffect, useRef } from "react"
import * as THREE from "three"
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js"
import { useRobotControl } from "@/hooks/use-robot-control"

export function RobotVisualization() {
  const containerRef = useRef<HTMLDivElement>(null)
  const { jointPositions, isConnected } = useRobotControl()

  useEffect(() => {
    if (!containerRef.current) return

    // Set up scene
    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0x0f172a)

    // Set up camera
    const camera = new THREE.PerspectiveCamera(
      75,
      containerRef.current.clientWidth / containerRef.current.clientHeight,
      0.1,
      1000,
    )
    camera.position.set(0.5, 0.5, 0.5)

    // Set up renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.setSize(containerRef.current.clientWidth, containerRef.current.clientHeight)
    containerRef.current.appendChild(renderer.domElement)

    // Add orbit controls
    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true

    // Add lights
    const ambientLight = new THREE.AmbientLight(0x404040)
    scene.add(ambientLight)

    const directionalLight = new THREE.DirectionalLight(0xffffff, 1)
    directionalLight.position.set(1, 1, 1)
    scene.add(directionalLight)

    // Create robot base
    const baseGeometry = new THREE.CylinderGeometry(0.05, 0.05, 0.02, 32)
    const baseMaterial = new THREE.MeshStandardMaterial({ color: 0x22d3ee })
    const base = new THREE.Mesh(baseGeometry, baseMaterial)
    base.position.y = 0.01
    scene.add(base)

    // Create joint 1 (base rotation)
    const joint1Geometry = new THREE.BoxGeometry(0.03, 0.08, 0.03)
    const joint1Material = new THREE.MeshStandardMaterial({ color: 0xa855f7 })
    const joint1 = new THREE.Mesh(joint1Geometry, joint1Material)
    joint1.position.y = 0.06
    base.add(joint1)

    // Create link 1
    const link1Geometry = new THREE.BoxGeometry(0.025, 0.1, 0.025)
    const link1Material = new THREE.MeshStandardMaterial({ color: 0x22d3ee })
    const link1 = new THREE.Mesh(link1Geometry, link1Material)
    link1.position.y = 0.09
    joint1.add(link1)

    // Create joint 2 (shoulder)
    const joint2Geometry = new THREE.BoxGeometry(0.04, 0.03, 0.03)
    const joint2Material = new THREE.MeshStandardMaterial({ color: 0xa855f7 })
    const joint2 = new THREE.Mesh(joint2Geometry, joint2Material)
    joint2.position.y = 0.065
    link1.add(joint2)

    // Create link 2
    const link2Geometry = new THREE.BoxGeometry(0.025, 0.12, 0.025)
    const link2Material = new THREE.MeshStandardMaterial({ color: 0x22d3ee })
    const link2 = new THREE.Mesh(link2Geometry, link2Material)
    link2.position.y = 0.06
    joint2.add(link2)

    // Create joint 3 (elbow)
    const joint3Geometry = new THREE.BoxGeometry(0.04, 0.03, 0.03)
    const joint3Material = new THREE.MeshStandardMaterial({ color: 0xa855f7 })
    const joint3 = new THREE.Mesh(joint3Geometry, joint3Material)
    joint3.position.y = 0.06
    link2.add(joint3)

    // Create link 3
    const link3Geometry = new THREE.BoxGeometry(0.02, 0.1, 0.02)
    const link3Material = new THREE.MeshStandardMaterial({ color: 0x22d3ee })
    const link3 = new THREE.Mesh(link3Geometry, link3Material)
    link3.position.y = 0.05
    joint3.add(link3)

    // Create joint 4 (wrist roll)
    const joint4Geometry = new THREE.CylinderGeometry(0.02, 0.02, 0.03, 32)
    const joint4Material = new THREE.MeshStandardMaterial({ color: 0xa855f7 })
    const joint4 = new THREE.Mesh(joint4Geometry, joint4Material)
    joint4.position.y = 0.05
    joint4.rotation.x = Math.PI / 2
    link3.add(joint4)

    // Create link 4
    const link4Geometry = new THREE.BoxGeometry(0.02, 0.08, 0.02)
    const link4Material = new THREE.MeshStandardMaterial({ color: 0x22d3ee })
    const link4 = new THREE.Mesh(link4Geometry, link4Material)
    link4.position.z = 0.04
    joint4.add(link4)

    // Create joint 5 (wrist pitch)
    const joint5Geometry = new THREE.BoxGeometry(0.03, 0.02, 0.03)
    const joint5Material = new THREE.MeshStandardMaterial({ color: 0xa855f7 })
    const joint5 = new THREE.Mesh(joint5Geometry, joint5Material)
    joint5.position.z = 0.04
    link4.add(joint5)

    // Create gripper
    const gripperBaseGeometry = new THREE.BoxGeometry(0.04, 0.02, 0.02)
    const gripperBaseMaterial = new THREE.MeshStandardMaterial({ color: 0x14b8a6 })
    const gripperBase = new THREE.Mesh(gripperBaseGeometry, gripperBaseMaterial)
    gripperBase.position.y = 0.01
    joint5.add(gripperBase)

    // Create gripper fingers
    const fingerGeometry = new THREE.BoxGeometry(0.01, 0.03, 0.01)
    const fingerMaterial = new THREE.MeshStandardMaterial({ color: 0x14b8a6 })

    const leftFinger = new THREE.Mesh(fingerGeometry, fingerMaterial)
    leftFinger.position.set(-0.015, -0.015, 0)
    gripperBase.add(leftFinger)

    const rightFinger = new THREE.Mesh(fingerGeometry, fingerMaterial)
    rightFinger.position.set(0.015, -0.015, 0)
    gripperBase.add(rightFinger)

    // Add grid helper
    const gridHelper = new THREE.GridHelper(1, 10, 0x444444, 0x222222)
    scene.add(gridHelper)

    // Add axes helper
    const axesHelper = new THREE.AxesHelper(0.2)
    scene.add(axesHelper)

    // Animation loop
    const animate = () => {
      requestAnimationFrame(animate)

      // Update joint positions based on the robot state
      if (isConnected) {
        // Convert degrees to radians
        const degToRad = (deg: number) => (deg * Math.PI) / 180

        // Update joint rotations
        base.rotation.y = degToRad(jointPositions.joint1)
        joint2.rotation.x = degToRad(jointPositions.joint2)
        joint3.rotation.x = degToRad(jointPositions.joint3)
        joint4.rotation.z = degToRad(jointPositions.joint4)
        joint5.rotation.x = degToRad(jointPositions.joint5)

        // Update gripper (scale between 0-100%)
        const gripperOpeningScale = jointPositions.gripper / 100
        leftFinger.position.x = -0.015 - 0.01 * gripperOpeningScale
        rightFinger.position.x = 0.015 + 0.01 * gripperOpeningScale
      }

      controls.update()
      renderer.render(scene, camera)
    }

    animate()

    // Handle window resize
    const handleResize = () => {
      if (!containerRef.current) return

      camera.aspect = containerRef.current.clientWidth / containerRef.current.clientHeight
      camera.updateProjectionMatrix()
      renderer.setSize(containerRef.current.clientWidth, containerRef.current.clientHeight)
    }

    window.addEventListener("resize", handleResize)

    // Clean up
    return () => {
      window.removeEventListener("resize", handleResize)
      if (containerRef.current) {
        containerRef.current.removeChild(renderer.domElement)
      }
    }
  }, [jointPositions, isConnected])

  return (
    <div
      ref={containerRef}
      className="w-full h-64 bg-slate-900/80 rounded-lg flex items-center justify-center border border-slate-700 overflow-hidden"
    >
      {/* {!isConnected && (
        <div className="absolute inset-0 flex items-center justify-center bg-slate-900/50 z-10">
          <p className="text-slate-400">Connect to robot to view 3D visualization</p>
        </div>
      )} */}
    </div>
  )
}
