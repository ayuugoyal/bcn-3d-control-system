import type React from "react"
import type { Metadata } from "next"
import { Inter } from "next/font/google"
import "./globals.css"
import { ThemeProvider } from "@/components/theme-provider"
import { RosProvider } from "@/contexts/ros-context"

const inter = Inter({ subsets: ["latin"] })

export const metadata: Metadata = {
  title: "BCN3D Moveo Controller",
  description: "Control interface for the BCN3D Moveo robot arm",
}

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode
}>) {
  return (
    <html lang="en" suppressHydrationWarning>
      <body className={inter.className}>
        <ThemeProvider attribute="class" defaultTheme="dark" enableSystem disableTransitionOnChange>
          <RosProvider>{children}</RosProvider>
        </ThemeProvider>
      </body>
    </html>
  )
}


import './globals.css'