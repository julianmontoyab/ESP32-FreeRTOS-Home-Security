# 🚨 ESP32 FreeRTOS Home Security System

> Transform your home into a smart fortress using the power of FreeRTOS on ESP32! 🏠🔒
> 

## 🎯 Project Overview

This DIY home security system leverages **FreeRTOS** real-time operating system to manage multiple concurrent tasks on an ESP32, creating a sophisticated security solution that:
- 🚪 Monitors door sensors in real-time
- ⏰ Activates during specified hours (11 PM - 5 AM)
- 🚨 Triggers whole-house alerts via Google Home
- 🔴🔵 Transforms all smart LEDs into police siren mode
- 🗣️ Can be deactivated with: "Hey Google, deactivate alarm"

## ✨ Key Features

### 🧠 FreeRTOS Implementation
- **6 Concurrent Tasks** running independently
- **Dual-Core Processing** - Network ops on Core 0, Real-time ops on Core 1
- **Inter-Task Communication** via Queues, Semaphores, and Event Groups
- **Priority-Based Scheduling** for critical alarm response
- **Resource Management** with Mutexes and Task Notifications

### 📊 Task Distribution
