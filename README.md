# AI-Powered Guide Robot with Boston Dynamics Spot

## Overview
This project implements an AI-powered robotic guide system using the Boston Dynamics Spot robot. The robot is equipped with advanced SLAM (Simultaneous Localization and Mapping) and navigation capabilities via ROS2's Nav2 stack, allowing it to autonomously navigate complex environments. It supports voice command navigation with a wake-up word, making interaction seamless and intuitive.

Designed specifically to assist visually impaired individuals, the robot acts as a reliable guide, helping users move safely to different locations by responding to natural voice instructions. Additionally, the robot includes a DistilBERT-based Q&A assistant to provide intelligent answers to user queries, enhancing the interactive experience.

## Features
- Boston Dynamics Spot as the hardware platform.
- SLAM and Nav2 for mapping and autonomous navigation.
- Voice Command Navigation: Users can issue navigation commands verbally.
- Wake-up Word Detection using Porcupine for hands-free activation.
- DistilBERT-based Q&A assistant for answering user questions.
- Guidance for Visually Impaired: Acts as a personal assistant to help visually impaired users move independently.

## Architecture
- ROS2 Workspace (`ros2_ws`) with packages for SLAM, navigation, voice interface, and Q&A assistant.
- Integration with Porcupine Wake Word Engine to detect the wake-up call.
- Voice commands and questions are parsed and processed.
- Navigation uses Nav2 stack to plan and execute routes safely.
- DistilBERT model is integrated for natural language question answering.
- Localization and mapping are performed in real-time using SLAM.

## Installation and Setup
1. Clone the repository into your ROS2 workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/arjungirish01/AI-Powered-Robot.git

## Usage

    Activate the robot using the configured wake-up word.

    Give voice commands to navigate to pre-defined locations or ask questions.

    The robot will autonomously plan and move to the requested destination or respond with answers.


## License

This project is licensed under the MIT License. See the LICENSE file for details.
