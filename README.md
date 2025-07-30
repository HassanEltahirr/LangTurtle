# ROS2 Turtlesim LLM Agent

## Overview

This project is a command-line agent that lets you control a ROS2 turtlesim turtle using natural language commands, powered by an OpenAI LLM (via LangChain). The agent can teleport the turtle, return it to the center, and respond to "thank you" commands by turning off the pen and moving the turtle to the center.

---

## Why makes this project useful?

- **Interdisciplinary Skills:**  
  - Combines robotics (ROS2, turtlesim), AI/NLP (OpenAI, LangChain), and software engineering (Python, tool integration).
- **Real-World Application:**  
  - Demonstrates how to build a natural language interface for a robot, a hot topic in both industry and research.
- **Technical Depth:**  
  - Shows hands-on experience with ROS2, LLMs, and Python.
- **Problem Solving & Creativity:**  
  - Integrates multiple technologies, handles user input, and manages error cases.
---

## Features

- **Natural language interface** for controlling turtlesim.
- **Tools for teleporting the turtle, returning to start, and handling "thank you" commands.**
- **Integration with ROS2** via `rclpy` and `turtlesim`.
- **Uses LangChain and OpenAI** for LLM-based command interpretation.

---

## Requirements

- Python 3.10+
- ROS2 (with turtlesim package)
- `rclpy` (ROS2 Python client library)
- `turtlesim` (ROS2 turtlesim package)
- `langchain`
- `openai`

---

## Setup Instructions

1. **Install ROS2 and turtlesim**  
   Follow the official ROS2 installation guide for your OS, then run:  
   ```sh
   sudo apt install ros-<distro>-turtlesim
   ```

2. **Create and activate a Python virtual environment:**  
   ```sh
   python3.10 -m venv venv
   source venv/bin/activate
   ```

3. **Install Python dependencies:**  
   ```sh
   pip install langchain openai rclpy
   ```
   (You may need to install ROS2 Python dependencies via your OS package manager.)

4. **Set your OpenAI API key:**  
   - The key is currently hardcoded in `llm.py`. For security, consider using environment variables instead.

---
Link to youtube video: https://www.youtube.com/watch?v=YyDtbpm66SE
## Usage

- Start the ROS2 turtlesim node in a separate terminal:  
  ```sh
  ros2 run turtlesim turtlesim_node
  ```
- Run the agent:  
  ```sh
  python main.py
  ```
- Enter commands like:
  - `go to 3,4`
  - `go back to center`
  - `thank you mr turtle`
  - Type `exit` or `quit` to stop.

---

## Available Commands

- **teleport_turtle:** Teleports the turtle to specified coordinates (e.g., "go to 5,3").
- **return_to_start:** Returns the turtle to the center (5.5, 5.5).
- **thank_you_turtle_tool:** Turns off the pen, teleports to center, and responds to "thank you mr turtle".

---

## File Structure

- `main.py`: Entry point, runs the agent loop.
- `tools.py`: ROS2 tool definitions for turtle control.
- `llm.py`: LLM setup (OpenAI via LangChain).
- `prompts.py`: Agent prompt template.

---

## Security Note

The OpenAI API key is hardcoded in `llm.py`. For production, use environment variables or a config file.

## License

(Add your license here.)
