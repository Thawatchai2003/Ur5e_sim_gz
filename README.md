# UR5e Voice Control

Voice-controlled system for controlling the UR5e collaborative robot using ROS2, MoveIt2, and Speech Recognition.

---

# System Requirements

Before starting the installation, make sure the following software and tools are installed on your system.

- Ubuntu 22.04
- ROS2 Humble
- Python 3
- MoveIt2
- UR5e Robot

---

# Prerequisites Installation

## 1. Update Ubuntu Packages

Update and upgrade all Ubuntu packages before installation.

### Command
```bash
sudo apt update
sudo apt upgrade -y
```

### Description
- `sudo` : Execute command with administrator privileges
- `apt update` : Update package lists
- `apt upgrade` : Upgrade installed packages
- `-y` : Automatically confirm installation

---

## 2. Install Python3 and pip

Install Python3 and pip package manager.

### Command
```bash
sudo apt install python3 python3-pip -y
```

### Description
- `python3` : Python programming language
- `python3-pip` : Python package manager
- `-y` : Automatically confirm installation

---

## 3. Install Colcon Build Tool

Install colcon build tools for ROS2 workspace compilation.

### Command
```bash
sudo apt install python3-colcon-common-extensions -y
```

### Description
- `python3-colcon-common-extensions` : ROS2 build tool extensions

---

## 4. Install Git

Install Git for cloning repositories from GitHub.

### Command
```bash
sudo apt install git -y
```

### Description
- `git` : Version control system for downloading repositories

---

## 5. Install ROS2 Humble

Install ROS2 Humble Desktop version.

### Command
```bash
sudo apt install ros-humble-desktop -y
```

### Description
- `ros-humble-desktop` : Full ROS2 Humble desktop installation

---

## 6. Source ROS2 Environment

Load the ROS2 environment setup.

### Command
```bash
source /opt/ros/humble/setup.bash
```

### Description
- `source` : Load environment variables
- `/opt/ros/humble/setup.bash` : ROS2 Humble setup file

---

## 7. Add ROS2 Environment Automatically

Automatically source ROS2 setup when opening terminal.

### Command
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Description
- `echo` : Write command into `.bashrc`
- `>>` : Append text to file
- `~/.bashrc` : Bash configuration file
- `source ~/.bashrc` : Reload bash configuration

---

# Workspace Installation

## 1. Create a ROS2 Workspace

Create a ROS2 workspace for the project.

### Command
```bash
mkdir -p ~/ur5e_ws/src
```

### Description
- `mkdir` : Create a new folder
- `-p` : Create nested folders automatically
- `~/UR5e_ws/src` : Workspace and source folder location

---

## 2. Clone UR5e Voice Control Repository

Clone the UR5e Voice Control package from GitHub into the workspace source folder.

### Command
```bash
cd ~/UR5e_ws

git clone https://github.com/your_username/ur5e_voice_control.git
```

### Description
- `cd ~/UR5e_ws/src` : Move to the ROS2 source folder
- `git clone` : Download repository from GitHub
- `UR5e_voice_control.git` : UR5e Voice Control repository
