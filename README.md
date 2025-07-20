# husarnetros

ROS 2 Talker/Listener over Husarnet

This project demonstrates how to run a basic ROS 2 publisher ("talker") and subscriber ("listener") system on computers that are not on the same local network. It uses Husarnet to create a secure peer-to-peer virtual network, allowing ROS 2 nodes to discover and communicate with each other over the internet.

Communication is specifically configured to use CycloneDDS with a custom XML file, forcing all ROS 2 traffic over the Husarnet virtual network interface.
Core Technologies

    ROS 2 Humble

    Ubuntu 22.04

    Husarnet (for peer-to-peer VPN)

    CycloneDDS (as the ROS 2 RMW/middleware)

Prerequisites

    A computer with Ubuntu 22.04.

    A working installation of ROS 2 Humble.

    A free Husarnet account, available at https://app.husarnet.com.

Setup and Installation

Follow these steps to configure the system and install all necessary components.
1. Husarnet Installation and Configuration

Husarnet creates a virtual network interface (hnet0) that your ROS 2 nodes will use.

A. Install the Husarnet Client:
Open a terminal and run the following command to download and install the Husarnet client on your Linux machine.
Generated bash

      
curl -s https://install.husarnet.com/install.sh | sudo bash

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END

B. Create a Husarnet Network:

    Log in to your Husarnet Dashboard.

    Click "Create network" to set up a new network for your project.

    In your new network's dashboard, click the "Add element" button and copy the Join Code.

C. Join the Network:
In your terminal, use the join code to connect your computer to the network. Give your computer a simple, memorable hostname. For this project, we used my-laptop.
Generated bash

      
sudo husarnet join <PASTE_YOUR_JOIN_CODE_HERE> my-laptop

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END

D. Verify the Connection:
You can test that Husarnet is working by pinging your own machine using its new hostname.
Generated bash

      
ping6 my-laptop

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END

You should see a successful ping reply, confirming the virtual network is active.
2. Install Project Dependencies

This project requires the CycloneDDS middleware for ROS 2.
First, ensure your system's package lists are up to date, then install the package.
Generated bash

      
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END
3. Build the ROS 2 Workspace

If you have cloned this repository, navigate to the workspace root and build the custom package.
Generated bash

      
cd ~/ros2_ws
colcon build

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END
Project Package and Nodes

This workspace contains a single custom package named my_package.
Package Structure
Generated code

      
ros2_ws/
├── src/
│   └── my_package/
│       ├── my_package/
│       │   ├── my_publisher_node.py  # The talker node
│       │   └── my_listener_node.py   # The listener node
│       ├── package.xml               # Package metadata and dependencies
│       └── setup.py                  # Python build file, defines executables
└── config/
    └── cyclonedds.xml                # Custom DDS configuration

    

IGNORE_WHEN_COPYING_START
Use code with caution.
IGNORE_WHEN_COPYING_END
Node Details

    my_publisher_node.py (Executable: my_node)

        Creates a node named my_publisher_node.

        Publishes a std_msgs/String message with the content "Hello from my_package: [counter]" to the /chatter topic once per second.

    my_listener_node.py (Executable: my_listener)

        Creates a node named my_listener_node.

        Subscribes to the /chatter topic.

        Prints any message it receives to the console.

CycloneDDS Configuration (cyclonedds.xml)

This is the key file for enabling communication over Husarnet. It tells CycloneDDS to:

    Ignore all other network interfaces (like Wi-Fi or Ethernet) and exclusively use the Husarnet virtual interface (hnet0).

    Disable multicast discovery, which is not supported by Husarnet.

    Use the specified peer (e.g., my-laptop) to help with node discovery.

How to Run the Talker and Listener (Single PC Test)

You can test the full system on a single computer by using two separate terminals.

Important: You must perform the environment setup (steps 1-3) in both terminals before running the nodes.
Terminal 1: Run the Talker Node

    Source the ROS 2 environment:
    Generated bash

      
source /opt/ros/humble/setup.bash

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END

Source your local workspace:
Generated bash

      
cd ~/ros2_ws
source install/setup.bash

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END

Set the environment variables to use CycloneDDS and our XML file:
Generated bash

      
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/gayathri/ros2_ws/config/cyclonedds.xml

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END

Run the talker:
Generated bash

      
ros2 run my_package my_node

    

IGNORE_WHEN_COPYING_START

    Use code with caution. Bash
    IGNORE_WHEN_COPYING_END

    You will see "Publishing..." messages appear.

Terminal 2: Run the Listener Node

    Source the ROS 2 environment:
    Generated bash

      
source /opt/ros/humble/setup.bash

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END

Source your local workspace:
Generated bash

      
cd ~/ros2_ws
source install/setup.bash

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END

Set the environment variables (this must be repeated):
Generated bash

      
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/gayathri/ros2_ws/config/cyclonedds.xml

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END

Run the listener:
Generated bash

      
ros2 run my_package my_listener

    

IGNORE_WHEN_COPYING_START
Use code with caution. Bash
IGNORE_WHEN_COPYING_END

You will see "I heard: ..." messages, confirming that the listener is receiving data from the talker through the Husarnet virtual network.
