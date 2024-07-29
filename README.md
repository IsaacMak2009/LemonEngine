# LemonEngine
### A simple python library for robotics beginners

> [!warning] 
> 
> ## This project is Incomplete 
>
>**This library is currently in a pre-release state and is not ready for general use. It contains known bugs and missing functionality. Using this library in production is not recommended at this time.**
>
>We welcome any contributions or bug reports from the community. Please feel free to submit pull requests or open issues on the project's GitHub repository. Your feedback and support will help us improve the library and work towards a stable release.
>
>Thank you for your understanding.

---
# API reference
### `LemonEngine.ai`
The `LemonEngine.ai` module provides access to a variety of AI models commonly used in robotics applications, such as object detection, segmentation, and classification. Some popular models included in this module are:

 - YOLO (You Only Look Once): A real-time object detection system that can identify multiple objects in a single image or video frame.

> More models coming soon...

### `LemonEngine.hardwares`
The `LemonEngine.hardwares` module provides APIs for interacting with various hardware components commonly used in robotics, such as motors, sensors, and actuators. This module is built on top of the Robot Operating System (ROS) framework, allowing for seamless integration with ROS-based systems.

> Incomplete 👷‍♂️

### `LemonEngine.sensors`
The `LemonEngine.sensors` module offers APIs for working with a wide range of sensors commonly used in robotics, including:

 - Camera sensors (RGB, depth, thermal)
 - Inertial Measurement Units (IMUs)
 - Light Detection and Ranging (LiDAR) sensors

This module also integrates with the ROS sensor processing ecosystem, allowing you to leverage existing ROS sensor drivers and processing pipelines.

> More type of sensors coming soon...

### `LemonEngine.utils`
The `LemonEngine.utils` module provides a collection of utility functions and tools to simplify common tasks in robotics development, such as:

 - Timer
 - Ros topic, service checker

These utilities are designed to help you write more efficient and maintainable code, allowing you to focus on the high-level aspects of your robotics applications.