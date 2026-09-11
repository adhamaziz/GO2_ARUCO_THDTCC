# Unitree Go2 ArUco Marker Detector (Dockerized ROS 2)

This repository provides a containerized ROS 2 environment for detecting ArUco markers on the Unitree Go2 robot using an onboard RealSense camera.

---

## 📁 Repository Structure

```text
go2_aruco_docker/
├── Dockerfile.aruco          # Container setup & dependencies
├── aruco_detector_node.py    # ROS 2 node for ArUco tag detection
├── entrypoint.sh             # ROS 2 workspace environment setup
└── launch.sh                 # Convenience script to start the container/node
