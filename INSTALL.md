# Installation Guide

This document provides instructions for setting up and running both the server (on a Raspberry Pi) and the client (on a macOS/PC) for the forklift project.

## Server Setup (Raspberry Pi)

1.  **Clone the Repository**
    ```bash
    git clone https://github.com/your-username/your-repository-name.git
    cd your-repository-name/server
    ```

2.  **Install System Dependencies**
    The server relies on system-level packages for hardware and camera interaction. It is highly recommended to install these first using `apt`.
    ```bash
    sudo apt update
    sudo apt install -y python3-picamera2 python3-rpi.gpio python3-libcamera python3-numpy python3-opencv
    ```

3.  **Install Python Dependencies**
    Install the required Python packages using the `requirements.txt` file.
    ```bash
    pip install -r requirements.txt
    ```

4.  **Run the Server**
    From within the `server` directory, run:
    ```bash
    python3 src/main.py
    ```

## Client Setup (macOS/PC)

1.  **Clone the Repository (if not already done)**
    ```bash
    git clone https://github.com/your-username/your-repository-name.git
    cd your-repository-name/client
    ```

2.  **Create a Virtual Environment (Recommended)**
    It's good practice to use a virtual environment to manage dependencies.
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    ```
    _On Windows, the activation command is `venv\Scripts\activate`_

3.  **Install Python Dependencies**
    Install the required Python packages using the `requirements.txt` file.
    ```bash
    pip install -r requirements.txt
    ```

4.  **Run the Client**
    From within the `client` directory, run:
    ```bash
    python3 src/main.py
    ``` 