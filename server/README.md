# Forklift Server

This is the server component of the Forklift project, designed to run on a Raspberry Pi. It handles video streaming, motor control, and command processing.

## Requirements

### System Dependencies
- Raspberry Pi OS (Bullseye or newer)
- Python 3.9 or newer
- Camera interface enabled
- GPIO access

### System Package Installation
```bash
# Update package lists
sudo apt update

# Install system dependencies
sudo apt install -y python3-picamera2  # For camera support
sudo apt install -y python3-rpi.gpio   # For GPIO access
```

### Python Dependencies
- RPi.GPIO
- opencv-python
- numpy
- python-dotenv
- websockets

## Setup

1. Enable the camera interface:
```bash
sudo raspi-config
# Navigate to Interface Options -> Camera and enable it
```

2. Create and activate a virtual environment:
```bash
python -m venv venv
source venv/bin/activate
```

3. Install Python dependencies:
```bash
pip install -r requirements.txt
```

## Usage

1. Create a `.env` file in the server directory with your configuration:
```env
HOST=0.0.0.0
COMMAND_PORT=3456
VIDEO_PORT=3457
```

2. Start the server:
```bash
python src/main.py
```

## Hardware Connections

### Motor Control
- GPIO pins for motor control (to be documented)

### Servo Control
- GPIO pins for servo control (to be documented)

### Camera
- Raspberry Pi Camera Module connected to the camera port

## Network Protocol

### Command Server (WebSocket)
- Port: 3456 (configurable)
- Protocol: WebSocket
- Commands:
  - motor: Control motor speed
  - servo: Control servo position
  - emergency_stop: Stop all movement

### Video Stream (WebSocket)
- Port: 3457 (configurable)
- Protocol: WebSocket
- Format: JPEG frames at 30 FPS

## Troubleshooting

### Camera Issues
1. Verify camera is enabled:
```bash
vcgencmd get_camera
# Should show supported=1 detected=1
```

2. Test camera with:
```bash
libcamera-hello
```

### GPIO Issues
1. Verify GPIO access:
```bash
groups
# Should include 'gpio' group
```

2. If not in gpio group:
```bash
sudo usermod -a -G gpio $USER
# Log out and back in for changes to take effect
```

## Development

### Code Structure
- `src/`: Main source code
  - `network/`: Network communication components
  - `controllers/`: Hardware control components
  - `config.py`: Configuration management

### Adding New Features
1. Create a new branch
2. Implement changes
3. Test thoroughly
4. Submit pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details. 