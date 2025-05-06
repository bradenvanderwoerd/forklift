# Forklift Server

This is the server component of the Forklift project, designed to run on a Raspberry Pi. It handles motor control, servo control, and video streaming.

## Requirements

- Raspberry Pi (4 recommended)
- Raspberry Pi Camera Module
- Motor driver board (e.g., L298N)
- DC motors (2x)
- Servo motor
- Python 3.7+

## Setup

1. Install system dependencies:
   ```bash
   sudo apt-get update
   sudo apt-get install -y python3-pip python3-venv
   ```

2. Create and activate virtual environment:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

3. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Enable camera interface:
   ```bash
   sudo raspi-config
   # Navigate to Interface Options > Camera and enable it
   ```

5. Enable I2C interface (if using I2C devices):
   ```bash
   sudo raspi-config
   # Navigate to Interface Options > I2C and enable it
   ```

## Usage

1. Start the server:
   ```bash
   python src/main.py
   ```

2. The server will:
   - Start video streaming on UDP port 5001
   - Start command server on TCP port 4001
   - Initialize motor and servo controllers

3. To stop the server, press Ctrl+C. The server will clean up resources properly.

## Hardware Connections

### Motor Control
- Left Motor
  - Forward: GPIO 5
  - Backward: GPIO 6
  - PWM: GPIO 17
- Right Motor
  - Forward: GPIO 22
  - Backward: GPIO 23
  - PWM: GPIO 24

### Servo Control
- PWM: GPIO 25

## Network Protocol

### Command Server (TCP:4001)
Commands are sent as JSON objects:
```json
{
    "type": "motor",
    "data": {
        "speed": 50
    }
}
```

Available commands:
- `motor`: Control motor speed (-100 to 100)
- `servo`: Control servo position (0-45 degrees)
- `emergency_stop`: Stop all motors and reset servo

### Video Stream (UDP:5001)
- Resolution: 640x480
- Frame rate: ~30 FPS
- JPEG compression with adaptive quality
- Frame sequence tracking

## Troubleshooting

1. **Camera Issues**
   - Check camera permissions
   - Verify camera index
   - Test camera in isolation

2. **Motor Control Issues**
   - Verify GPIO connections
   - Check power supply
   - Test motor driver

3. **Network Issues**
   - Check firewall settings
   - Verify port availability
   - Test network connectivity

## Development

1. Create a feature branch:
   ```bash
   git checkout -b feature/your-feature
   ```

2. Make changes and test:
   ```bash
   python src/main.py
   ```

3. Run tests:
   ```bash
   pytest tests/
   ```

4. Submit pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details. 