# Raspberry Pi Car Robot Project Documentation

## Overview
This project implements a remote-controlled car robot using a Raspberry Pi, featuring real-time video streaming, motor control, and servo control capabilities. The system uses a client-server architecture where the Raspberry Pi acts as the server, and a macOS computer acts as the client.

## Project Structure
```
forklift/
├── server/                 # Raspberry Pi server code
│   ├── src/
│   │   ├── controllers/    # Hardware controllers
│   │   │   ├── motor.py
│   │   │   ├── servo.py
│   │   │   └── camera.py
│   │   ├── network/       # Network handling
│   │   │   ├── tcp_server.py
│   │   │   └── video_stream.py
│   │   ├── utils/         # Utility functions
│   │   │   ├── logger.py
│   │   │   └── config.py
│   │   └── main.py        # Server entry point
│   ├── requirements.txt
│   └── README.md
│
├── client/                # macOS client code
│   ├── src/
│   │   ├── ui/           # User interface
│   │   │   ├── main_window.py
│   │   │   ├── video_display.py
│   │   │   └── control_panel.py
│   │   ├── network/      # Network handling
│   │   │   ├── tcp_client.py
│   │   │   └── video_receiver.py
│   │   ├── utils/        # Utility functions
│   │   │   ├── logger.py
│   │   │   └── config.py
│   │   └── main.py       # Client entry point
│   ├── requirements.txt
│   └── README.md
│
└── shared/               # Shared code between client and server
    ├── protocol/        # Network protocol definitions
    │   ├── commands.py
    │   └── messages.py
    └── utils/           # Shared utilities
        └── constants.py
```

## Hardware Requirements

### Core Components
1. Raspberry Pi (4 recommended)
2. Camera module (compatible with Raspberry Pi)
3. Motor driver board (e.g., L298N)
4. DC motors (2x for movement)
5. Servo motor (for camera tilt)
6. Power supply (appropriate for motors)
7. Jumper wires
8. Chassis and wheels

### GPIO Pin Configuration (BCM numbering)

#### Motor Control
- **Left Motor**
  - Forward: GPIO 5
  - Backward: GPIO 6
  - PWM: GPIO 17

- **Right Motor**
  - Forward: GPIO 22
  - Backward: GPIO 23
  - PWM: GPIO 24

#### Servo Control
- PWM: GPIO 25

## Software Architecture

### Server-Side (Raspberry Pi)

#### 1. Motor Control
The `MotorController` class handles all motor operations with PID-based speed control:

```python
class MotorController:
    def __init__(self):
        self.setup_gpio()
        self.current_speed = 0
        self.pid_controller = PIDController()
        
    def set_speed(self, speed: float):
        # PID-based speed control
        adjusted_speed = self.pid_controller.compute(speed)
        self.apply_speed(adjusted_speed)
```

Key features:
- PID-based speed control for smooth operation
- Independent motor control
- Safety features (automatic stop on errors)
- Current sensing for motor protection
- Proper GPIO cleanup

#### 2. Servo Control
The `ServoController` class manages the servo with position tracking:

```python
class ServoController:
    def __init__(self, pin=25):
        self.pin = pin
        self.current_position = 0
        self.target_position = 0
        self.step_size = 5  # Degrees per step
        
        # PWM setup
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 50)  # 50 Hz
```

Features:
- Position control (0-45 degrees)
- Step-based movement
- Position tracking
- Smooth operation

#### 3. Video Streaming
The server implements UDP-based video streaming with adaptive quality:

```python
class VideoStreamer:
    def __init__(self):
        self.quality_controller = AdaptiveQualityController()
        self.frame_sequencer = FrameSequencer()
        
    def stream_video(self):
        while True:
            frame = self.capture_frame()
            quality = self.quality_controller.get_optimal_quality()
            frame_data = self.compress_frame(frame, quality)
            self.send_frame(frame_data)
```

Features:
- Adaptive quality based on network conditions
- Frame sequence tracking
- Chunked transmission
- Error recovery

#### 4. Command Processing
The server handles commands through TCP with a command queue:

```python
class CommandServer:
    def __init__(self):
        self.command_queue = CommandQueue()
        self.heartbeat_timer = HeartbeatTimer()
        
    def handle_command(self, command: Command):
        self.command_queue.add(command)
        self.process_next_command()
```

Features:
- Command queue for reliable execution
- Heartbeat mechanism
- Command validation
- Error handling

### Client-Side (macOS)

#### 1. Video Display
- Real-time video streaming
- Frame statistics (FPS, size, quality)
- Adaptive quality display
- Error recovery

#### 2. Control Interface
- Keyboard-based control
- Speed adjustment
- Servo control
- Emergency stop
- Connection status monitoring

## Network Communication

### TCP Channel (Control)
- Port: 4001 (default)
- Features:
  - Command queue
  - Heartbeat mechanism
  - Command acknowledgment
  - Error recovery

### UDP Channel (Video)
- Port: 5001 (default)
- Features:
  - Adaptive quality streaming
  - Frame sequence tracking
  - Chunked transmission
  - Error handling
  - Bandwidth optimization

## Development Workflow

### 1. Setup Phase
1. Clone the repository
2. Set up development environment:
   ```bash
   # Server (Raspberry Pi)
   cd server
   python -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   
   # Client (macOS)
   cd client
   python -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

### 2. Development Process
1. **Feature Development**
   - Create feature branch from `main`
   - Implement feature
   - Write tests
   - Create pull request

2. **Testing**
   - Unit tests for each component
   - Integration tests for client-server communication
   - Hardware tests for motor and servo control
   - Network tests for video streaming

3. **Code Review**
   - Review pull requests
   - Check code quality
   - Verify test coverage
   - Ensure documentation is updated

4. **Deployment**
   - Merge to `main` branch
   - Tag release version
   - Update documentation
   - Deploy to Raspberry Pi

### 3. Testing Workflow
1. **Unit Testing**
   ```bash
   # Server
   cd server
   pytest tests/unit/
   
   # Client
   cd client
   pytest tests/unit/
   ```

2. **Integration Testing**
   ```bash
   # Run integration tests
   pytest tests/integration/
   ```

3. **Hardware Testing**
   - Test motor control
   - Test servo movement
   - Test video streaming
   - Test network communication

### 4. Deployment Workflow
1. **Server Deployment**
   ```bash
   # On Raspberry Pi
   git pull origin main
   pip install -r requirements.txt
   sudo systemctl restart forklift-server
   ```

2. **Client Deployment**
   ```bash
   # On macOS
   git pull origin main
   pip install -r requirements.txt
   ```

## Best Practices

1. **Code Quality**
   - Follow PEP 8 style guide
   - Write docstrings for all functions
   - Use type hints
   - Write unit tests

2. **Version Control**
   - Use meaningful commit messages
   - Create feature branches
   - Review code before merging
   - Keep commits atomic

3. **Documentation**
   - Update documentation with code changes
   - Document hardware setup
   - Maintain troubleshooting guide
   - Keep README up to date

4. **Testing**
   - Write tests for new features
   - Maintain test coverage
   - Test on actual hardware
   - Document test procedures

## Troubleshooting Guide

### Common Issues

1. **Camera Issues**
   - Check camera permissions
   - Verify camera index
   - Test camera in isolation
   - Check video stream quality

2. **Motor Control Issues**
   - Verify GPIO connections
   - Check power supply
   - Test motor driver
   - Monitor current draw

3. **Network Issues**
   - Check firewall settings
   - Verify port availability
   - Test network connectivity
   - Monitor bandwidth usage

4. **Performance Issues**
   - Monitor CPU usage
   - Check memory usage
   - Optimize video quality
   - Adjust frame rate

## Future Improvements (ignore for now)

1. **Hardware**
   - Add distance sensors
   - Implement battery monitoring
   - Add LED indicators
   - Improve motor control

2. **Software**
   - Add autonomous features
   - Implement path planning
   - Add obstacle avoidance
   - Improve video quality

3. **Network**
   - Add encryption
   - Implement authentication
   - Improve error handling
   - Add bandwidth control 