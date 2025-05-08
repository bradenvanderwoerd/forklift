"""
Command Protocol Specification for Forklift Control

This module defines the command protocol used for communication between the client and server.
All commands are sent as JSON objects over the WebSocket connection.

Command Types:
1. DRIVE - Movement commands (WASD)
2. MOTOR - Direct motor control
3. SERVO - Servo position control
4. STOP - Emergency stop and movement stop

Command Format:
{
    "type": str,      # Command type (DRIVE|MOTOR|SERVO|STOP)
    "action": str,    # Action (START|STOP|SET)
    "direction": str, # Direction (FORWARD|BACKWARD|LEFT|RIGHT|NONE)
    "speed": int,     # Speed value (0-100)
    "value": int,     # Raw value for MOTOR/SERVO (0-100 or degrees)
    "timestamp": int  # Unix timestamp in milliseconds
}

Examples:

1. Drive Forward at 75% speed:
{
    "type": "DRIVE",
    "action": "START",
    "direction": "FORWARD",
    "speed": 75,
    "timestamp": 1234567890
}

2. Stop Movement:
{
    "type": "DRIVE",
    "action": "STOP",
    "direction": "NONE",
    "speed": 0,
    "timestamp": 1234567890
}

3. Emergency Stop:
{
    "type": "STOP",
    "action": "STOP",
    "direction": "NONE",
    "speed": 0,
    "timestamp": 1234567890
}

4. Set Motor Speed:
{
    "type": "MOTOR",
    "action": "SET",
    "value": 75,  # 0-100
    "timestamp": 1234567890
}

5. Set Servo Position:
{
    "type": "SERVO",
    "action": "SET",
    "value": 45,  # degrees
    "timestamp": 1234567890
}

Notes:
- All speed values are 0-100
- Servo positions are in degrees
- Timestamps are used for command ordering and timeout detection
- The server should implement a timeout mechanism (e.g., 500ms) for drive commands
- Emergency stop should be processed immediately and override all other commands
"""

# Command Types
DRIVE = "DRIVE"
MOTOR = "MOTOR"
SERVO = "SERVO"
STOP = "STOP"

# Actions
START = "START"
STOP = "STOP"
SET = "SET"

# Directions
FORWARD = "FORWARD"
BACKWARD = "BACKWARD"
LEFT = "LEFT"
RIGHT = "RIGHT"
NONE = "NONE"

# Constants
MAX_SPEED = 100
MIN_SPEED = 0
DEFAULT_SPEED = 50
COMMAND_TIMEOUT_MS = 500  # Timeout for drive commands 