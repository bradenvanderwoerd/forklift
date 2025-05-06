# Forklift

A remote-controlled car robot project using Raspberry Pi, featuring real-time video streaming, motor control, and servo control capabilities.

## Project Structure

```
forklift/
├── server/                 # Raspberry Pi server code
│   ├── src/
│   │   ├── controllers/    # Hardware controllers
│   │   ├── network/       # Network handling
│   │   ├── utils/         # Utility functions
│   │   └── main.py        # Server entry point
│   ├── requirements.txt
│   └── README.md
│
├── client/                # macOS client code
│   ├── src/
│   │   ├── ui/           # User interface
│   │   ├── network/      # Network handling
│   │   ├── utils/        # Utility functions
│   │   └── main.py       # Client entry point
│   ├── requirements.txt
│   └── README.md
│
└── shared/               # Shared code between client and server
    ├── protocol/        # Network protocol definitions
    └── utils/           # Shared utilities
```

## Development Setup

### On MacBook (Client Development)

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/forklift.git
   cd forklift
   ```

2. Set up client environment:
   ```bash
   cd client
   python -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

3. Run the client:
   ```bash
   python src/main.py
   ```

### On Raspberry Pi (Server Development)

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/forklift.git
   cd forklift
   ```

2. Set up server environment:
   ```bash
   cd server
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

3. Run the server:
   ```bash
   python src/main.py
   ```

## Development Workflow

1. **Development on MacBook**:
   - Make changes to both client and server code
   - Test client code locally
   - Commit and push changes to GitHub

2. **Testing on Raspberry Pi**:
   - Pull latest changes from GitHub
   - Test server code with actual hardware
   - Make any necessary adjustments
   - Push changes back to GitHub

## Contributing

1. Create a feature branch:
   ```bash
   git checkout -b feature/your-feature
   ```

2. Make your changes and commit:
   ```bash
   git add .
   git commit -m "Description of changes"
   ```

3. Push to GitHub:
   ```bash
   git push origin feature/your-feature
   ```

4. Create a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details. 