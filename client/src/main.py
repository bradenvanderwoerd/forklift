import sys
import os
import logging
from colorama import Fore, Style, init as colorama_init

# --- Python Path Modification ---
# Add the project's root directory (one level up from 'client') to the Python path.
# This allows for absolute imports from the 'src' directory (e.g., 'from src.ui...').
# Assumes this script is in /path/to/project/client/src/main.py
# and we want to add /path/to/project/ to sys.path
project_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.append(project_root)

# --- Application Imports ---
from PyQt6.QtWidgets import QApplication
from client.src.ui.main_window import MainWindow # Corrected import path

# Initialize colorama for colored console output.
# autoreset=True ensures that color changes are reset after each print statement.
colorama_init(autoreset=True)

# --- Custom Log Formatter ---
class ColorFormatter(logging.Formatter):
    """
    A custom logging formatter that adds color to log messages based on their level.
    This enhances readability of console output, especially for warnings and errors.
    """
    COLORS = {
        logging.DEBUG: Fore.CYAN,       # Debug messages in Cyan
        logging.INFO: Fore.GREEN,       # Info messages in Green
        logging.WARNING: Fore.YELLOW,     # Warning messages in Yellow
        logging.ERROR: Fore.RED,        # Error messages in Red
        logging.CRITICAL: Fore.MAGENTA + Style.BRIGHT, # Critical messages in Bright Magenta
    }

    def format(self, record: logging.LogRecord) -> str:
        """
        Formats the log record with appropriate colors.

        Args:
            record: The log record to format.

        Returns:
            The formatted, colorized log message string.
        """
        color = self.COLORS.get(record.levelno, "")  # Default to no color if level not mapped
        message = super().format(record)
        return f"{color}{message}{Style.RESET_ALL}"

# --- Logging Setup ---
# Configure the root logger to use the custom color formatter.
# All log messages will be sent to the console (StreamHandler).

# Create a handler that outputs to the console.
handler = logging.StreamHandler()
# Set the custom formatter for this handler.
handler.setFormatter(ColorFormatter("%(asctime)s - %(name)s - %(levelname)s: %(message)s"))

# Get the root logger and remove any existing handlers.
root_logger = logging.getLogger()
root_logger.handlers = [handler]
# Set the default logging level for the application (e.g., INFO, DEBUG).
root_logger.setLevel(logging.INFO)

# Create a logger specific to this module.
logger = logging.getLogger(__name__)

# --- Main Application Entry Point ---
def main():
    """
    Main function to initialize and run the PyQt6 application.
    It sets up the QApplication, creates the MainWindow, shows it,
    and starts the application's event loop.
    """
    logger.info("Starting Forklift Control Client application...")
    app = QApplication(sys.argv)  # Create the application instance.

    logger.info("Initializing Main Window...")
    window = MainWindow()         # Create the main UI window.
    window.show()                 # Display the main window.

    logger.info("Application event loop started.")
    # Start the Qt event loop. sys.exit() ensures a clean exit code is returned.
    exit_code = app.exec()
    logger.info(f"Application exited with code {exit_code}.")
    sys.exit(exit_code)

# --- Script Execution Guard ---
if __name__ == "__main__":
    # This ensures that main() is called only when the script is executed directly,
    # not when it's imported as a module.
    main() 