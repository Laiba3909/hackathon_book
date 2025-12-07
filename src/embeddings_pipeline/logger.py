# src/embeddings_pipeline/logger.py

import logging
import os
from datetime import datetime

LOG_DIR = "logs"
LOG_FILE_PREFIX = "embeddings_pipeline"

# Ensure log directory exists
os.makedirs(LOG_DIR, exist_ok=True)

def setup_logging(name=__name__, level=logging.INFO):
    """
    Sets up a custom logger for the embeddings pipeline.
    Logs to both console and a time-stamped file.
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.propagate = False # Prevent messages from being passed to the root logger

    # Create formatters
    console_formatter = logging.Formatter('%(levelname)s: %(message)s')
    file_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    # Console Handler
    if not any(isinstance(handler, logging.StreamHandler) for handler in logger.handlers):
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(console_formatter)
        logger.addHandler(console_handler)

    # File Handler
    log_filename = f"{LOG_FILE_PREFIX}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    log_filepath = os.path.join(LOG_DIR, log_filename)
    
    if not any(isinstance(handler, logging.FileHandler) for handler in logger.handlers):
        file_handler = logging.FileHandler(log_filepath)
        file_handler.setFormatter(file_formatter)
        logger.addHandler(file_handler)

    return logger

# Example usage
if __name__ == "__main__":
    logger = setup_logging(__name__, logging.DEBUG)
    logger.debug("This is a debug message.")
    logger.info("This is an info message.")
    logger.warning("This is a warning message.")
    logger.error("This is an error message.")
    logger.critical("This is a critical message.")

    try:
        raise ValueError("Something went wrong!")
    except ValueError as e:
        logger.exception("An exception occurred:")
