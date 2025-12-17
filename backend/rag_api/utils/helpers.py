"""Error handling and logging infrastructure."""
import logging
from logging.handlers import RotatingFileHandler
import os
from datetime import datetime


def setup_logging():
    """Set up logging infrastructure for the application."""
    # Create logs directory if it doesn't exist
    if not os.path.exists("logs"):
        os.makedirs("logs")
    
    # Create a custom logger
    logger = logging.getLogger("rag_chatbot")
    logger.setLevel(logging.DEBUG)
    
    # Create handlers
    c_handler = logging.StreamHandler()  # Console handler
    f_handler = RotatingFileHandler(
        f"logs/rag_chatbot_{datetime.now().strftime('%Y%m%d')}.log",
        maxBytes=1000000,  # 1MB
        backupCount=5
    )
    c_handler.setLevel(logging.INFO)
    f_handler.setLevel(logging.DEBUG)
    
    # Create formatters and add it to handlers
    c_format = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
    f_format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s')
    c_handler.setFormatter(c_format)
    f_handler.setFormatter(f_format)
    
    # Add handlers to the logger
    logger.addHandler(c_handler)
    logger.addHandler(f_handler)
    
    return logger


def log_error(error: Exception, context: str = ""):
    """Log an error with context information."""
    logger = logging.getLogger("rag_chatbot")
    logger.error(f"Error in {context}: {str(error)}", exc_info=True)


def log_info(message: str, context: str = ""):
    """Log an informational message."""
    logger = logging.getLogger("rag_chatbot")
    logger.info(f"[{context}] {message}")


def log_debug(message: str, context: str = ""):
    """Log a debug message."""
    logger = logging.getLogger("rag_chatbot")
    logger.debug(f"[{context}] {message}")


def log_warning(message: str, context: str = ""):
    """Log a warning message."""
    logger = logging.getLogger("rag_chatbot")
    logger.warning(f"[{context}] {message}")