import logging
import os

import argparse

# The default log filename
LOG_FILE_NAME = "log.log"
LOG_FORMAT_FILE = "%(asctime)s,%(levelname)s,%(filename)s,%(funcName)s,%(lineno)d,%(message)s"
LOG_FORMAT_CONSOLE = "%(asctime)s\t%(levelname)s: %(message)s"

class Logger():

    def __init__(self, log_level):



        # First, lets get the root logger
        self.log = logging.getLogger('root')

        # Create a new log file and setup logging to a file
        log_file = self._create_new_logfile(LOG_FILE_NAME)
        file_handler = logging.FileHandler(log_file, mode='a')
        formatter_file = logging.Formatter(LOG_FORMAT_FILE)
        file_handler.setFormatter(formatter_file)
        file_handler.setLevel(logging.DEBUG)

        # Create a log file to go to the console
        stream_handler = logging.StreamHandler()
        formatter_console = logging.Formatter(LOG_FORMAT_CONSOLE)
        stream_handler.setFormatter(formatter_console)

        # Here you can optionally set the name of the log
        stream_handler.set_name("console_log")

        # Set the log level for the console output (based on what we passed into this function)
        stream_handler.setLevel(log_level)

        # Set the main logging level to debug so the file handler can support it. The stream handler (to console) will use the level set above.
        self.log.setLevel(logging.DEBUG)
        self.log.addHandler(file_handler)
        self.log.addHandler(stream_handler)
        self.log.info(f"Saving logs to: {file_handler.baseFilename}")

    def _create_new_logfile(self, basename):

        # Create a directory where we want to save the log file
        current_dir = os.path.dirname(__file__)
        log_dir = os.path.join(current_dir, "logs")

        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        full_filename = os.path.join(log_dir, f"{basename}")

        return full_filename
