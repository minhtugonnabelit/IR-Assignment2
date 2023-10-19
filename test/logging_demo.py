""" 

@file       logging_demo.py 

 

This sample script is provided as a demo on using some more advanced features on python's logging 

module as well as using argparse to add arguments to your script. 

 

1. Run this script: 

 

python logging_demo.py 

 

2. Show debug messages with: 

 

python logging_demo.py -v 

 

3. Explore arguments by parsing an int, a string, and multiple floats 

 

python logging_demo.py -i 42 -s Hello! -f 1.2 3.14 10 

 

View the outputs 

 

@author:  jacob.vartanian@uts.edu.au 

@date:  September 2023 

"""

import logging

import argparse
import os


# The default log filename

LOG_FILE_NAME = "log.log"


# Here is how a log will format to a file

LOG_FORMAT_FILE = "%(asctime)s,%(levelname)s,%(filename)s,%(funcName)s,%(lineno)d,%(message)s"


# Here is how a log will format to the console

LOG_FORMAT_CONSOLE = "%(asctime)s\t%(levelname)s: %(message)s"


def load_args():

    parser = argparse.ArgumentParser(
        description="Program to demo logging and argparse", formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument('-v', '--verbose', action='store_true',
                        help="Enable debug outputs")

    parser.add_argument('-i', '--sample_int', type=int, default=None, dest='my_int',
                        help="Add an integer")

    parser.add_argument('-s', '--sample_string', type=str, default=None, dest='my_string',
                        help="Add a string")

    parser.add_argument('-f', '--many_floats', type=float, default=None, dest='my_floats', nargs='+',
                        help="Add a number of floating point arguments")

    return parser.parse_args()


def create_new_logfile(basename):

    # Create a directory where we want to save the log file
    current_dir = os.path.dirname(__file__)

    log_dir = os.path.join(current_dir, "logs")

    if not os.path.exists(log_dir):

        os.makedirs(log_dir)

    full_filename = os.path.join(log_dir, f"{basename}")

    return full_filename


def init_log(log_level):

    # First, lets get the root logger

    log = logging.getLogger('root')

    # Create a new log file and setup logging to a file

    log_file = create_new_logfile(LOG_FILE_NAME)

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

    log.setLevel(logging.DEBUG)

    log.addHandler(file_handler)

    log.addHandler(stream_handler)

    log.info(f"Saving logs to: {file_handler.baseFilename}")

    return log


def main(args):

    if args.verbose:

        log_level = logging.DEBUG

    else:

        log_level = logging.INFO

    log = init_log(log_level)

    log.debug("Here is an example DEBUG log message")

    log.info("Here is an example INFO log message")

    log.warning("Here is an example WARNING log message")

    log.error("Here is an example ERROR log message")

    log.critical("Here is an example CRITICAL log message")

    if args.my_int:

        log.info(f"Your int was {args.my_int}")

    if args.my_string:

        log.info(f"Your string was {args.my_string}")

    if args.my_floats:

        log.info(f"Your floats were {args.my_floats}")


if __name__ == "__main__":

    args = load_args()

    main(args)
