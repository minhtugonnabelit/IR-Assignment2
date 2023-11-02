import serial
import time
import threading

class PhysicalEstop:
    """
    Set up guide: https://www.arduino.cc/en/Tutorial/BuiltInExamples/Button
    """

    DEBOUNCE_DELAY = 1 

    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        """
        Port:
            Linux: /dev/ttyUSB0
            Windows: COMx
        """
        self.triggered = False
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.ser.flush()

        self.thread = threading.Thread(target=self.read_loop)
        self.thread.start()

    def read_button(self):

        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode().strip()
            current_state = int(data)
            return current_state

    def read_loop(self):

        self.last_recognize = time.time()
        try:
            while True:
                button_state = self.read_button()
                # print("button_state:  ", button_state)

                # Deboucer
                if button_state == 1:
                    current_time = time.time()
                    if current_time - self.last_recognize > PhysicalEstop.DEBOUNCE_DELAY:
                        self.last_recognize = time.time()
                        print("recognize button")
                        self.triggered = True

        finally:
            self.close()

    def is_pressed(self):
        current_time = time.time()
        if current_time - self.last_recognize > PhysicalEstop.DEBOUNCE_DELAY:
            return False
        return self.triggered

    def close(self):
        self.ser.close()


if __name__ == "__main__":
    estop = PhysicalEstop()  # modify the port if needed

