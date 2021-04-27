# Built-in libraries
import threading
import time
from collections import deque

# External libraries (installed from pip)
from serial import Serial, SerialException


class SerialHandler:
    def __init__(self, buffer_size: int = 100000):
        self.__serial: Serial = None
        self.__deque_data = deque(maxlen=buffer_size)
        self.__deque_log = deque(maxlen=buffer_size)
        self.__serial_reading_thread: threading.Thread = None
        self.__serial_logging_thread: threading.Thread = None
        self.__threads_started = False

    def __del__(self):
        self.stop()
        if self.__serial is not None:
            self.__serial.close()

    def is_connected(self):
        return self.__serial is not None

    def start(self, port: str, baud: int, timeout_seconds: float, log_filename: str = None,
              log_show_timestamp: bool = False, log_suffix: str = None, log_extension: str = "log",
              display_characters_on_console=False, line_validation_function=None):
        """
            Starts serial connection along with internal threads.
            Optionally starts logging thread when `log_filename` is provided.
        """
        # Serial constructor can raise SerialException
        self.__serial = Serial(port=port, baudrate=baud, timeout=timeout_seconds)
        self.__serial.reset_input_buffer()
        self.__serial.reset_output_buffer()

        self.__serial_reading_thread = SerialReadingThread(self.__serial, self.__deque_data, deque_log=self.__deque_log)

        if log_filename:
            self.__serial_logging_thread = SerialLoggingThread(self.__deque_log, filename=log_filename,
                show_timestamp=log_show_timestamp, suffix=log_suffix, extension=log_extension,
                display_characters_on_console=display_characters_on_console, line_validation_function=line_validation_function)

        if not self.__threads_started:
            self.__threads_started = True
            if self.__serial_reading_thread:
                self.__serial_reading_thread.start()
            if self.__serial_logging_thread:
                self.__serial_logging_thread.start()

    def stop(self):
        """
            Stops internal serial threads.
        """
        if self.__threads_started:
            self.__threads_started = False
            if self.__serial_reading_thread:
                self.__serial_reading_thread.stop()
                self.__serial_reading_thread.join(1)
            if self.__serial_logging_thread:
                self.__serial_logging_thread.stop()
                self.__serial_logging_thread.join(1)

    def set_display_logged_characters(self, flag: bool):
        self.__serial_logging_thread.set_display_characters_on_console(flag)

    def get_next_invalid_line(self):
        return self.__serial_logging_thread.get_next_invalid_line()

    def send_string(self, string: str):
        """
            Sends string to serial.

            Parameters:
                string: String to send.

            Returns: bool: True is successful, False if couldn't send or `string` was None or `string` cannot be
            converted to bytes.
        """
        status = False
        if string:
            try:
                status = self.send_bytes(string.encode('ascii'))
            except UnicodeDecodeError:
                pass
        return status

    def send_bytes(self, data: bytes):
        """
            Sends bytes to serial.

            Parameters:
                data (bytes): Bytes to send.

            Returns:
                bool: True is successful, False if couldn't send or `data` was None.
        """
        if data:
            sent = 0
            try:
                sent = self.__serial.write(data)
                self.__serial.flush()
            except Exception:
                pass
            return sent > 0
        return False

    def pop_line_as_bytes(self, line_ending: bytes = b'\r\n'):
        """
            Pops single line read from serial.
            Line is detected by given line ending.
            Line bytes are removed from internal buffer.

            Parameters:
                line_ending (bytes): Bytes that make line ending.

            Returns: bytes: Bytes making the line including the line ending. If no line was detected in the buffer
            returns None.
        """
        # Find line ending
        data = self.peek_bytes()
        if not data:
            return None

        line_end_index = data.find(line_ending)
        if line_end_index < 0:
            # Line ending not found
            return None

        # Pop bytes up to the index including the line ending
        return self.pop_bytes(line_end_index + len(line_ending))

    def pop_bytes_all(self):
        """
            Pops all available bytes read from serial.
            Bytes are removed from internal buffer.

            Returns:
                bytes: Popped bytes. If no bytes are available returns None.
        """
        data = bytes()

        while self.__deque_data:
            try:
                # Keep popping and building data until exception
                data = data + self.__deque_data.popleft()
            except IndexError:
                # Deque is empty
                break

        if len(data) == 0:
            return None

        return data

    def pop_bytes(self, number_of_bytes: int):
        """
            Pops available bytes read from serial up to a certain number of bytes.
            Bytes are removed from internal buffer.

            Returns:
                bytes: Popped bytes. If no bytes are available returns None.
        """
        data = bytes()

        if number_of_bytes <= 0:
            return data

        while self.__deque_data:
            try:
                # Keep popping and building data until exception or number of bytes satisfied
                data = data + self.__deque_data.popleft()
            except IndexError:
                # Deque is empty
                break
            if len(data) == number_of_bytes:
                break

        if len(data) == 0:
            return None

        return data

    def peek_bytes(self, number_of_bytes: int = -1):
        """
            Peeks available bytes read from serial up to a certain number of bytes.
            Bytes are not removed from internal buffer.
            This mean the client will have to call another API that will consume the bytes from the buffer.

            Parameters:
                number_of_bytes: Number of bytes to peek. If -1 will peek all bytes.

            Returns:
                bytes: Peeked bytes. If no bytes are available returns None.
        """
        data = bytes()

        if (number_of_bytes == 0) or (number_of_bytes < -1):
            return None

        i: int = 0
        # Iterating over deque can cause problems since deque is not fully thread-safe
        # Use a loop and access elements by index to omit that concurrency problem
        while i < len(self.__deque_data):
            data = data + self.__deque_data[i]
            i = i + 1
            if number_of_bytes > 0:
                # If enough number of bytes was grabbed, exit the loop
                if len(data) >= number_of_bytes:
                    break

        if len(data) == 0:
            return None

        return data

    def get_number_of_bytes(self):
        """
            Returns the number of bytes read from serial.

            Returns:
                int: Number of bytes.
        """
        return len(self.__deque_data)


class SerialLoggingThread(threading.Thread):
    def __init__(self, logging_deque: deque, filename=None, show_timestamp=False, suffix=None, extension=".log",
                 display_characters_on_console=False, line_validation_function=None):
        self.__logging_deque = logging_deque
        self.__show_timestamp = show_timestamp
        self.__suffix = None
        if suffix is not None:
            self.__suffix = suffix.encode('ascii')
        self.__filename = filename
        self.__filename_details = None
        if filename is not None:
            self.__filename_details = {
                "filename": filename,
                "extension": extension
            }
        self.__display_characters_on_console = display_characters_on_console
        self.__is_stopped = threading.Event()
        self.__current_line = bytearray()
        self.__line_validation_function = line_validation_function
        self.__invalid_lines = deque()
        threading.Thread.__init__(self)

    def create_new_file(self, filename, extension):
        self.__filename = filename
        if self.__filename is not None:
            file_created = False
            i = 0
            while not file_created:
                if i > 0:
                    self.__filename = filename + '-' + str(i) + '.' + extension
                else:
                    self.__filename = filename + '.' + extension
                try:
                    open(self.__filename, 'x')
                    file_created = True
                except IOError:
                    file_created = False
                    i = i + 1

    def set_display_characters_on_console(self, flag: bool):
        self.__display_characters_on_console = flag
    
    def get_next_invalid_line(self):
        try:
            return self.__invalid_lines.popleft()
        except IndexError:
            return False

    def stop(self):
        self.__is_stopped.set()

    def run(self):
        if self.__filename_details:
            self.create_new_file(self.__filename_details["filename"], self.__filename_details["extension"])
        new_line_detected = False
        while not self.__is_stopped.is_set():
            time.sleep(0.1)
            if self.__logging_deque:
                bytes_to_log = bytearray()
                while self.__logging_deque:
                    next_stamped_data = self.__logging_deque.popleft()
                    if new_line_detected:
                        new_line_detected = False
                        if self.__show_timestamp:
                            timestamp = next_stamped_data["timestamp"]
                            bytes_to_log += b'%.3f,' % timestamp
                    bytes_to_log += next_stamped_data["data"]
                    self.__current_line += next_stamped_data["data"]
                    if next_stamped_data["data"] == b'\n':
                        new_line_detected = True
                        if self.__line_validation_function:
                            is_valid_line = self.__line_validation_function(self.__current_line)
                            if not is_valid_line:
                                self.__invalid_lines.append(self.__current_line)
                        self.__current_line = bytearray()
                    if self.__suffix is not None:
                        if self.__suffix == b'\n':
                            new_line_detected = True
                        bytes_to_log.append(self.__suffix)
                if bytes_to_log:
                    if self.__display_characters_on_console:
                        try:
                            print(bytes_to_log.decode('ascii'), end="")
                        except UnicodeDecodeError:
                            pass
                    with open(self.__filename, "ab") as fh:
                        fh.write(bytes_to_log)


class SerialReadingThread(threading.Thread):
    def __init__(self, serial: Serial, deque_data: deque, deque_log: deque = None):
        self.__serial = serial
        self.__deque_data = deque_data
        self.__deque_log = deque_log
        self.__is_stopped = threading.Event()
        threading.Thread.__init__(self)

    def stop(self):
        self.__is_stopped.set()

    def run(self):
        while not self.__is_stopped.is_set():
            try:
                # Reads one byte from serial and appends it into the deque
                # read() is blocking up to the timeout value passed to Serial
                byte = self.__serial.read()
                if byte:
                    self.__deque_data.append(byte)
                    if self.__deque_log is not None:
                        stamped_byte = {
                            "timestamp": time.time(),
                            "data": byte
                        }
                        self.__deque_log.append(stamped_byte)
            except SerialException:
                print("SerialReadingThread serial exception")
                break
