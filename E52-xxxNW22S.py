# -*- coding: utf-8 -*-

"""
 █████╗ ███████╗    ███████╗ ██████╗██████╗ ██╗   ██╗██████╗ ████████╗
██╔══██╗██╔════╝    ██╔════╝██╔════╝██╔══██╗╚██╗ ██╔╝██╔══██╗╚══██╔══╝
███████║███████╗    ███████╗██║     ██████╔╝ ╚████╔╝ ██████╔╝   ██║   
██╔══██║╚════██║    ╚════██║██║     ██╔══██╗  ╚██╔╝  ██╔═══╝    ██║   
██║  ██║███████║    ███████║╚██████╗██║  ██║   ██║   ██║        ██║   
╚═╝  ╚═╝╚══════╝    ╚══════╝ ╚═════╝╚═╝  ╚═╝   ╚═╝   ╚═╝        ╚═╝  

 LoRa UART Python Library for E52-xxxNW22S
    • Ability to send data types other than String
    • Splits Mixed Lines
    • Separates Asynchronous vs. Command Responses
    • Avoids "UserMessageSUCCESS" merges
    • Extracts only "SUCCESS" line for send confirmation

Author:      Abdullah SECKIN
Version:     1.2.0
License:     MIT License
Date:        20.03.2025
Dependencies:
    - pyserial
Compatibility:
    - Windows | Linux | macOS


# Example usage
>>> if __name__ == "__main__":
>>>     import time
>>>     logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
>>>     try:
>>>         lora = LoRaModule(port="/dev/cu.usbserial-2120", baudrate=115200, timeout=1, retries=3, log_enabled=True)
>>>         lora.reset()
>>>         # It is necessary to prevent confusion that may occur during separation.
>>>         lora.set_head(0)
>>>         def on_async(line):
>>>             print("Async message from module:", line)
>>>         lora.async_callback = on_async
>>>         resp_ch = lora.set_channel(12, 1)
>>>         print("Channel set response:", resp_ch)
>>>         resp_opt = lora.set_option(3, 1)
>>>         print("Option set response:", resp_opt)
>>>         message = ("3455").encode('ascii')
>>>         print(message)
>>>         while True:
>>>             resp = lora.send_message(message, expect="SUCCESS", timeout=3)
>>>             print("Send result:", resp)
>>>             time.sleep(0.2)
>>>     except Exception as e:
>>>         logging.error("Error: %s", e)
>>>     finally:
>>>         lora.close()

"""

import re
import serial
import threading
import logging
import time
from sys import getsizeof
import struct

class LoRaModule:
    """
    A Python library for interacting with the E52-xxxNW22S LoRa module via UART using AT commands,
    while carefully splitting mixed lines so asynchronous messages are never merged with
    command responses. For user data sends, only the line containing 'SUCCESS' is returned;
    all other lines received during that window are forwarded to the async callback.
    """

    def __init__(self, port="/dev/cu.usbserial-2110", baudrate=115200, timeout=1, retries=3, log_enabled=True):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.retries = retries
        self.log_enabled = log_enabled

        # Dedicated logger for this class.
        self.logger = logging.getLogger("LoRaModule")
        self.logger.disabled = not self.log_enabled

        # A lock to ensure only one command (or send operation) at a time.
        self._cmd_lock = threading.Lock()

        # Event + buffer used to collect lines belonging to the active command/send.
        self._response_event = threading.Event()
        self._response_lines = []

        # Flag indicating we have an active command or send in progress.
        self._waiting_for_cmd = False

        # Optional callback for truly asynchronous lines (not part of an active command).
        self.async_callback = None

        # Control flag for the continuous reader thread.
        self._running = True

        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            self.logger.info("Opened serial port %s at %d baud.", port, baudrate)
        except Exception as e:
            self.logger.error("Failed to open serial port %s: %s", port, str(e))
            raise

        # Launch background reader thread
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def _split_mixed_line(self, line):
        """
        If a single line from the module is something like:
            "Hello Module A!AT+OPTION=OK"
        we split it into separate chunks:
            ["Hello Module A!", "AT+OPTION=OK"]

        We also split on "OK", "SUCCESS", etc., so that lines like
            "Hello Module A!SUCCESS"
        become ["Hello Module A!", "SUCCESS"].

        Return a list of separate segments, each treated as if it came on its own line.
        """
        # We'll do repeated text replacements to insert a "<SPLIT>" marker
        # before "AT+", "OK", "SUCCESS" if they're not at the beginning,
        # then split on "<SPLIT>" to produce separate lines.

        def insert_marker(pattern, marker, txt):
            # Insert marker before pattern unless pattern is at start of text
            return re.sub(
                rf'(?<!^)({pattern})',
                f'{marker}\\1',
                txt,
                flags=re.IGNORECASE
            )

        text = line
        text = insert_marker("AT\\+", "<SPLIT>", text)
        text = insert_marker("OK", "<SPLIT>", text)
        text = insert_marker("SUCCESS", "<SPLIT>", text)

        parts = text.split("<SPLIT>")
        segments = [p.strip() for p in parts if p.strip()]

        return segments

    def _reader_loop(self):
        """
        Continuously read from the serial port, carefully splitting any "mixed" lines into segments.
        If we are waiting for a command, lines that pass our filter go into the command buffer,
        otherwise they go to async_callback.
        """
        while self._running:
            try:
                line = self.ser.readline()
                if not line:
                    continue

                decoded_line = line.decode('utf-8', errors='ignore').strip()
                # Split into separate segments if the line merges e.g. "Hello Module A!AT+OPTION=OK"
                segments = self._split_mixed_line(decoded_line)

                for seg in segments:
                    seg = seg.strip()
                    if not seg:
                        continue
                    if self._waiting_for_cmd:
                        # Temporarily store this line
                        self._response_lines.append(seg)
                        # Signal that new data arrived
                        self._response_event.set()
                    else:
                        # Asynchronous data
                        if self.async_callback:
                            self.async_callback(seg)
                        else:
                            self.logger.info("Async: %s", seg)

            except Exception as e:
                self.logger.error("Error in reader loop: %s", e)

    def _default_response_filter(self, line):
        """
        Decide if a received line belongs to the active command vs. being an async line.
        Typically, lines containing 'AT+', 'OK', '=' or 'SUCCESS' are command responses.
        """
        line_upper = line.upper()
        if "AT+" in line_upper:
            return True
        if "OK" in line_upper:
            return True
        if "SUCCESS" in line_upper:
            return True
        if "=" in line:
            return True
        return False

    def _format_command(self, command, *params, query=False, remote=False):
        """
        Build the AT command string.
        """
        prefix = "++AT+" if remote else "AT+"
        if query:
            return f"{prefix}{command}=?"
        elif params:
            param_str = ",".join(str(p) for p in params)
            return f"{prefix}{command}={param_str}"
        else:
            return f"{prefix}{command}"

    def _send_at_command(
        self,
        command_str,
        is_query=False,
        expected_response="OK",
        cmd_timeout=2,
        response_filter=None
    ):
        """
        Send an AT command and wait for its response. The lines read while waiting
        are tested with 'response_filter' to see if they belong to this command.
        If a line doesn't pass the filter, it's forwarded to async_callback.
        """
        if response_filter is None:
            response_filter = self._default_response_filter

        self.logger.debug("Sending command: %s", command_str)

        with self._cmd_lock:
            self.ser.reset_input_buffer()
            self._response_lines = []
            self._response_event.clear()
            self._waiting_for_cmd = True

            # Send command
            self.ser.write(command_str.encode('utf-8'))

            # Wait for incoming lines up to cmd_timeout
            end_time = time.time() + cmd_timeout
            while time.time() < end_time:
                self._response_event.wait(0.2)
                self._response_event.clear()

            # Mark command as done
            self._waiting_for_cmd = False

            # Filter out lines that are truly for this command
            valid_for_cmd = []
            async_lines = []
            for ln in self._response_lines:
                if response_filter(ln):
                    valid_for_cmd.append(ln)
                else:
                    async_lines.append(ln)

            # If any lines are obviously async, push them out
            if async_lines and self.async_callback:
                for a_ln in async_lines:
                    self.async_callback(a_ln)

            response = "\n".join(valid_for_cmd)
            if not is_query and expected_response:
                # For set/command operations, ensure we got the expected response
                if expected_response.upper() not in response.upper():
                    raise Exception(
                        f"Unexpected response for cmd '{command_str}': {response}"
                    )

            return response

    # ----------------------------------------------------------------------
    # Sending user data (i.e., not AT commands)
    # ----------------------------------------------------------------------
    def send_message(self, message, expect="SUCCESS", timeout=5):
        """
        Send a user data message (plain text). Wait for 'expect' (e.g. "SUCCESS").
        After the read loop finishes, we parse lines:
          • If a line contains the expect token, that's the "send result".
          • All other lines are treated as asynchronous data and passed to async_callback.

        If no line has the expect token, we raise an Exception.
        """
        with self._cmd_lock:
            self.ser.reset_input_buffer()
            self._response_lines = []
            self._response_event.clear()
            self._waiting_for_cmd = True

            if isinstance(message, str):
                self.ser.write(message.encode('utf-8'))
            else:
                self.ser.write(message)
            end_time = time.time() + timeout
            while time.time() < end_time:
                self._response_event.wait(0.2)
                self._response_event.clear()

            self._waiting_for_cmd = False

            expect_upper = expect.upper() if expect else ""
            success_line = None
            async_lines = []

            # Check each collected line; if it has the success token, we store it as success
            # otherwise, we treat it as async data
            for ln in self._response_lines:
                if expect_upper and expect_upper in ln.upper():
                    success_line = ln
                else:
                    async_lines.append(ln)

            # Send async lines to the callback
            if async_lines and self.async_callback:
                for a_ln in async_lines:
                    self.async_callback(a_ln)

            # If no line contained the success token, raise an error
            if not success_line and expect_upper:
                raise Exception(
                    f"send_message failed: no '{expect}' token found in:\n"
                    + "\n".join(self._response_lines)
                )

            # Return whichever line had the success token
            return success_line if success_line else ""

    def send_message_loop(self, messages, delay=1, expect="SUCCESS", timeout=5):
        """
        Send multiple user-data messages in a loop.
        Return the list of responses or errors.
        """
        results = []
        for msg in messages:
            try:
                resp = self.send_message(msg, expect=expect, timeout=timeout)
                results.append(resp)
            except Exception as e:
                results.append(f"Error sending '{msg}': {e}")
            time.sleep(delay)
        return results

    # ----------------------------------------------------------------------
    # AT COMMANDS
    # ----------------------------------------------------------------------
    def reset(self):
        return self._send_at_command(self._format_command("RESET"), is_query=False)

    def default_settings(self):
        return self._send_at_command(self._format_command("DEFAULT"), is_query=False)

    def iap(self):
        return self._send_at_command(self._format_command("IAP"), is_query=False)

    def get_info(self):
        return self._send_at_command(self._format_command("INFO", query=True), is_query=True)

    def get_dev_type(self):
        return self._send_at_command(self._format_command("DEVTYPE", query=True), is_query=True)

    def get_fwcode(self):
        return self._send_at_command(self._format_command("FWCODE", query=True), is_query=True)

    def get_power(self):
        return self._send_at_command(self._format_command("POWER", query=True), is_query=True)

    def set_power(self, power, save=0):
        return self._send_at_command(self._format_command("POWER", power, save), is_query=False)

    def get_channel(self):
        return self._send_at_command(self._format_command("CHANNEL", query=True), is_query=True)

    def set_channel(self, channel, save=0):
        return self._send_at_command(self._format_command("CHANNEL", channel, save), is_query=False)

    def get_uart(self):
        return self._send_at_command(self._format_command("UART", query=True), is_query=True)

    def set_uart(self, baud, parity):
        return self._send_at_command(self._format_command("UART", baud, parity), is_query=False)

    def get_rate(self):
        return self._send_at_command(self._format_command("RATE", query=True), is_query=True)

    def set_rate(self, rate):
        return self._send_at_command(self._format_command("RATE", rate), is_query=False)

    def get_option(self):
        return self._send_at_command(self._format_command("OPTION", query=True), is_query=True)

    def set_option(self, option, save=0):
        return self._send_at_command(self._format_command("OPTION", option, save), is_query=False)

    def get_panid(self):
        return self._send_at_command(self._format_command("PANID", query=True), is_query=True)

    def set_panid(self, panid, save=0):
        return self._send_at_command(self._format_command("PANID", panid, save), is_query=False)

    def get_type(self):
        return self._send_at_command(self._format_command("TYPE", query=True), is_query=True)

    def set_type(self, node_type):
        return self._send_at_command(self._format_command("TYPE", node_type), is_query=False)

    def get_src_addr(self):
        return self._send_at_command(self._format_command("SRC_ADDR", query=True), is_query=True)

    def set_src_addr(self, addr, save=0):
        return self._send_at_command(self._format_command("SRC_ADDR", addr, save), is_query=False)

    def get_dst_addr(self):
        return self._send_at_command(self._format_command("DST_ADDR", query=True), is_query=True)

    def set_dst_addr(self, addr, save=0):
        return self._send_at_command(self._format_command("DST_ADDR", addr, save), is_query=False)

    def get_src_port(self):
        return self._send_at_command(self._format_command("SRC_PORT", query=True), is_query=True)

    def set_src_port(self, port, save=0):
        return self._send_at_command(self._format_command("SRC_PORT", port, save), is_query=False)

    def get_dst_port(self):
        return self._send_at_command(self._format_command("DST_PORT", query=True), is_query=True)

    def set_dst_port(self, port, save=0):
        return self._send_at_command(self._format_command("DST_PORT", port, save), is_query=False)

    def get_member_rad(self):
        return self._send_at_command(self._format_command("MEMBER_RAD", query=True), is_query=True)

    def set_member_rad(self, rad, save=0):
        return self._send_at_command(self._format_command("MEMBER_RAD", rad, save), is_query=False)

    def get_nonmember_rad(self):
        return self._send_at_command(self._format_command("NONMEMBER_RAD", query=True), is_query=True)

    def set_nonmember_rad(self, rad, save=0):
        return self._send_at_command(self._format_command("NONMEMBER_RAD", rad, save), is_query=False)

    def get_csma_rng(self):
        return self._send_at_command(self._format_command("CSMA_RNG", query=True), is_query=True)

    def set_csma_rng(self, rng):
        return self._send_at_command(self._format_command("CSMA_RNG", rng), is_query=False)

    def get_router_score(self):
        return self._send_at_command(self._format_command("ROUTER_SCORE", query=True), is_query=True)

    def set_router_score(self, score):
        return self._send_at_command(self._format_command("ROUTER_SCORE", score), is_query=False)

    def get_head(self):
        return self._send_at_command(self._format_command("HEAD", query=True), is_query=True)

    def set_head(self, enable):
        return self._send_at_command(self._format_command("HEAD", enable), is_query=False)

    def get_back(self):
        return self._send_at_command(self._format_command("BACK", query=True), is_query=True)

    def set_back(self, enable):
        return self._send_at_command(self._format_command("BACK", enable), is_query=False)

    def get_security(self):
        return self._send_at_command(self._format_command("SECURITY", query=True), is_query=True)

    def set_security(self, enable):
        return self._send_at_command(self._format_command("SECURITY", enable), is_query=False)

    def get_reset_aux(self):
        return self._send_at_command(self._format_command("RESET_AUX", query=True), is_query=True)

    def set_reset_aux(self, enable):
        return self._send_at_command(self._format_command("RESET_AUX", enable), is_query=False)

    def get_reset_time(self):
        return self._send_at_command(self._format_command("RESET_TIME", query=True), is_query=True)

    def set_reset_time(self, reset_time):
        return self._send_at_command(self._format_command("RESET_TIME", reset_time), is_query=False)

    def get_filter_time(self):
        return self._send_at_command(self._format_command("FILTER_TIME", query=True), is_query=True)

    def set_filter_time(self, time_val):
        return self._send_at_command(self._format_command("FILTER_TIME", time_val), is_query=False)

    def get_ack_time(self):
        return self._send_at_command(self._format_command("ACK_TIME", query=True), is_query=True)

    def set_ack_time(self, time_val):
        return self._send_at_command(self._format_command("ACK_TIME", time_val), is_query=False)

    def get_router_time(self):
        return self._send_at_command(self._format_command("ROUTER_TIME", query=True), is_query=True)

    def set_router_time(self, time_val):
        return self._send_at_command(self._format_command("ROUTER_TIME", time_val), is_query=False)

    def group_add(self, group):
        return self._send_at_command(self._format_command("GROUP_ADD", group), is_query=False)

    def group_del(self, group):
        return self._send_at_command(self._format_command("GROUP_DEL", group), is_query=False)

    def group_clear(self, enable):
        return self._send_at_command(self._format_command("GROUP_CLR", enable), is_query=False)

    def router_clear(self, enable):
        return self._send_at_command(self._format_command("ROUTER_CLR", enable), is_query=False)

    def router_save(self, enable):
        return self._send_at_command(self._format_command("ROUTER_SAVE", enable), is_query=False)

    def router_read(self, enable):
        return self._send_at_command(self._format_command("ROUTER_READ", enable), is_query=False)

    def get_mac(self):
        return self._send_at_command(self._format_command("MAC", query=True), is_query=True)

    def get_key(self):
        return self._send_at_command(self._format_command("KEY", query=True), is_query=True)

    def set_key(self, key):
        return self._send_at_command(self._format_command("KEY", key), is_query=False)

    def close(self):
        """Stop the reader thread and close the serial port."""
        self._running = False
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1)
        if self.ser.is_open:
            self.ser.close()
        self.logger.info("Serial port %s closed.", self.port)



