# -*- coding: utf-8 -*-



"""
 █████╗ ███████╗    ███████╗ ██████╗██████╗ ██╗   ██╗██████╗ ████████╗
██╔══██╗██╔════╝    ██╔════╝██╔════╝██╔══██╗╚██╗ ██╔╝██╔══██╗╚══██╔══╝
███████║███████╗    ███████╗██║     ██████╔╝ ╚████╔╝ ██████╔╝   ██║   
██╔══██║╚════██║    ╚════██║██║     ██╔══██╗  ╚██╔╝  ██╔═══╝    ██║   
██║  ██║███████║    ███████║╚██████╗██║  ██║   ██║   ██║        ██║   
╚═╝  ╚═╝╚══════╝    ╚══════╝ ╚═════╝╚═╝  ╚═╝   ╚═╝   ╚═╝        ╚═╝  
                                                            
            LoRa UART Python Library for E52-xxxNW22S
                  Robust • Reliable • Easy-to-use

Author:      Abdullah SECKIN
Version:     1.0.0
License:     MIT License
Date:        19.03.2025
Dependencies:
    - pyserial
Compatibility:
    - Windows | Linux | macOS

Description:
    This library provides an easy-to-use Python interface for the E52-xxxNW22S 
    LoRa modules via UART using AT commands. It supports complete module configuration, 
    messaging functionality, and handles asynchronous data with robust threading 
    and filtering techniques.

Example Usage:

    >>> if __name__ == "__main__":
    >>> from E52-xxxNW22S import LoRaModule
    >>> # Configure detailed logging.
    >>> logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
    >>> try:
    >>>     # Create an instance (adjust port as necessary)
    >>>     lora = LoRaModule(port="/dev/cu.usbserial-2120", baudrate=115200, timeout=1, retries=3, log_enabled=True)
    >>>     # Set the channel (this should return something like "AT+CHANNEL=OK")
    >>>     #set_channel_resp = lora.set_channel(13, 1)
    >>>     #print("Set Channel Response:\n",set_channel_resp)
    >>>     # Set the otion as broadcast (this should return something like "AT+OPTION=OK")
    >>>     set_option_resp = lora.set_option(3, 1)
    >>>     print("Set Option Response:\n",set_option_resp)    
    >>>     # Query module info (this returns the key parameters without an "OK")
    >>>     info_resp = lora.get_info()
    >>>     print("Module Info:\n",info_resp)    
    >>>     # Query the current channel (example: returns "AT+CHANNEL=0x0d,13")
    >>>     channel_resp = lora.get_channel()
    >>>     print("Channel Query Response:\n", channel_resp) 
    >>>     def handle_async_message(message):
    >>>         print("Received async message:", message)
    >>>     #Start Massage Read Loop
    >>>     lora.async_callback = handle_async_message
    >>>     while True:
    >>>         single_response = lora.send_message("Hello Module B!")
    >>>         print("Single send response:", single_response)
    >>>         time.sleep(1)  
    >>>     # Close the connection when done.
    >>>     #lora.close()  
    >>> except Exception as e:
    >>>     logging.error("Error during LoRaModule operation: %s", e)

"""



import serial
import threading
import logging
import time

class LoRaModule:
    """
    A Python library for interacting with the E52-xxxNW22S LoRa module via UART using AT commands.
    
    This version distinguishes between query responses (which return key/value info)
    and set/command responses (which return "OK"). It also creates a dedicated reader thread
    to capture command responses separately from asynchronous messages.
    
    Example:
        lora = LoRaModule(port="/dev/cu.usbserial-2110", baudrate=115200, timeout=1)
        info = lora.get_info()
        print("Module Info:\n", info)
        lora.close()
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

        # Lock to ensure only one command is sent at a time.
        self._cmd_lock = threading.Lock()
        # Event and buffer used to capture responses for commands.
        self._response_event = threading.Event()
        self._response_lines = []
        # Flag indicating that a command is currently waiting for a response.
        self._waiting_for_cmd = False
        # Optional callback for asynchronous messages (those not part of a command response)
        self.async_callback = None
        # Control flag for the reader thread.
        self._running = True

        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            self.logger.info("Opened serial port %s at %d baud.", port, baudrate)
        except Exception as e:
            self.logger.error("Failed to open serial port %s: %s", port, str(e))
            raise

        # Start the reader thread.
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def _reader_loop(self):
        """
        Continuously read lines from the UART.
        If a command is active, append lines to the response buffer;
        otherwise, treat them as asynchronous messages.
        """
        while self._running:
            try:
                line = self.ser.readline()
                if not line:
                    continue
                # Decode the received bytes.
                line = line.decode('utf-8', errors='ignore').strip()
                if self._waiting_for_cmd:
                    self._response_lines.append(line)
                    self._response_event.set()
                else:
                    # This is an asynchronous message from the channel.
                    if self.async_callback:
                        self.async_callback(line)
                    else:
                        self.logger.info("Async message: %s", line)
            except Exception as e:
                self.logger.error("Error in reader loop: %s", e)
    def _default_response_filter(self, line):
        """
        Default filter to decide if a received line belongs to a command response.
        Here we assume that valid responses contain "OK", "=" or start with "AT+".
        Adjust as needed.
        """
        return ("OK" in line) or ("=" in line) or line.startswith("AT+")
    def send_message(self, message, expect="SUCCESS", timeout=5):
        """
        Send raw user data (message) to the other module.
        The message is sent directly over the UART. The method then waits for the
        expected response (e.g., "SUCCESS") from the module. If the expected response
        is not received within the timeout period, an exception is raised.

        :param message: The message string to send.
        :param expect: Substring expected in the response (default "SUCCESS").
        :param timeout: Maximum time in seconds to wait for the response.
        :return: The response string if the expected substring is found.
        :raises Exception: if the expected response is not received.
        """
        # Use the same command lock to ensure exclusive access
        with self._cmd_lock:
            self.ser.reset_input_buffer()
            self._response_lines = []
            self._response_event.clear()
            self._waiting_for_cmd = True
            self.ser.write(message.encode('utf-8'))
            
            end_time = time.time() + timeout
            while time.time() < end_time:
                self._response_event.wait(0.2)
                self._response_event.clear()
                for line in self._response_lines:
                    if expect in line:
                        self._waiting_for_cmd = False
                        return line
            self._waiting_for_cmd = False
            raise Exception(f"Message sending failed: expected '{expect}' not received within {timeout} seconds.")

    def send_message_loop(self, messages, delay=1, expect="SUCCESS", timeout=5):
        """
        Send a sequence of messages in a loop. Each message is sent using send_message.
        
        :param messages: Iterable of message strings to be sent.
        :param delay: Delay in seconds between each send.
        :param expect: Expected response substring for each send.
        :param timeout: Timeout in seconds for waiting for each response.
        :return: A list of responses (or error messages) for each message sent.
        """
        responses = []
        for msg in messages:
            try:
                resp = self.send_message(msg, expect=expect, timeout=timeout)
                responses.append(resp)
            except Exception as e:
                responses.append(f"Error: {e}")
            time.sleep(delay)
        return responses
    def _format_command(self, command, *params, query=False, remote=False):
        """
        Build the AT command string.
        
        :param command: Base command (without the "AT+" prefix)
        :param params: Optional parameters (if any)
        :param query: If True, the command is a query (ends with "=?")
        :param remote: If True, prefix with "++AT+" for remote configuration
        :return: Formatted command string.
        """
        prefix = "++AT+" if remote else "AT+"
        if query:
            return f"{prefix}{command}=?"
        elif params:
            param_str = ",".join(str(p) for p in params)
            return f"{prefix}{command}={param_str}"
        else:
            return f"{prefix}{command}"

    def _send_at_command(self, command_str, is_query=False, expected_response="OK", cmd_timeout=2, response_filter=None):
        """
        Send an AT command and wait for its response. Lines that do not pass the response_filter
        are treated as asynchronous and forwarded via the async_callback.
        
        :param command_str: The AT command string to send.
        :param is_query: True for query commands.
        :param expected_response: For set commands, a substring that must be present.
        :param cmd_timeout: How long to wait for responses.
        :param response_filter: A function taking a line and returning True if it is part of the command response.
                                If None, a default filter is used.
        :return: A string containing the filtered response.
        :raises Exception: If for a set command the expected_response is not found.
        """
        if response_filter is None:
            response_filter = self._default_response_filter

        self.logger.debug("Sending command: %s", command_str)
        with self._cmd_lock:
            self.ser.reset_input_buffer()
            self._response_lines = []
            self._response_event.clear()
            self._waiting_for_cmd = True
            self.ser.write(command_str.encode('utf-8'))
            
            end_time = time.time() + cmd_timeout
            while time.time() < end_time:
                self._response_event.wait(0.2)
                self._response_event.clear()
            self._waiting_for_cmd = False
            
            # Separate valid command responses from asynchronous messages.
            valid_lines = []
            extra_async = []
            for line in self._response_lines:
                if response_filter(line):
                    valid_lines.append(line)
                else:
                    extra_async.append(line)
            # If there are extra lines, pass them to the async callback.
            if extra_async and self.async_callback:
                for line in extra_async:
                    self.async_callback(line)
            
            response = "\n".join(valid_lines)
            if not is_query and expected_response and expected_response not in response:
                raise Exception(f"Unexpected response for command '{command_str}': {response}")
            return response


    # AT Command Methods (set and query operations)

    def reset(self):
        """Send AT+RESET to restart the module."""
        return self._send_at_command(self._format_command("RESET"), is_query=False)

    def default_settings(self):
        """Restore module to factory settings using AT+DEFAULT."""
        return self._send_at_command(self._format_command("DEFAULT"), is_query=False)

    def iap(self):
        """Enter IAP upgrade mode using AT+IAP."""
        return self._send_at_command(self._format_command("IAP"), is_query=False)

    def get_info(self):
        """Query main module parameters using AT+INFO=?."""
        return self._send_at_command(self._format_command("INFO", query=True), is_query=True)

    def get_dev_type(self):
        """Query module model using AT+DEVTYPE=?."""
        return self._send_at_command(self._format_command("DEVTYPE", query=True), is_query=True)

    def get_fwcode(self):
        """Query firmware code using AT+FWCODE=?."""
        return self._send_at_command(self._format_command("FWCODE", query=True), is_query=True)

    def get_power(self):
        """Query RF output power using AT+POWER=?."""
        return self._send_at_command(self._format_command("POWER", query=True), is_query=True)

    def set_power(self, power, save=0):
        """
        Set module transmit power.
        
        :param power: Power in dBm (-9 to +22)
        :param save: 1 to save to flash, 0 otherwise.
        """
        return self._send_at_command(self._format_command("POWER", power, save), is_query=False)

    def get_channel(self):
        """Query the working channel using AT+CHANNEL=?."""
        return self._send_at_command(self._format_command("CHANNEL", query=True), is_query=True)

    def set_channel(self, channel, save=0):
        """
        Set the working channel.
        
        :param channel: Channel number.
        :param save: 1 to save to flash, 0 otherwise.
        """
        return self._send_at_command(self._format_command("CHANNEL", channel, save), is_query=False)

    def get_uart(self):
        """Query the UART parameters using AT+UART=?."""
        return self._send_at_command(self._format_command("UART", query=True), is_query=True)

    def set_uart(self, baud, parity):
        """
        Set UART parameters.
        
        :param baud: Baud rate.
        :param parity: Parity (0:8N0, 1:8E1, 2:8O1)
        """
        return self._send_at_command(self._format_command("UART", baud, parity), is_query=False)

    def get_rate(self):
        """Query the air rate using AT+RATE=?."""
        return self._send_at_command(self._format_command("RATE", query=True), is_query=True)

    def set_rate(self, rate):
        """
        Set the air rate.
        
        :param rate: 0 for 62.5K, 1 for 21.825K, 2 for 7K.
        """
        return self._send_at_command(self._format_command("RATE", rate), is_query=False)

    def get_option(self):
        """Query the communication method using AT+OPTION=?."""
        return self._send_at_command(self._format_command("OPTION", query=True), is_query=True)

    def set_option(self, option, save=0):
        """
        Set the communication method.
        
        :param option: 1: Unicast, 2: Multicast, 3: Broadcast, 4: Anycast.
        :param save: 1 to save to flash.
        """
        return self._send_at_command(self._format_command("OPTION", option, save), is_query=False)

    def get_panid(self):
        """Query the network ID using AT+PANID=?."""
        return self._send_at_command(self._format_command("PANID", query=True), is_query=True)

    def set_panid(self, panid, save=0):
        """
        Set the network ID.
        
        :param panid: Network identification code.
        :param save: 1 to save to flash.
        """
        return self._send_at_command(self._format_command("PANID", panid, save), is_query=False)

    def get_type(self):
        """Query the node type using AT+TYPE=?."""
        return self._send_at_command(self._format_command("TYPE", query=True), is_query=True)

    def set_type(self, node_type):
        """
        Set the node type.
        
        :param node_type: 0 for routing node, 1 for terminal node.
        """
        return self._send_at_command(self._format_command("TYPE", node_type), is_query=False)

    def get_src_addr(self):
        """Query the local address using AT+SRC_ADDR=?."""
        return self._send_at_command(self._format_command("SRC_ADDR", query=True), is_query=True)

    def set_src_addr(self, addr, save=0):
        """
        Set the local address.
        
        :param addr: Address (0 ~ 65535).
        :param save: 1 to save to flash.
        """
        return self._send_at_command(self._format_command("SRC_ADDR", addr, save), is_query=False)

    def get_dst_addr(self):
        """Query the target address using AT+DST_ADDR=?."""
        return self._send_at_command(self._format_command("DST_ADDR", query=True), is_query=True)

    def set_dst_addr(self, addr, save=0):
        """
        Set the target address.
        
        :param addr: Address (0 ~ 65535).
        :param save: 1 to save to flash.
        """
        return self._send_at_command(self._format_command("DST_ADDR", addr, save), is_query=False)

    def get_src_port(self):
        """Query the current port using AT+SRC_PORT=?."""
        return self._send_at_command(self._format_command("SRC_PORT", query=True), is_query=True)

    def set_src_port(self, port, save=0):
        """
        Set the current port.
        
        :param port: Port number.
        :param save: 1 to save to flash.
        """
        return self._send_at_command(self._format_command("SRC_PORT", port, save), is_query=False)

    def get_dst_port(self):
        """Query the target port using AT+DST_PORT=?."""
        return self._send_at_command(self._format_command("DST_PORT", query=True), is_query=True)

    def set_dst_port(self, port, save=0):
        """
        Set the target port.
        
        :param port: Port number (e.g., 1 for normal, 14 for remote configuration).
        :param save: 1 to save to flash.
        """
        return self._send_at_command(self._format_command("DST_PORT", port, save), is_query=False)

    def get_member_rad(self):
        """Query the multicast member radius using AT+MEMBER_RAD=?."""
        return self._send_at_command(self._format_command("MEMBER_RAD", query=True), is_query=True)

    def set_member_rad(self, rad, save=0):
        """
        Set the multicast member radius.
        
        :param rad: Radius (0 ~ 15).
        :param save: 1 to save to flash.
        """
        return self._send_at_command(self._format_command("MEMBER_RAD", rad, save), is_query=False)

    def get_nonmember_rad(self):
        """Query the multicast non-member radius using AT+NONMEMBER_RAD=?."""
        return self._send_at_command(self._format_command("NONMEMBER_RAD", query=True), is_query=True)

    def set_nonmember_rad(self, rad, save=0):
        """
        Set the multicast non-member radius.
        
        :param rad: Radius (0 ~ 15).
        :param save: 1 to save to flash.
        """
        return self._send_at_command(self._format_command("NONMEMBER_RAD", rad, save), is_query=False)

    def get_csma_rng(self):
        """Query the CSMA random avoidance time using AT+CSMA_RNG=?."""
        return self._send_at_command(self._format_command("CSMA_RNG", query=True), is_query=True)

    def set_csma_rng(self, rng):
        """
        Set the CSMA random avoidance time.
        
        :param rng: Time in ms (20 ~ 65535).
        """
        return self._send_at_command(self._format_command("CSMA_RNG", rng), is_query=False)

    def get_router_score(self):
        """Query the maximum number of consecutive route failures using AT+ROUTER_SCORE=?."""
        return self._send_at_command(self._format_command("ROUTER_SCORE", query=True), is_query=True)

    def set_router_score(self, score):
        """
        Set the maximum number of consecutive route failures.
        
        :param score: Value (1 ~ 15).
        """
        return self._send_at_command(self._format_command("ROUTER_SCORE", score), is_query=False)

    def get_head(self):
        """Query the extra frame header function using AT+HEAD=?."""
        return self._send_at_command(self._format_command("HEAD", query=True), is_query=True)

    def set_head(self, enable):
        """
        Enable/disable the extra frame header function.
        
        :param enable: 1 to enable, 0 to disable.
        """
        return self._send_at_command(self._format_command("HEAD", enable), is_query=False)

    def get_back(self):
        """Query the return message function using AT+BACK=?."""
        return self._send_at_command(self._format_command("BACK", query=True), is_query=True)

    def set_back(self, enable):
        """
        Enable/disable the return message function.
        
        :param enable: 1 to enable, 0 to disable.
        """
        return self._send_at_command(self._format_command("BACK", enable), is_query=False)

    def get_security(self):
        """Query the encryption function using AT+SECURITY=?."""
        return self._send_at_command(self._format_command("SECURITY", query=True), is_query=True)

    def set_security(self, enable):
        """
        Enable/disable the encryption function.
        
        :param enable: 1 to enable, 0 to disable.
        """
        return self._send_at_command(self._format_command("SECURITY", enable), is_query=False)

    def get_reset_aux(self):
        """Query the auto-reset LED2 function using AT+RESET_AUX=?."""
        return self._send_at_command(self._format_command("RESET_AUX", query=True), is_query=True)

    def set_reset_aux(self, enable):
        """
        Set auto-reset LED2 change.
        
        :param enable: 1 to enable, 0 to disable.
        """
        return self._send_at_command(self._format_command("RESET_AUX", enable), is_query=False)

    def get_reset_time(self):
        """Query the automatic reset time using AT+RESET_TIME=?."""
        return self._send_at_command(self._format_command("RESET_TIME", query=True), is_query=True)

    def set_reset_time(self, reset_time):
        """
        Set the automatic reset time.
        
        :param reset_time: Minutes (0 to 255; 0 turns off auto-reset).
        """
        return self._send_at_command(self._format_command("RESET_TIME", reset_time), is_query=False)

    def get_filter_time(self):
        """Query the broadcast filter timeout using AT+FILTER_TIME=?."""
        return self._send_at_command(self._format_command("FILTER_TIME", query=True), is_query=True)

    def set_filter_time(self, time_val):
        """
        Set the broadcast filter timeout.
        
        :param time_val: Timeout in ms (3000 ~ 65535).
        """
        return self._send_at_command(self._format_command("FILTER_TIME", time_val), is_query=False)

    def get_ack_time(self):
        """Query the request response timeout using AT+ACK_TIME=?."""
        return self._send_at_command(self._format_command("ACK_TIME", query=True), is_query=True)

    def set_ack_time(self, time_val):
        """
        Set the request response timeout.
        
        :param time_val: Timeout in ms (1000 ~ 65535).
        """
        return self._send_at_command(self._format_command("ACK_TIME", time_val), is_query=False)

    def get_router_time(self):
        """Query the routing request timeout using AT+ROUTER_TIME=?."""
        return self._send_at_command(self._format_command("ROUTER_TIME", query=True), is_query=True)

    def set_router_time(self, time_val):
        """
        Set the routing request timeout.
        
        :param time_val: Timeout in ms (1000 ~ 65535).
        """
        return self._send_at_command(self._format_command("ROUTER_TIME", time_val), is_query=False)

    def group_add(self, group):
        """
        Add a multicast group using AT+GROUP_ADD.
        
        :param group: Group address (0 ~ 65535).
        """
        return self._send_at_command(self._format_command("GROUP_ADD", group), is_query=False)

    def group_del(self, group):
        """
        Delete a multicast group using AT+GROUP_DEL.
        
        :param group: Group address (0 ~ 65535).
        """
        return self._send_at_command(self._format_command("GROUP_DEL", group), is_query=False)

    def group_clear(self, enable):
        """
        Clear the multicast group table using AT+GROUP_CLR.
        
        :param enable: 1 to clear the entire table.
        """
        return self._send_at_command(self._format_command("GROUP_CLR", enable), is_query=False)

    def router_clear(self, enable):
        """
        Clear the routing table in RAM using AT+ROUTER_CLR.
        
        :param enable: 1 to clear.
        """
        return self._send_at_command(self._format_command("ROUTER_CLR", enable), is_query=False)

    def router_save(self, enable):
        """
        Save or delete the routing table in flash using AT+ROUTER_SAVE.
        
        :param enable: 1 to save, 0 to delete.
        """
        return self._send_at_command(self._format_command("ROUTER_SAVE", enable), is_query=False)

    def router_read(self, enable):
        """
        Load the routing table from flash using AT+ROUTER_READ.
        
        :param enable: 1 to load.
        """
        return self._send_at_command(self._format_command("ROUTER_READ", enable), is_query=False)

    def get_mac(self):
        """Query the MAC address using AT+MAC=?."""
        return self._send_at_command(self._format_command("MAC", query=True), is_query=True)

    def get_key(self):
        """Query the encryption key using AT+KEY=?."""
        return self._send_at_command(self._format_command("KEY", query=True), is_query=True)

    def set_key(self, key):
        """
        Set the data encryption key using AT+KEY.
        
        :param key: Encryption key (0 ~ 0x7FFFFFFF).
        """
        return self._send_at_command(self._format_command("KEY", key), is_query=False)

    def close(self):
        """Stop the reader thread and close the serial port."""
        self._running = False
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1)
        if self.ser.is_open:
            self.ser.close()
        self.logger.info("Serial port %s closed.", self.port)
