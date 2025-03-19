# E52-xxxNW22S For Python

This library provides an easy-to-use Python interface for the E52-xxxNW22S  LoRa modules via UART using AT commands. It supports complete module configuration,  messaging functionality, and handles asynchronous data with robust threading  and filtering techniques.

## Example Usage

* İmport and connect to module:
```python
    from E52-xxxNW22S import LoRaModule

    lora = LoRaModule(port="/dev/cu.usbserial-2120", baudrate=115200,   timeout=1, retries=3, log_enabled=True)

```
* Get Module Info:
```python
    # Query module info (this returns the key parameters.)
    info_resp = lora.get_info()
    print("Module Info:\n",info_resp)
```

* Configure:
```python
    # Set the channel (this should return something like "A+CHANNEL=OK")
    set_channel_resp = lora.set_channel(13, 1)
    print("Set Channel Response:\n",set_channel_resp)

    # Set the otion as broadcast (this should return something like "AT +OPTION=OK")
    set_option_resp = lora.set_option(3, 1)
    print("Set Option Response:\n",set_option_resp)

    # Set the target address (this should return something like "AT +DSADDR=OK")
    set_dst_addr = lora.set_dst_addr(34, 1)
    print("Set Dst Address Response:\n",set_dst_addr)

```

* Send Massage:
```python
    single_response = lora.send_message("Will You Marry Me Module B!?")
    print("Single send response:", single_response)

```

* Read Taken Messages:
```python
    def handle_async_message(message):
        print("Received async message:", message)
        #Example output: 'Yes Module A I Love U'

    #Start Massage Read Loop
    lora.async_callback = handle_async_message

    # Keep the main thread alive to continue processing async messages.
    try:
        while True:
            time.sleep(1)  # Your async callback runs in the background.

```


  
## Licance

[MIT](https://choosealicense.com/licenses/mit/)

  
## Support

For support, send an email to mail@abdullahseckin.com.

  
## Badges

[![GPLv3 License](https://img.shields.io/badge/Platforms-Linux_macOS_Windows-white)](https://opensource.org/licenses/)

[![MIT License](https://img.shields.io/badge/Language-Python-blue)](https://choosealicense.com/licenses/mit/)

[![AGPL License](https://img.shields.io/badge/Network-LorRa-yellow)](http://www.gnu.org/licenses/agpl-3.0)




  
