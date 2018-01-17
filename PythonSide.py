
#!/usr/bin/env python3
# This is based on the exmaple code from Adafruit Bluetooth LE library
# Author: Jack Qiao, Chuck Fu

# these libraries are used for fourier transform
import math
from matplotlib.mlab import find
import numpy as np
# these libraries are used for bluetooth LE connection
import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART

import time

# Get the BLE provider for the current platform.
ble = Adafruit_BluefruitLE.get_provider()

def freqToLight(freq):
    if  freq < 500:
        return (0,0,230)
    elif freq >=500 and freq < 1000:
        return  (200, 0, 200)
    elif freq >= 1000 and freq < 1500:
        return (0, 200, 0)
    elif freq >= 1500 and freq < 2500:
        return (200, 200, 0)
    elif freq >= 2500:
        return (230, 0, 0)
    else:
        return (0, 0, 0)

# Main function implements the program logic so it can run in a background
# thread.  Most platforms require the main thread to handle GUI events and other
# asyncronous events like BLE actions.  All of the threading logic is taken care
# of automatically though and you just need to provide a main function that uses
# the BLE provider.
def main():
    # Clear any cached data because both bluez and CoreBluetooth have issues with
    # caching data and it going stale.
    ble.clear_cached_data()

    # Get the first available BLE network adapter and make sure it's powered on.
    adapter = ble.get_default_adapter()
    adapter.power_on()
    print('Using adapter: {0}'.format(adapter.name))

    # Disconnect any currently connected UART devices.  Good for cleaning up and
    # starting from a fresh state.
    print('Disconnecting any connected UART devices...')
    UART.disconnect_devices()

    # Scan for UART devices.
    print('Searching for UART device...')
    try:
        adapter.start_scan()
        # Search for the first UART device found (will time out after 60 seconds
        # but you can specify an optional timeout_sec parameter to change it).
        device = UART.find_device()
        if device is None:
            raise RuntimeError('Failed to find UART device!')
    finally:
        # Make sure scanning is stopped before exiting.
        adapter.stop_scan()

    print('Connecting to device...')
    device.connect()  # Will time out after 60 seconds, specify timeout_sec parameter
                      # to change the timeout.

    # Once connected do everything else in a try/finally to make sure the device
    # is disconnected when done.
    try:
        # Wait for service discovery to complete for the UART service.  Will
        # time out after 60 seconds (specify timeout_sec parameter to override).
        print('Discovering services...')
        UART.discover(device)

        # Once service discovery is complete create an instance of the service
        # and start interacting with it.
        uart = UART(device)
        # print ('Ready to send')
        while(True):
            # Now wait to receive audio data from bluetooth, until a sample buffer
            # of 128 is filled.
            while (len(audio) < 128):
                received = uart.read(timeout_sec=20)
                if received is not None:
                    # Received data, print it out.
                    audio.append(received)
                    # print('Received: {0}'.format(received))
                else:
                    # Timeout waiting for data, None is returned.
                    print('Received no data!')

            # produce a frequency value of the audio data
            fourier = np.fft.fft(audio)
            freqs = np.fft.fftfreq(len(fourier))
            idx = np.argmax(np.abs(fourier))
            freq = freqs[idx]
            freq_in_hertz = abs(freq * 11025.0) # frequency (Hz) = abs(fft_freq * frame_rate)
            print(freq_in_hertz)
            # map the frequency to a color.
            (r, g, b) = freqToLight(freq_in_hertz)
            # Writing characters to the UART SERVICE. 
            # print(g, " ", r, " ", b)
            rgb = bytes([r, g, b])
            print(rgb) 
            # write that color over to bluetooth.
            uart.write(rgb)
            time.sleep(0.5)

    except KeyboardInterrupt:
        rgb = bytes([0, 0, 0])
        uart.write(rgb)
        pass
    finally:
        # Make sure device is disconnected on exit.
        device.disconnect()

# Initialize the BLE system.  MUST be called before other BLE calls!
ble.initialize()

# Start the mainloop to process BLE events, and run the provided function in
# a background thread.  When the provided main function stops running, returns
# an integer status code, or throws an error the program will exit.
ble.run_mainloop_with(main)
