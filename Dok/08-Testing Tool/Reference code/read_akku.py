import time
from synchronized import read_synchronized_response

version_req = bytes([0x3F, 0x76, 0x00, 0x00, 0x4B])
capacity_req= bytes([0x3F, 0x63, 0x00, 0x00, 0x5E])
remaining_req= bytes([0x3F, 0x72, 0x00, 0x00, 0x4F])
status_req= bytes([0x3F, 0x73, 0x00, 0x00, 0x4E])
LFC_req= bytes([0x3F, 0x63, 0xA5, 0x00, 0xB9])
day_req= bytes([0x3F, 0x74, 0x00, 0x00, 0x4D])

ans=bytes()

# Define flags from bit 7 (MSB) down to bit 0 (LSB)
# Index 0 is Bit 7, Index 7 is Bit 0
flags = ['n', 'h', 'r', 'd', 'm', 'v', 'i', 'g','a', 'w', 's', 'u', 'T', 't', 'H', 'c']

DEFAULT_DATA = [None, None, None, None, None, None, None]

def read_akku(device):
    """ Send commands to the Battery and read the answer"""
    #print("Started")
    uart = device.protocols.uart

    # Transmit
    # --Version--

    uart.write(version_req)
    time.sleep(0.02)
    #ans, parity = uart.read(buffer_size=5)  # returns bytes if your target echoes
    ans = read_synchronized_response(uart, expected_len=5)


    version = ans[2]
    #print("SW version", version)
    time.sleep(0.02)

    # --Capacity--
    uart.write(capacity_req)
    time.sleep(0.02)
    ans = read_synchronized_response(uart, expected_len=5)
    #ans, parity = uart.read(buffer_size=5)  # returns bytes if your target echoes


    # Reconstruct a 16-bit integer
    capacity = int.from_bytes(ans[2:4], byteorder='little')
    # print("Capacity", capacity)
    time.sleep(0.3)

    # --Remaining time--

    uart.write(remaining_req)
    time.sleep(0.02)
    #ans, parity = uart.read(buffer_size=5)  # returns bytes if your target echoes
    ans = read_synchronized_response(uart, expected_len=5)
    remaining = int.from_bytes(ans[2:4], byteorder='little')
    # print("Remaining", remaining)
    time.sleep(0.3)

    # --Status--

    uart.write(status_req)
    time.sleep(0.02)
    ans, parity = uart.read(buffer_size=5)  # returns bytes if your target echoes

    # Byte 2 and 3 have the status flags
    byte = int.from_bytes(ans[2:4], byteorder='little')

    # Build the string by checking if each bit is set
    result = ""
    for i in range(15, -1, -1):  # Counting down from 7 to 0
        # Check if the i-th bit is 1
        if (byte >> i) & 1:
            result += flags[15 - i]
        else:
            result += "-"

    # print(f"Status: {result}")

    time.sleep(0.3)

    # --LFC

    uart.write(LFC_req)
    time.sleep(0.02)
    #ans, parity = uart.read(buffer_size=5)  # returns bytes if your target echoes
    ans = read_synchronized_response(uart, expected_len=5)

    # Reconstruct a 16-bit integer
    LFC = int.from_bytes(ans[2:4], byteorder='little')
    # print("LFC", LFC)
    time.sleep(0.3)

    # --Day counter

    uart.write(day_req)
    time.sleep(0.02)
    #ans, parity = uart.read(buffer_size=5)  # returns bytes if your target echoes
    ans = read_synchronized_response(uart, expected_len=5)
    day = int.from_bytes(ans[2:3])
    # print("Day counter", day)
    time.sleep(0.3)

    # Read the voltage value
    scope = device.analog_input
    scope.read_status()
    voltage = scope[0].get_sample()
    voltage = format(voltage, ".3f")
    # print("Voltage", voltage)



    return version, capacity, remaining, result, LFC, day, voltage