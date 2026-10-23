import time
from synchronized import read_synchronized_response

dischargeA5_req = bytes([0x21, 0x64, 0xA5, 0x00, 0xD6])

ans=bytes()

def discharge_akku(device):
    uart = device.protocols.uart
    uart.write(dischargeA5_req)
    time.sleep(0.02)
    # ans, parity = uart.read(buffer_size=5)  # returns bytes if your target echoes
    ans = read_synchronized_response(uart, expected_len=5)

    while len(ans) < 5:
        uart.write(dischargeA5_req)
        time.sleep(0.01)
        ans, parity = uart.read(buffer_size=5)
