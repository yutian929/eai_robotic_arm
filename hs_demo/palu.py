import serial

class AsrproNode:
    def __init__(self):
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)

    def serial_read(self):
        if self.serial_port.in_waiting > 0:
            received_data = self.serial_port.readline().decode().strip()
            print(received_data)
            if received_data == 'awake':
                print('I am awake!')

def main():
    asrpro_node = AsrproNode()
    try:
        while True:
            asrpro_node.serial_read()
    except KeyboardInterrupt:
        pass
    asrpro_node.serial_port.close()

if __name__ == '__main__':
    main()
