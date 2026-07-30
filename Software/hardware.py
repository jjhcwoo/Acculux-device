import serial.tools.list_ports


def find_probe_port():

    ports = serial.tools.list_ports.comports()

    for port in ports:
        print(
            port.device,
            port.description
        )

        # Change this to something unique from your device
        if "USB" in port.description:
            return port.device

    return None