from evdev import InputDevice, list_devices

def list_input_devices():
    print("Available Input Devices:")
    for path in list_devices():
        try:
            device = InputDevice(path)
            print(f"Path: {path}")
            print(f"  Name: {device.name}")
            print(f"  Phys: {device.phys}")
            print(f"  Uniq: {device.uniq}")
            print(f"  Info: vendor 0x{device.info.vendor:04x}, product 0x{device.info.product:04x}")
            print()
        except Exception as e:
            print(f"Error accessing device at {path}: {e}")

if __name__ == "__main__":
    list_input_devices()
