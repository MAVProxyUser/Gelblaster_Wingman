import hid

# List all connected HID devices
devices = hid.enumerate()

if not devices:
    print("No HID devices found.")
else:
    for device in devices:
        print(f"Vendor ID: {hex(device['vendor_id'])}, Product ID: {hex(device['product_id'])}, "
              f"Manufacturer: {device['manufacturer_string']}, Product: {device['product_string']}, "
              f"Interface: {device['interface_number']}")

