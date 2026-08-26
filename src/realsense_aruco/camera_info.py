"""List connected RealSense devices."""


def main() -> None:
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise SystemExit("pyrealsense2 is not installed; run ./scripts/setup.sh") from exc
    try:
        devices = list(rs.context().query_devices())
    except RuntimeError as exc:
        raise SystemExit(f"Cannot access RealSense devices: {exc}. Check udev rules and USB permissions.") from exc
    if not devices:
        print("No RealSense device detected")
        return
    for index, device in enumerate(devices):
        name = device.get_info(rs.camera_info.name)
        serial = device.get_info(rs.camera_info.serial_number)
        firmware = device.get_info(rs.camera_info.firmware_version)
        print(f"{index}: {name}; serial={serial}; firmware={firmware}")


if __name__ == "__main__":
    main()

