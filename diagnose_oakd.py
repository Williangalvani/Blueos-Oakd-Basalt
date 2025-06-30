#!/usr/bin/env python3
"""
OAK-D Device Diagnostic Script
This script helps diagnose OAK-D connection issues
"""

import depthai as dai
import time
import sys

def diagnose_oakd():
    """Diagnose OAK-D device connection issues"""
    
    print("🔍 OAK-D Device Diagnostic Script")
    print("=" * 50)
    
    # Check depthai version
    print(f"📦 DepthAI version: {dai.__version__}")
    
    # Check for available devices
    print("\n🔌 Checking for available devices...")
    try:
        devices = dai.Device.getAllAvailableDevices()
        print(f"   Found {len(devices)} device(s):")
        
        if not devices:
            print("   ❌ No devices found!")
            print("\n📋 Troubleshooting steps:")
            print("   1. Make sure your OAK-D device is connected via USB")
            print("   2. Check USB cable (try a different one if available)")
            print("   3. Try a different USB port (preferably USB 3.0)")
            print("   4. Check device LED indicators:")
            print("      - Power LED should be solid")
            print("      - Status LEDs may blink during initialization")
            print("   5. If device was working before, try disconnecting and reconnecting")
            print("   6. Check if another application is using the device")
            return False
        
        for i, device in enumerate(devices):
            print(f"   {i+1}. Device: {device.name}")
            print(f"      MX ID: {device.deviceId}")
            print(f"      State: {device.state}")
            print(f"      Protocol: {device.protocol}")
            
    except Exception as e:
        print(f"   ❌ Error checking devices: {e}")
        return False
    
    # Try to connect to first available device
    if devices:
        print(f"\n🔗 Attempting to connect to first device...")
        try:
            # Create a simple pipeline
            pipeline = dai.Pipeline()
            
            # Try to connect
            with dai.Device(pipeline, devices[0]) as device:
                print(f"   ✅ Successfully connected to: {device.getDeviceName()}")
                print(f"   USB Speed: {device.getUsbSpeed()}")
                print(f"   Product: {device.getProductName()}")
                
                # Check cameras
                connected_cameras = device.getConnectedCameras()
                print(f"   Connected cameras: {len(connected_cameras)}")
                for cam in connected_cameras:
                    print(f"      - {cam}")
                
                return True
                
        except Exception as e:
            print(f"   ❌ Failed to connect: {e}")
            return False
    
    return False

def wait_for_device():
    """Wait for a device to be connected"""
    print("\n⏳ Waiting for device to be connected (press Ctrl+C to exit)...")
    
    try:
        while True:
            devices = dai.Device.getAllAvailableDevices()
            if devices:
                print(f"\n🎉 Device detected: {devices[0].name}")
                return True
            
            print(".", end="", flush=True)
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\n⚠️ Waiting cancelled by user")
        return False

if __name__ == "__main__":
    print("Starting OAK-D diagnostic...")
    
    # Initial diagnosis
    success = diagnose_oakd()
    
    if not success:
        # Offer to wait for device
        response = input("\n❓ Would you like to wait for a device to be connected? (y/n): ")
        if response.lower().startswith('y'):
            if wait_for_device():
                print("\n🔄 Re-running diagnosis...")
                diagnose_oakd()
            else:
                print("\n📝 Next steps:")
                print("   1. Check physical connection")
                print("   2. Try different USB cable/port")
                print("   3. Restart the device if possible")
                print("   4. Check if device appears in 'lsusb' output")
    else:
        print("\n✅ Device connection successful!")
        print("   Your OAK-D device should work with the basalt application.")

    print("\n🏁 Diagnostic complete.") 