from xensegripper.node import MasterNode, XenseSensorNode, XenseGripperNode
from xensegripper.node.camera_server import CameraServer
from xensesdk.ezros import call_service, ip_tool
import argparse

def main():
    parser = argparse.ArgumentParser(description="Xense Gripper Master Node")
    parser.add_argument("-m", "--master", action="store_true", help="Run master node")
    parser.add_argument("-s", "--serial_number", type=str, help="Run sensor node with serial number")
    parser.add_argument("-g", "--gripper", action="store_true", help="Run gripper node")
    parser.add_argument("-r", "--reboot", action="store_true", help="Reboot")
    parser.add_argument("-c", "--camera", type=str, help="Run camera node with camera ID")
    parser.add_argument('--frame_size', type=int, nargs=2, default=(640, 480), help='Size of the Camera feed (width height)')
    parser.add_argument('--frame_rate', type=int, default=50, help='Frame rate of the Camera feed, min 50')


    args = parser.parse_args()
    
    if args.master:
        master_service = MasterNode()
        master_service.run()
    elif args.serial_number:
        node = XenseSensorNode(serial_number=args.serial_number)
        node._logger.info(f"Node {args.serial_number} ended.")
    elif args.gripper:
        gripper = XenseGripperNode()
        gripper._logger.info("gripper process end.")
    elif args.reboot:
        mac_addr = ip_tool.get_mac_address('eth0')
        call_service(f"master_{mac_addr}", "reboot")
        print(f"Reboot command sent to master node [master_{mac_addr}]...")
    elif args.camera:
        CameraServer(serial_number=args.camera, frame_size=args.frame_size, frame_rate=args.frame_rate).run()
    else:
        print("Please specify an action: --master, --serial_number <SN>, or --gripper.")
        sensors = MasterNode.scan_xense_devices()[0]
        print("Available sensors:")
        for sensor in sensors:
            print(f" - {sensor}")
        print("==========\n")
        parser.print_help()

if __name__ == '__main__':
    main()