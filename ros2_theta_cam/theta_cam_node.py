import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time
from subprocess import getoutput

DEVICE_IDVENDER_STR = "idVendor           0x05ca Ricoh Co., Ltd"
DEVICE_IDPRODUCT_STR = "idProduct          0x2712"

# Nodeクラスを継承
class ThetaNode(Node):
    def __init__(self):
        # Node.__init__を引数node_nameにtalkerを渡して継承
        super().__init__("teta_cam")

        self.declare_parameter('mode', '4K')
        self.declare_parameter('serial', '')
        
        mode = self.get_parameter('mode').get_parameter_value().string_value
        mode = mode.replace('"', '')
        mode = mode.replace("'", '')
        if not mode == '4K' and not mode == '2K':
            raise ValueError("mode must be setted '2K' or '4K'!!")
        serial = self.get_parameter('serial').get_parameter_value().string_value
        serial = serial.replace('"', '')
        serial = serial.replace("'", '')

        is_found = False
        while True:
            device_list_str = getoutput("lsusb -v")
            device_list = device_list_str.rstrip().split('\n')

            for i in range(len(device_list)):
                if DEVICE_IDVENDER_STR in device_list[i] and DEVICE_IDPRODUCT_STR in device_list[i+1]:
                    if not serial == "":
                        if serial in device_list[i+5]:
                            is_found = True
                        else:
                            print(device_list[i+5])
                    else:
                        is_found = True

            if is_found == True:
                break
            else:
                print("wait to connect THETA")
                if not serial == "":
                    print("serial number: " + serial)
                time.sleep(2)
        
        if serial == "":
            config_str = "thetauvcsrc mode=" + mode + " ! decodebin ! autovideoconvert ! video/x-raw,format=BGRx ! queue ! videoconvert ! video/x-raw,format=BGR ! queue ! appsink"
        else:
            config_str = "thetauvcsrc mode=" + mode + " serial=" + serial + " ! decodebin ! autovideoconvert ! video/x-raw,format=BGRx ! queue ! videoconvert ! video/x-raw,format=BGR ! queue ! appsink"
        self.cap = cv2.VideoCapture(config_str)
        if not self.cap.isOpened():
            raise IOError('Cannot open RICOH THETA')

        # Node.create_publisher(msg_type, topic)に引数を渡してpublisherを作成
        self.publisher = self.create_publisher(Image, 'image_topic', 10)
        self.timer_period = 0.01  # seconds

        # Node.create_timer(timer_period_sec, callback)に引数を渡してタイマーを作成
        self.timer = self.create_timer(self.timer_period, self.publish_image_callback)

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

    def publish_image_callback(self):
        bridge = CvBridge()
        ret, frame = self.cap.read()
        if ret:
            image = cv2.resize(frame, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
            image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.publisher.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)

    node = ThetaNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

