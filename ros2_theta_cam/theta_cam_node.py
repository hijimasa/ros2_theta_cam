import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time
from subprocess import getoutput
import subprocess
from multiprocessing import Process, Pipe
import usb.core
import usb.util
import os
import queue
from threading import Thread
import gphoto2cffi as gp


DEVICE_IDVENDER_STR =   "idVendor           0x05ca Ricoh Co., Ltd"
DEVICE_IDPRODUCT_STR =  "idProduct          0x2712"
DEVICE_IDPRODUCT_STR2 = "idProduct          0x0368"


class FrameGrabber:
    def __init__(self, config_str):
        # １回だけ開く
        self.cap = cv2.VideoCapture(config_str, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise IOError("Cannot open capture pipeline")
        # サイズ１のキュー（常に最新１枚だけ残す）
        self.q = queue.Queue(maxsize=1)
        self.running = True
        # 背景スレッド起動
        t = Thread(target=self._grab_loop, daemon=True)
        t.start()

    def _grab_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            # もし前のフレームが残っていれば捨てる
            if self.q.full():
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    pass
            self.q.put(frame)

    def read(self, timeout=1.0):
        try:
            frame = self.q.get(timeout=timeout)
            return True, frame
        except queue.Empty:
            return False, None

    def release(self):
        self.running = False
        self.cap.release()

# Nodeクラスを継承
class ThetaNode(Node):
    def __init__(self):
        # Node.__init__を引数node_nameにtalkerを渡して継承
        super().__init__("theta_cam")

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
                    print(device_list[i])
                    if not serial == "":
                        if serial in device_list[i+5]:
                            is_found = True
                        else:
                            print(device_list[i+5])
                    else:
                        is_found = True
                if DEVICE_IDVENDER_STR in device_list[i] and DEVICE_IDPRODUCT_STR2 in device_list[i+1]:
                    # ベンダー／プロダクト ID を指定
                    dev = usb.core.find(idVendor=0x05ca, idProduct=0x0368)
                    if dev is not None:
                        # リセット実行
                        dev.reset()
                        print("USB デバイスをリセットしました")
                        time.sleep(1)
                    dev = usb.core.find(idVendor=0x05ca, idProduct=0x0368)
                    #Bus 003 Device 019: ID 05ca:0368 Ricoh Co., Ltd RICOH THETA V
                        # アクティブなコンフィグの全インターフェースでドライバをデタッチ
                    for cfg in dev:
                        for intf in cfg:
                            num = intf.bInterfaceNumber
                            if dev.is_kernel_driver_active(num):
                                dev.detach_kernel_driver(num)
                                usb.util.release_interface(dev, num)
                    time.sleep(1)
                    device_port = "usb:" + device_list[i-9][4:7] + ":" + device_list[i-9][15:18]
                    print("device port: " + device_port)
                    subprocess.run(f"gphoto2 --port={device_port} --set-config 5013=32773", shell=True)
                    time.sleep(1)

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
        self.frame_grabber = FrameGrabber(config_str)

        # Node.create_publisher(msg_type, topic)に引数を渡してpublisherを作成
        self.publisher = self.create_publisher(Image, 'image_topic', 10)
        # タイマー周期を60Hz基準に設定（カメラの30FPSを安定的に取得）
        self.timer_period = 0.0167  # seconds (約60Hz)

        # Node.create_timer(timer_period_sec, callback)に引数を渡してタイマーを作成
        self.timer = self.create_timer(self.timer_period, self.publish_image_callback)

        self.error_flag = False
        # 30FPSから10FPSへの間引き用カウンタ（3回に1回publish）
        self.counter = 0

        # CvBridgeを1回だけ生成（毎回生成するとオーバーヘッドが大きい）
        self.bridge = CvBridge()

    def __del__(self):
        return

    def publish_image_callback(self):
        ret, frame = self.frame_grabber.read(timeout=1.0)

        if ret:
            self.counter = self.counter + 1
            if self.counter >= 3:
                self.counter = 0
                # スライシングによる高速リサイズ（cv2.resizeより軽量）
                image = frame[::2, ::2]
                image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                self.publisher.publish(image_msg)
        else:
            # フレーム取得失敗時はエラーログを出してシャットダウン
            self.get_logger().error('Failed to capture image. Shutting down node.')
            # ROS2 の spin を抜ける
            self.error_flag = True
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)

    node = ThetaNode()

     # 手動 spin ループ
    try:
        while rclpy.ok() and not node.error_flag:
            rclpy.spin_once(node, timeout_sec=0.1)
    # すべてのエラーをキャッチ
    except Exception as e:
        node.get_logger().error(f'Exception occurred: {e}')
        node.error_flag = True
        node.timer.cancel()
    finally:
        node.get_logger().info('Cleaning up and exiting.')
        node.destroy_node()
        rclpy.shutdown()
        os._exit(0)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

