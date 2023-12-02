import rclpy
from capture_types.action import Capture
from capture_types.srv import CameraConfig
from rclpy.node import Node
import time 

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.ap_test_client = self.create_client(CameraConfig, '/eos_cam/set_aperture')
        self.ss_test_client = self.create_client(CameraConfig, '/eos_cam/set_shutterspeed')
        while not self.ap_test_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = CameraConfig.Request()

    def ap_send_request(self, x, y):
        self.req.x = x
        self.req.y = y
        self.future = self.ap_test_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def ss_send_request(self, x, y):
        self.req.x = x
        self.req.y = y
        self.future = self.ss_test_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    start = time.time()
    response = minimal_client.ap_send_request('2.8','')
    print(time.time()-start)
    response = minimal_client.ap_send_request('32','')
    end = time.time()
    print(end-start)

    start = time.time()
    response = minimal_client.ss_send_request('30','')
    print(time.time()-start)
    response = minimal_client.ss_send_request('1/8000','')
    end = time.time()
    print(end-start)

    minimal_client.get_logger().info(f'Result {response.output_msg}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()