import rclpy
from rclpy.node import Node
import warnings
from rclpy.action import ActionServer
import time
from cam_interface.capture import EOS
from cam_interface import gphoto_util


from cam_types.action import Capture
from cam_types.srv import Config

class EOS_node(Node):

    def __init__(self):
        super().__init__('eos_cam')

        #self.declare_parameter('port', rclpy.Parameter.Type.STRING) # USB port address, to be set in the launch file
        self.declare_parameter('port', 'usb:002,011') 

        try:
            port = self.get_parameter('port')
        except rclpy.exceptions.ParameterUninitializedException: 
            warnings.warn("Could not start camera node, the port has not been specified")
            exit()

        self.cam = EOS(port=port.value)
        
        # set up services
        self.sync_date_time_srv = self.create_service(Config, '~/sync_date_time', self.sync_date_time)
        self.drive_focus_srv = self.create_service(Config, '~/drive_focus', self.drive_focus)
        self.get_ap_srv = self.create_service(Config, '~/get_aperture', self.get_aperture)
        self.set_ap_srv = self.create_service(Config, '~/set_aperture', self.set_aperture)


    def sync_date_time(self,request,response):
        self.cam.sync_date_time()
        response.current_val = 'done'
        return response
    
    def drive_focus(self,request,response):
        # input values:
        # 0,1,2 == small, medium, large increment --> nearer
        # 3 == none
        # 4,5,6 == small, medium, large increment --> further 
        if len(request.val1) > 0:
            self.cam.manual_focus(value=int(request.val1))
            response.current_val = 'done'
            return response
        else:
            warnings.warn("No value passed to the manual focus drive service. Focus has not been adjusted.")
            response.current_val = 'error, missing val1'
            return response
    
    def get_aperture(self,request,response):
        current, choices= self.cam.set_aperture(value=None, list_choices=True)
        response.current_val = current
        response.choices = [str(choice) for choice in choices]
        return response
    
    def set_aperture(self,request,response):
        current, choices= self.cam.set_aperture(value=request.val1, list_choices=False)
        response.current_val = current
        response.choices = [str(choice) for choice in choices]
        return response
    



def main():
    rclpy.init()
    print(gphoto_util.detect_EOS_cameras()[0])
    cam_node = EOS_node()

    rclpy.spin(cam_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
