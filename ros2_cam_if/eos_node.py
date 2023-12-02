import rclpy
from rclpy.node import Node
import warnings
from rclpy.action import ActionServer
import os 

from .cam_interface.capture import EOS
from .cam_interface import gphoto_util
# from cam_interface.capture import EOS
# from cam_interface import gphoto_util

from capture_types.action import Capture
from capture_types.srv import CameraConfig

class EOS_node(Node):

    def __init__(self):
        super().__init__('eos_cam')

        self.declare_parameter('port', rclpy.Parameter.Type.STRING) # USB port address, to be set in the launch file
        self.declare_parameter('target_path', rclpy.Parameter.Type.STRING) # path to the target folder for image and vid downloads, to be set in the launch file
        # self.declare_parameter('port', 'usb:002,020')
        # self.declare_parameter('target_path', '/home/karoline/rosws/media')
        # when initializing the camera, please make sure to specify a unique target path for media downloads, otherwise files might be overwritten

        try:
            port = self.get_parameter('port')
        except rclpy.exceptions.ParameterUninitializedException: 
            warnings.warn("Could not start camera node, the port has not been specified")
            exit()
        try:
            target_path = self.get_parameter('target_path')
        except rclpy.exceptions.ParameterUninitializedException:
            warnings.warn("Could not start camera node, a target path for media downloads has not been specified")
            exit()

        self.cam = EOS(port=port.value)
        self.cam.sync_date_time() # sync camera date and time with PC at startup
        
        # set up services
        self.sync_date_time_srv = self.create_service(CameraConfig, '~/sync_date_time', self.sync_date_time)
        self.drive_focus_srv = self.create_service(CameraConfig, '~/drive_focus', self.drive_focus)
        self.get_ap_srv = self.create_service(CameraConfig, '~/get_aperture', self.get_aperture)
        self.set_ap_srv = self.create_service(CameraConfig, '~/set_aperture', self.set_aperture)
        self.get_ss_srv = self.create_service(CameraConfig, '~/get_shutterspeed', self.get_shutter_speed)
        self.set_ss_srv = self.create_service(CameraConfig, '~/set_shutterspeed', self.set_shutter_speed)
        self.get_iso_srv = self.create_service(CameraConfig, '~/get_iso', self.get_iso)
        self.set_iso_srv = self.create_service(CameraConfig, '~/set_iso', self.set_iso)
        self.set_contaf_srv = self.create_service(CameraConfig, '~/set_continuous_af', self.set_cont_af)
        self.get_imform_srv = self.create_service(CameraConfig, '~/get_image_format', self.get_imform)
        self.set_imform_srv = self.create_service(CameraConfig, '~/set_image_format', self.set_imform)
        self.set_afpoint_srv = self.create_service(CameraConfig, '~/set_af_point', self.set_afpoint)
        self.trigger_af_srv = self.create_service(CameraConfig, '~/trigger_af', self.trigger_AF)

        # set up actions
        self._single_capture_svr = ActionServer(self,Capture,'~/single_capture',self.single_capture)
        self._preview_capture_svr = ActionServer(self,Capture,'~/preview_capture',self.preview_capture)
        self._preview_video_svr = ActionServer(self,Capture,'~/preview_video',self.preview_video)
        self._burst_srv = ActionServer(self,Capture, '~/burst_capture', self.burst_capture)
        self._video_capture_svr = ActionServer(self,Capture,'~/video_capture',self.video_capture)


    def sync_date_time(self,request,response):
        self.cam.sync_date_time()
        response.output_msg = 'done'
        return response
    
    def drive_focus(self,request,response):
        # input values:
        # 0,1,2 == small, medium, large increment --> nearer
        # 3 == none
        # 4,5,6 == small, medium, large increment --> further 
        if len(request.x) > 0:
            msg = self.cam.manual_focus(value=int(request.x))
            response.output_msg = msg
            return response
        else:
            warnings.warn("No value passed to the manual focus drive service. Focus has not been adjusted.")
            response.output_msg = 'error, missing value x'
            return response
    
    def get_aperture(self,request,response):
        current, choices, msg = self.cam.set_aperture(value=None, list_choices=True)
        response.current_val = current
        response.choices = [str(choice) for choice in choices]
        response.output_msg = msg
        return response
    
    def set_aperture(self,request,response):
        current, choices, msg = self.cam.set_aperture(value=request.x, list_choices=False)
        response.current_val = current
        response.choices = [str(choice) for choice in choices]
        response.output_msg = msg
        return response
    
    def get_shutter_speed(self,request,response):
        current, choices, msg = self.cam.set_shutterspeed(value=None, list_choices=True)
        response.current_val = current
        response.choices = [str(choice) for choice in choices]
        response.output_msg = msg
        return response
    
    def set_shutter_speed(self,request,response):
        current, choices, msg = self.cam.set_shutterspeed(value=request.x, list_choices=False)
        response.current_val = current
        response.choices = [str(choice) for choice in choices]
        response.output_msg = msg
        return response
    
    def get_iso(self,request,response):
        current, choices, msg = self.cam.set_iso(value=None, list_choices=True)
        response.current_val = current
        response.choices = [str(choice) for choice in choices]
        response.output_msg = msg
        return response
    
    def set_iso(self,request,response):
        current, choices, msg = self.cam.set_iso(value=request.x, list_choices=False)
        response.current_val = current
        response.choices = [str(choice) for choice in choices]
        response.output_msg = msg
        return response
    
    def set_cont_af(self,request,response):
        current, msg = self.cam.set_continuous_AF(value=request.x)
        response.current_val = current
        response.output_msg = msg
        return response
    
    def get_imform(self,request,response):
        current, choices, msg = self.cam.set_image_format(value=None, list_choices=True)
        response.current_val = current
        response.choices = [str(choice) for choice in choices]
        response.output_msg = msg
        return response
    
    def set_imform(self,request,response):
        current, choices, msg = self.cam.set_image_format(value=request.x, list_choices=False)
        response.current_val = current
        response.choices = choices
        response.output_msg = msg
        return response
    
    def set_afpoint(self,request,response):
        try:
            x = int(request.x)
            y = int(request.y)
        except:
            response.output_msg = 'error, x and y coordinates must be integers, check for any trailing spaces or zeros'
            return response
        current, msg = self.cam.set_AF_location(x=x, y=y)
        response.current_val = current
        response.output_msg = msg
        return response
    
    def trigger_AF(self,request,response):
        msg = self.cam.trigger_AF()
        response.output_msg = msg
        return response
    
    def single_capture(self, goal_handle):
        success, msg = self.cam.capture_immediate(download=goal_handle.request.download, target_path=self.get_parameter('target_path').value)
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result = Capture.Result()
        result.output_path = self.get_parameter('target_path').value
        result.output_msg = msg
        return result
    
    def preview_capture(self, goal_handle):
        # OVERWRITES previous preview pictures by default
        # does not need any inputs
        target_file = self.get_parameter('target_path').value + '/preview.jpg'
        success, msg = self.cam.capture_preview(target_file=target_file)
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result = Capture.Result()
        result.output_path = target_file
        result.output_msg = msg
        return result
    
    def preview_video(self, goal_handle):
        # OVERWRITES previous preview videos by default
        target_file = self.get_parameter('target_path').value + '/preview.mp4'
        success, msg = self.cam.record_preview_video(t=goal_handle.request.duration, target_file=target_file , resolution_prio=True)
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result = Capture.Result()
        result.output_path = target_file
        result.output_msg = msg
        return result
    
    def burst_capture(self, goal_handle):
        success, files, msg = self.cam.capture_burst(t=goal_handle.request.duration)
        if success:
            goal_handle.succeed()
            feedback_msg = Capture.Feedback()
            feedback_msg.status = 'Capture successful. Files saved to camera.'
            goal_handle.publish_feedback(feedback_msg)
        else:
            goal_handle.abort()
            result = Capture.Result()
            result.output_path = ''
            result.output_msg = msg
            return result

        if goal_handle.request.download:
            feedback_msg.status = 'Downloading files to PC'
            goal_handle.publish_feedback(feedback_msg)
            path = self.get_parameter('target_path').value + '/' + 'burst'
            os.path.exists(path)
            if not os.path.exists(path):
                os.makedirs(path)
            for image in files: 
                target_file = path + '/' + image.split('/')[-1]
                self.cam.download_file(image, target_file=target_file)
            msg = 'Files downloaded to PC'
        result = Capture.Result()
        result.output_path = path
        result.output_msg = msg
        return result
    
    def video_capture(self, goal_handle):
        # Video files are named by the camera naming convention, so should not overwrite by default
        success, msg = self.cam.record_video(t=goal_handle.request.duration, download=goal_handle.request.download, target_path=self.get_parameter('target_path').value)
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result = Capture.Result()
        result.output_path = self.get_parameter('target_path').value
        result.output_msg = msg
        return result
    

def main():
    rclpy.init()
    print(gphoto_util.detect_EOS_cameras()[0])
    cam_node = EOS_node()

    rclpy.spin(cam_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
