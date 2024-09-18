import rospy
import numpy as np
import open3d as o3d

from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import yaml

from t_pointcloud import t_filter


def readYaml(file, label):
    with open(file, 'r') as f:
        return yaml.safe_load(f)[label]


class CameraHandle:
    def __init__(self, int_yaml) -> None:
        self.cv_bridge = CvBridge()
        self.initialize_intrinsics(int_yaml)

    def initialize_intrinsics(self, int_yaml):
        with open(int_yaml, 'r') as int_yaml:
            cam_params = yaml.safe_load(int_yaml)
            
            # Load raw params
            self.dst_coeffs = np.array(cam_params["distortion_coeffs"])

            self.fx, self.fy, self.cx, self.cy = cam_params["intrinsics"]
            self.mtx = np.array([
                [self.fx, 0, self.cx],
                [0, self.fy, self.cy],
                [0, 0, 1]
            ])
            
            self.w, self.h = tuple(cam_params["resolution"])

            # Get optimal params
            self.optimal_mtx, self.optimal_roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dst_coeffs, (self.w,self.h), 1, (self.w,self.h))
            self.map_raw, self.map_undistorted = cv.initUndistortRectifyMap(self.mtx, self.dst_coeffs, None, self.optimal_mtx, (self.w,self.h), 5)

    def get_undistorted_image(self, img_msg, crop_to_roi=True):
        # Bridge image msg to opencv format
        try:
           img_raw = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
           rospy.logerr(e)

        # Undistort image
        img_undist = cv.remap(img_raw, self.map_raw, self.map_undistorted, cv.INTER_LINEAR)
        if crop_to_roi:
            img_undist = self.crop_to_roi(img_undist)
        return img_undist
    
    def crop_to_roi(self, img):
        x, y, w, h = self.optimal_roi
        return img[y:y+h, x:x+w]

    def project_points(self, xyz):
        uv = self.optimal_mtx @ xyz
        uv /= uv[2]
        return uv[:2]
    
    def uv_in_roi(self, uv):
        x, y, w, h = self.optimal_roi
        return (uv[0] >= x) & (uv[0] < x+w) & (uv[1] >= y) & (uv[1] < y+h)
    
    def get_color_from_image(self, image, points):
        uv = np.rint(self.project_points(points)).astype(np.int32)
        in_frame_mask = self.uv_in_roi(uv)

        uv_in_frame = uv[:, in_frame_mask]
        colors = image[uv_in_frame[1], uv_in_frame[0]]

        return colors, in_frame_mask


class ImageRenderer:
    def __init__(self, camera_object, clouds, info) -> None:
        # Store cloud data
        self.clouds = clouds
        self.cloud_interval = readYaml(info, "cloud_max_interval")
        
        self.cam = camera_object
        self.cam_params = o3d.camera.PinholeCameraParameters()

        self.w, self.h = int(2*self.cam.cx + 1), int(2*self.cam.cy + 1)
        self.cam_params.intrinsic = o3d.camera.PinholeCameraIntrinsic(self.w, self.h, self.cam.fx, self.cam.fy, self.w/2 - 0.5, self.h/2 - 0.5)

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(visible = False, width=self.w, height=self.h)

        self.render_opt = self.vis.get_render_option()
        self.render_opt.background_color = readYaml(info, "reconstruction_background_color")
        self.render_opt.point_size = readYaml(info, "reconstruction_point_size")

        self.view_ctr = self.vis.get_view_control()
        self.view_ctr.set_constant_z_near(0.1) # Solves issue with points being clipped in near field of view
        


    def update_pointcloud(self, t_query, dt):
        pcd_window = o3d.geometry.PointCloud()

        # Self.clouds contain a key (t0) and a pcd
        for t0, pcd in self.clouds.items():
            # Is cloud in future?
            if t_query - dt > t0 + self.cloud_interval:
                continue
            # Is cloud in past?
            if t_query + dt < t0 :
                continue

            pcd = t_filter(pcd, channel="timestamps", min=t_query-dt, max=t_query+dt)
            pcd_window += pcd.to_legacy()
                
        self.vis.clear_geometries()
        self.vis.add_geometry(pcd_window)
        

    def render_image(self, T_cam):        
        self.cam_params.extrinsic = T_cam
        self.view_ctr.convert_from_pinhole_camera_parameters(self.cam_params)
        
        self.vis.poll_events()
        self.vis.update_renderer()
        
        return self.cam.crop_to_roi(np.asarray(self.vis.capture_screen_float_buffer()))*255