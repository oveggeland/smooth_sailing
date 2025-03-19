import open3d as o3d
import cv2 as cv
import numpy as np
import os

from t_pointcloud import t_channel_to_color, t_filter, t_get_channel

class PcdVisualizer:
    def __init__(self) -> None:        
        self.exp = "/home/oskar/smooth_sailing/data/long3/exp/journal"  
    
        #### Init geometry ####
        # Ice map
        self.pcd = o3d.t.io.read_point_cloud(os.path.join(self.exp, "processed_clouds", "7raw.ply"))
        self.pcd = self.pcd.append(o3d.t.io.read_point_cloud(os.path.join(self.exp, "processed_clouds", "8raw.ply")))
        #self.pcd = self.pcd.append(o3d.t.io.read_point_cloud(os.path.join(self.exp, "processed_clouds", "9raw.ply")))
        #self.pcd = self.pcd.append(o3d.t.io.read_point_cloud(os.path.join(self.exp, "processed_clouds", "7raw.ply")))
        # self.pcd = self.pcd.append(o3d.t.io.read_point_cloud(os.path.join(self.exp, "processed_clouds", "8raw.ply")))
        # self.pcd = self.pcd.append(o3d.t.io.read_point_cloud(os.path.join(self.exp, "processed_clouds", "9raw.ply")))
        # self.pcd = self.pcd.append(o3d.t.io.read_point_cloud(os.path.join(self.exp, "processed_clouds", "10raw.ply")))
        
        self.image_fovs = []
        self.image_fovs.append(o3d.io.read_triangle_mesh("/home/oskar/smooth_sailing/data/long3/exp/journal/camera_fov/1725812066.08.ply"))
        self.image_fovs.append(o3d.io.read_triangle_mesh("/home/oskar/smooth_sailing/data/long3/exp/journal/camera_fov/1725811946.08.ply"))
        
        for mesh in self.image_fovs:
            mesh.paint_uniform_color((0, 0, 0))
        
        ts = t_get_channel(self.pcd, "timestamps")
        print(ts.min(), ts.max())
        # 1725811700.99948 1725811901.2992394
        #1725812101.5755522
        self.pcd = t_filter(self.pcd, "timestamps", min=1725811940, max=1725812090)
        
        #self.pcd = self.pcd.append(o3d.t.io.read_point_cloud(os.path.join(self.exp, "processed_clouds", "9raw.ply")))
        self.ice_geom = self.pcd.to_legacy()
        self.initialize_colors()

        # Convex hull
        self.show_seg_mesh = False
        self.seg_mesh = o3d.io.read_triangle_mesh(os.path.join(self.exp, "lidar_fov", "global.ply"))
        self.seg_mesh = self.seg_mesh.translate((0,0,0.01))

    
    def initialize_colors(self):
        self.c_intensity = t_channel_to_color(self.pcd, "intensities", cv.COLORMAP_COOL, 1, 99)
        self.c_topo = t_channel_to_color(self.pcd, "topo", cv.COLORMAP_VIRIDIS, 1, 99)

        

        self.c_rgb = o3d.utility.Vector3dVector(self.pcd.point.colors.numpy())
        self.colors = {
            "topo": self.c_topo,
            "rgb": self.c_rgb,
            "intensity": self.c_intensity,
        }


    def run(self):
        def toggle_seg_mesh(vis):
            render_option = vis.get_render_option()
            render_option.mesh_show_back_face = True
            
            if self.show_seg_mesh:
                vis.remove_geometry(self.seg_mesh, reset_bounding_box=False)
                self.show_seg_mesh = False
            else:
                vis.add_geometry(self.seg_mesh, reset_bounding_box=False)
                self.show_seg_mesh = True
            return False
        
        def toggle_lidar_mesh(vis):
            if self.show_lidar_mesh:
                vis.remove_geometry(self.current_lidar_mesh, reset_bounding_box=False)
                self.show_lidar_mesh = False
            else:
                vis.add_geometry(self.current_lidar_mesh, reset_bounding_box=False)
                self.show_lidar_mesh = True
            return False
    
        
        def toggle_background(vis):
            opt = vis.get_render_option()
            if np.allclose(np.asarray(opt.background_color), 0):
                opt.background_color = np.asarray([1, 1, 1])
            else:
                opt.background_color = np.asarray([0, 0, 0])
            return False
        
        def update_color(vis, color):
            opt = vis.get_render_option()
            opt.point_color_option = o3d.visualization.PointColorOption.Color
            self.ice_geom.colors = self.colors[color]
            vis.update_geometry(self.ice_geom)
            return False
        
        def color_option_topo(vis):
            return update_color(vis, "topo")
        
        def color_option_rgb(vis):
            return update_color(vis, "rgb")
        
        def color_option_intensity(vis):
            return update_color(vis, "intensity")

        
        def save_screen(vis):
            fname = os.path.join("/home/oskar/smooth_sailing/data/long3/exp/journal/images", "test.jpg")
            img = np.asarray(vis.capture_screen_float_buffer())*255#, cv.COLOR_BGR2RGB)
            cv.imwrite(fname, img)

            return False


        key_to_callback = {}
        key_to_callback[ord("1")] = color_option_topo
        key_to_callback[ord("2")] = color_option_rgb
        key_to_callback[ord("3")] = color_option_intensity
        key_to_callback[ord("B")] = toggle_background
        key_to_callback[ord("M")] = toggle_seg_mesh
        key_to_callback[ord("S")] = save_screen

        o3d.visualization.draw_geometries_with_key_callbacks([self.ice_geom]+self.image_fovs, key_to_callback)
        
if __name__ == "__main__":
    vis = PcdVisualizer()
    vis.run()