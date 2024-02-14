import cv2
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R





class PointCloudProjector(Node):
    def __init__(self):
        super().__init__('point_cloud_projector')

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.image_sub = self.create_subscription(Image, '/resize/image', self.image_callback, 10)
        self.pc_sub = self.create_subscription(PointCloud2, '/velodyne_points', self.pc_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/zed/zed_node/rgb/camera_info', self.camera_info_callback, 10)
        
        # Publisher for the overlaid image
        self.image_pub = self.create_publisher(Image, '/projected_image', 10)

        # Store the latest messages
        self.latest_image = None
        self.camera_info = None



    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def camera_info_callback(self, msg):
        self.camera_info = msg




    def pc_callback(self, point_cloud_msg):
        if self.latest_image is None:
            self.get_logger().error('No image data received yet.')
            return
        if self.camera_info is None:
            self.get_logger().error('No camera info received yet.')
            return

        # Get transformation from camera to lidar
        try:
            trans = self.tf_buffer.lookup_transform('zed_left_camera_optical_frame', 'velodyne', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('Could not transform LiDAR to Camera: ' + str(e))
            return
    
        # Extract camera intrinsic parameters
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        height = self.camera_info.height
        width = self.camera_info.width
        #print(height, width)
        # Convert PointCloud2 to array
        pc_array = pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        fi = []
        fi = np.arctan(pc_array['y']/pc_array['x'])
        
        # Assuming pc_array is a numpy array
        filtered_pc_array = pc_array[(pc_array['x'] >= 0) & (fi <= np.pi/3) & (fi >= -np.pi/3) ]
        # bekreftelse på at mattematikken er korrekt 
        #print((np.arctan(filtered_pc_array['y']/filtered_pc_array['x']))*180/np.pi, 'cordinates', 'filterd pc array', (filtered_pc_array['x']), (filtered_pc_array['y']))

        projected_img = self.latest_image.copy()

        distance = 0
        for point in filtered_pc_array:
                # Convert point coordinates to float
            point = (float(point[0]), float(point[1]), float(point[2]))
            #x, y, z = (point['x']), (point['y']), (point['z'])
            distance = np.sqrt(point[0]**2+point[1]**2+point[2]**2)
            colour = min(distance*100, 255)
            #print(colour)
            # Transform point to camera frame
            """transformation_matrix = [
                [0, -1, 0, 0.009999999999999978],
                [0, 0, -1, -0.172000000000000007],
                [1, 0, 0, -0.12000000000000001],
                [0, 0, 0, 1]
            ]"""
            # 0.06000000000000001 0.016000000000000007 0.009999999999999978
            # Define the rotation matrix and translation vector
            rotation_matrix = [
                [0, -1, 0],
                [0, 0, -1],
                [1, 0, 0]
            ]
            translation_vector = [0.08, 0.06, 0.01]
            # 0.08 0.06 0.01


            # Apply rotation
            rotated_point = [sum(p * m for p, m in zip(point, row)) for row in rotation_matrix]

            # Apply translation
            transformed_point = [rp + tv for rp, tv in zip(rotated_point, translation_vector)]

            # Now, translated_point contains the final transformed coordinates

            
            #transformed_point = [sum(p * m for p, m in zip(point, row)) for row in transformation_matrix]
            # This will give you the transformed point in the form [x', y', z', 1]

            #rotation_matrix = [(1,0,0),
            #                   (0,-1,0),
            #                   (0,0,-1)]
            #tranlation_vector_matrix = [(0,0,0)]
            #rotated_point = [sum(p * m for p, m in zip(point, row)) for row in transformation_matrix]
            #translated_points = rotation_matrix
            #print(rotated_point)
            #point_camera_frame = self.transform_point(trans, point)
            #print(point_camera_frame)
            # Project to 2D
            x_2d = (fx * transformed_point[0]) / transformed_point[2] + cx
            y_2d = (fy* transformed_point[1]) / transformed_point[2] + cy
            #print(x_2d,y_2d)
            #x_2d = (fx * point_camera_frame[0]) / (point_camera_frame[2]) + cx
            #y_2d = (fy * point_camera_frame[1]) / (point_camera_frame[2]) + cy
            #print(x_2d, y_2d)
            # Check if the point is within the camera's field of view
            if 0 <= x_2d  < width and 0 <= y_2d < height:
                cv2.circle(projected_img, (round(x_2d), round(y_2d)), 1, (colour, 255, 0), -1)

        # Publish the overlaid image
        overlaid_img_msg = self.bridge.cv2_to_imgmsg(projected_img, "bgr8")
        self.image_pub.publish(overlaid_img_msg)

        # Optionally display the image
        # cv2.imshow("Projected Image", projected_img)
        # cv2.waitKey(1)

    

    def transform_point(self, trans, point):
    # Create a rotation object from the quaternion
        r = R.from_quat([
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        ])
        #print(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w, 'resultat', str(r.as_matrix()))
        # Apply rotation to the point
        rotated_point = r.apply(point)
        
        # Apply translation
        translated_point = rotated_point + np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        ])
        print(trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z)
        #translated_point[0] = -translated_point[0]
        
        print(translated_point.tolist(), "trans", trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z)
        return translated_point.tolist()

def main(args=None):
    rclpy.init(args=args)
    projector = PointCloudProjector()
    rclpy.spin(projector)
    projector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
