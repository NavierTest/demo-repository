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
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from geometry_msgs.msg import Pose2D  # Ensure this import is correct
import statistics
from scipy import stats
#from sklearn.cluster import DBSCAN



class PointCloudProjector(Node):
    def __init__(self):
        super().__init__('point_cloud_projector')

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.image_sub = self.create_subscription(Image, '/resize/image', self.image_callback, 10)
        self.pc_sub = self.create_subscription(PointCloud2, '/velodyne_points', self.pc_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/resize/camera_info', self.camera_info_callback, 10)
        self.bBox = self.create_subscription(Detection2DArray, '/detections_output', self.bBox_callback, 10)

        # Publisher for the overlaid image
        self.image_pub = self.create_publisher(Image, '/projected_image', 10)

        # Store the latest messages
        self.latest_image = None
        self.camera_info = None
        self.bounding_boxes = []



    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    
    def bBox_callback(self, msg):
        self.bounding_boxes = []  # Reset bounding boxes
        for detection in msg.detections:
            bbox = detection.bbox
            
            center = bbox.center  # This is a Pose2D object
           
            center_x = bbox.center.position.x
            center_y = bbox.center.position.y
            size_x = bbox.size_x
            size_y = bbox.size_y
            # Calculate the bounding box edges
            x_min = center_x - size_x / 2
            y_min = center_y - size_y / 2
            x_max = center_x + size_x / 2
            y_max = center_y + size_y / 2

            self.bounding_boxes.append({
                'x_min': x_min,
                'y_min': y_min,
                'x_max': x_max,
                'y_max': y_max
            })




    def pc_callback(self, point_cloud_msg):
        if self.latest_image is None:
            self.get_logger().error('No image data received yet.')
            return
        if self.camera_info is None:
            self.get_logger().error('No camera info received yet.')
            return
        if not self.bounding_boxes:
            self.get_logger().info('No bounding boxes available.')
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
        #fx = 354.415587
        #fy = 354.977832
        #cx = 493.095981
        #cy = 304.126669
        coords_2d_Obj = []
        for point in filtered_pc_array:
            # Convert point coordinates to float
            point = (float(point[0]), float(point[1]), float(point[2]))
            #x, y, z = (point['x']), (point['y']), (point['z'])
            #distance = np.sqrt(point[0]**2+point[1]**2+point[2]**2)
            #colour = min(distance*100, 255)
        
            rotation_matrix = [
                [0, -1, 0],
                [0, 0, -1],
                [1, 0, 0]
            ]
            translation_vector = [0, 0, 0.2]

            # Apply rotation
            rotated_point = [sum(p * m for p, m in zip(point, row)) for row in rotation_matrix]

            # Apply translation
            transformed_point = [rp + tv for rp, tv in zip(rotated_point, translation_vector)]

            x_2d = (fx * transformed_point[0]) / transformed_point[2] + cx
            y_2d = (fy* transformed_point[1]) / transformed_point[2] + cy
            #coords_2d_Bbox = []
            #trans_and_ans_point = [(x_2d,y_2d,(point[0]),(point[1]),(point[2]))]

            for bbox in self.bounding_boxes:
                if bbox['x_min'] <= x_2d <= bbox['x_max'] and bbox['y_min'] <= y_2d <= bbox['y_max']:
                    # Calculate distance to the original point
                    distance = np.sqrt(sum(p**2 for p in point))
                    coords_2d_Obj.append((x_2d, y_2d, distance))


            # Collect all depth values
        depths = [depth for _, _, depth in coords_2d_Obj]

        # Round depths to a certain precision to find the mode effectively
        rounded_depths = np.round(depths, decimals=2)  # Adjust the precision as needed
        
        # Check if there are any depth values to process
        if len(rounded_depths) > 0:
            # Find the mode depth value
            mode_depth = stats.mode(rounded_depths)[0][0]

            # Define the threshold for filtering based on mode depth
            depth_threshold = 0.25

            # Filter points based on their proximity to the mode depth value
            filtered_coords = [(x, y, depth) for x, y, depth in coords_2d_Obj if abs(depth - mode_depth) <= depth_threshold]

            if filtered_coords:
                # Process your filtered coordinates as before
                print(filtered_coords[0])
                for x_2d, y_2d, _ in filtered_coords:
                    cv2.circle(projected_img, (round(x_2d), round(y_2d)), 1, (0, 255, 0), -1)


        """ # Draw circles for points within the threshold
            for x_2d_ob, y_2d_ob, distance_ob in coords_2d_Obj:
                if distance_ob <= min_dist + 0.15:
                    cv2.circle(projected_img, (round(x_2d_ob), round(y_2d_ob)), 1, (0, 255, 0), -1)"""

                    


                
        """distance_array = []
            for bbox in self.bounding_boxes:
                #filter points that hits the object
                distance_array = []
                if bbox['x_min'] <= x_2d <= bbox['x_max'] and bbox['y_min'] <= y_2d <= bbox['y_max']:
                    # Calculate distance to the point within the bounding box
                    distance = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
                    print(distance)
                    #distance_array.append(distance)
                    #min_dist = min(distance_array)
                    #for i in distance_array:
                        #object_dist = []
                        #if distance_array[i] <= min_dist + 0.05:
                            #object_dist.append(distance_array[i])
                    
                    #avrage_dist = statistics.mean(object_dist) 
                    #print(avrage_dist)
                    

                    color = min(distance*100, 255)
                    # Mark the point on the projected image
                    cv2.circle(projected_img, (int(x_2d), int(y_2d)), 1, (0, 255, color), -1)
                    # Optionally, calculate and store distances for more detailed analysis      

            #if 0 <= x_2d  < width and 0 <= y_2d < height:
                #cv2.circle(projected_img, (round(x_2d), round(y_2d)), 1, (colour, 255, 0), -1)"""
            


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

    return translated_point.tolist()

def main(args=None):
    rclpy.init(args=args)
    projector = PointCloudProjector()
    rclpy.spin(projector)
    projector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
