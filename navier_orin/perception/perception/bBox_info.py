from vision_msgs.msg import Detection2DArray
import rclpy
from rclpy.node import Node

class bBoxClass(Node):
    def __init__(self):
        super().__init__('Bbox_test')

        # Create subscriptions 
        self.bBox = self.create_subscription(Detection2DArray, '/detections_output', self.bBox_callback, 10)


    def bBox_callback(self, msg):
        for detection in msg.detections:
            for result in detection.results:
                class_id = result.hypothesis.class_id
                score = result.hypothesis.score  # If you also want to print the score
                
                bbox_center_x = detection.bbox.center.position.x
                bbox_center_y = detection.bbox.center.position.y
                bbox_theta = detection.bbox.center.theta
                bbox_size_x = detection.bbox.size_x
                bbox_size_y = detection.bbox.size_y
                
                print(f"Class ID: {class_id}, Score: {score}")
                print(f"Bounding Box Center: ({bbox_center_x}, {bbox_center_y}), Theta: {bbox_theta}")
                print(f"Bounding Box Size: {bbox_size_x} x {bbox_size_y}\n")

def main(args=None):
    rclpy.init(args=args)
    bBox_infoamtion = bBoxClass()
    rclpy.spin(bBox_infoamtion)
    bBox_infoamtion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
