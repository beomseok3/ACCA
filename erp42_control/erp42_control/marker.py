import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point,PoseWithCovarianceStamped

class MarkerPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')
        
        self.sub_init = self.create_subscription(PoseWithCovarianceStamped,'initialpose',self.callback_init,qos_profile=10)
        self.publisher = self.create_publisher(MarkerArray, 'markerr_array', 10)
        self.timer = self.create_timer(0.5, self.publish_markers)
        self.marker_array = MarkerArray()
        # self.points = [(-51.32337188720703, -33.37735366821289), (-56.187355041503906, -45.85456466674805), (-44.98202133178711, -61.7526741027832), (3.1205594539642334, -63.8331184387207), (33.15129089355469, -56.067848205566406), (39.73629379272461, -21.59504508972168)]
        self.points = []
        
        self.i =0

        
    def callback_init(self,msg):
        # Define the points as provided
        x,y = msg.pose.pose.position.x, msg.pose.pose.position.y
        #Kcity
        # self.points = [(23.583086013793945, 44.68124771118164), (41.48026657104492, 77.4826431274414), (41.10720443725586, 99.65604400634766), (21.3468074798584, 110.56675720214844), (12.809849739074707, 131.71347045898438), (22.107276916503906, 150.3382568359375), (38.61094665527344, 154.302978515625), (59.60713577270508, 143.14654541015625), (78.80033874511719, 149.413818359375), (92.86607360839844, 175.26931762695312), (112.33516693115234, 211.57684326171875), (131.51490783691406, 257.3350524902344), (136.92379760742188, 280.112060546875), (137.53985595703125, 301.34765625), (138.3419189453125, 322.13037109375), (135.43121337890625, 384.5190734863281), (134.97653198242188, 411.4892578125), (108.131591796875, 433.099365234375), (83.99140167236328, 431.75396728515625), (67.6582260131836, 416.9241027832031), (70.56803131103516, 334.7068786621094), (90.34085845947266, 308.0325012207031), (118.09099578857422, 308.0494384765625), (128.59963989257812, 278.4043884277344), (108.31996154785156, 223.01483154296875), (93.47908020019531, 194.215576171875), (83.50749969482422, 169.2460479736328), (73.35724639892578, 150.46775817871094), (33.62983703613281, 75.23554229736328)]
        # self.points = [(-39.62443923950195, -62.97883605957031), (22.173192977905273, -69.15447235107422), (35.364715576171875, -59.505401611328125), (42.553321838378906, -18.44838523864746), (32.25053024291992, -1.0876102447509766), (-11.788719177246094, 3.980168104171753), (-47.4533828815921,	-14.7799070979004)]
        #school
        self.points.append((x,y))
        # Initialize markers
        for point in self.points:

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "markers"
            marker.id = self.i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.0

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.i+=1
            marker.color.a = 1.0

            self.marker_array.markers.append(marker)
        print(len(self.points))
        
        
    def publish_markers(self):
        for marker in self.marker_array.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.marker_array)
        print(self.points)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
