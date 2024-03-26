import message_filters
from rclpy.node import Node
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
class ExampleNode(Node):
    def __init__(self):
        super().__init__("ExampleNode")
        self.image_sub = message_filters.Subscriber(self, Image, "/image_topic")
        self.plc_sub = message_filters.Subscriber(
            self, PointCloud2, "/point_cloud_topic", qos_profile=qos_profile_sensor_data
        )
        queue_size = 30
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.plc_sub], queue_size)
        self.ts.registerCallback(self.callback)
        # or you can use ApproximateTimeSynchronizer if msgs dont have exactly the same timestamp
        # self.ts = message_filters.ApproximateTimeSynchronizer(
        #     [self.image_sub, self.plc_sub],
        #     30,
        #     0.01,  # defines the delay (in seconds) with which messages can be synchronized
        # )
    def callback(self, image_msg, pcl_msg):
        # do your stuff here
        pass
        
def main(args=None):
    print('visual_tracker')
    rclpy.init(args=args)
    exampleNode = ExampleNode()
    print('visualTracker init done')
    try:
        rclpy.spin(exampleNode)
    finally:
        exampleNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
