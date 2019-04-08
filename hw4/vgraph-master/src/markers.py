import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def main():
    # Create ROS node
    rospy.init_node('marker_test', anonymous=False)

    # Publish markers to 'marker_test' topic
    pub_node = rospy.Publisher('marker_test', MarkerArray, queue_size=10)

    # Initialize MarkerArray
    marker_arr = MarkerArray()

    # Initialize Marker
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.ns = "nsl"
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0

    marker.type = Marker.LINE_LIST
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.color.g = 1.0 # green
    marker.color.a = 1.0 # alpha

    # Create points
    points = []
    pt1 = Point(1, 1, 1)
    pt2 = Point(2, 2, 2)
    points.append(pt1)
    points.append(pt2)

    marker.points = points

    marker_arr.markers.append(marker)

    # Draw markers
    while not rospy.is_shutdown():
        pub_node.publish(marker_arr)
        rospy.Rate(30).sleep()

if __name__ == '__main__':
    try:
        main()
    except:
        rospy.loginfo('Failed')

