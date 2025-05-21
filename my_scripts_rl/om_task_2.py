import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
import tf

from pick_from_vision_V3 import ROS_OM_Node

class ContinuousOMNode:
    def __init__(self, joint_names, request_topic='/object_request', rate_hz=1):
        rospy.init_node('continuous_pick_and_place', anonymous=True)
        # Publisher to request object detection
        self.request_pub = rospy.Publisher(request_topic, String, queue_size=1)
        self.listener = tf.TransformListener()
        # Instantiate the motion executor
        self.om_node = ROS_OM_Node(joint_names)
        self.rate = rospy.Rate(rate_hz)

    def get_object_pose_world(self,target_frame="world", object_frame="detected_object"):
            try:
                self.listener.waitForTransform(target_frame, object_frame, rospy.Time(0), rospy.Duration(15.0))
                (trans, rot) = self.listener.lookupTransform(target_frame, object_frame, rospy.Time(0))

                rospy.loginfo("Object position in %s frame:", target_frame)
                rospy.loginfo("  Translation: x=%.3f, y=%.3f, z=%.3f", *trans)
                rospy.loginfo("  Orientation (quaternion): x=%.3f, y=%.3f, z=%.3f, w=%.3f", *rot)

                return np.array(trans), np.array(rot)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr("Transform failed: %s", str(e))
                return None, None

    def run_set_pick(self):
        rospy.loginfo("Manual pick-and-place node started.")
        while not rospy.is_shutdown():
            try:
                user_input = input("Enter pick location as 'x y z' (or 'exit' to quit): ")
            except EOFError:
                break

            if not user_input or user_input.lower() == 'exit':
                rospy.loginfo("Shutting down.")
                break

            try:
                x, y, z = map(float, user_input.strip().split())
                pick_position = np.array([x, y, z])
            except ValueError:
                rospy.logwarn("Invalid input format. Please enter three space-separated numbers (e.g., '0.4 0.2 0.1').")
                continue

            rospy.loginfo(f"Received pick position: {pick_position}")

            try:
                formatted_str = f"{x},{y},{z}"
            except ValueError:
                rospy.logwarn("Invalid input. Please enter 3 floats separated by spaces.")
                continue

            # Publish string
            self.request_pub.publish(formatted_str)
            rospy.loginfo(f"Published: {formatted_str}")

            # Define a place position (e.g., offset by 20 cm in x)
            place_position = np.array([0.12, -0.12, 0.0])

            # Execute pick and place
            self.om_node.execute_pick_and_place(pick_position,
                                                place_position,
                                                np.deg2rad(60),
                                                np.deg2rad(60)
                                                )

            self.rate.sleep()

    def get_object_trans(self,obj_name):
        # Publish detection request
        self.request_pub.publish(obj_name)
        rospy.loginfo(f"Requested detection for object: '{obj_name}'")

        # Wait for tf to be available
        trans, rot = None, None
        while not rospy.is_shutdown():
            trans, rot = self.get_object_pose_world()
           # trans, rot = self.get_object_pose_world(object_frame=obj_name)
            if trans is not None:
                return trans
            rospy.loginfo(f"Waiting for transform for '{obj_name}'...") 
            rospy.sleep(0.5)

    def run_set_object(self):
        rospy.loginfo("Continuous pick-and-place node started.")
        while not rospy.is_shutdown():
            try:
                # Read object frame name from console
                obj_name = input("Enter object frame name (or 'exit' to quit): ")
            except EOFError:
                break

            if not obj_name or obj_name.lower() == 'exit':
                rospy.loginfo("Shutting down.")
                break

            trans = self.get_object_trans(obj_name)
            print("Trans:", trans)
            try:
                # Read object frame name from console
                obj_name = input("OK?: ")
            except EOFError:
                break

            # Execute pick and place
            pick_position = np.array(trans)
            pick_position = pick_position + np.array([0.0 0.02, 0.04]) # set offset
            #self.om_node.execute_pick(pick_position)

            # Define a place position or reuse pick_position
            #place_position = self.get_object_trans("two")
            place_position = np.array([0.12, -0.12, 0.0])
            place_position = place_position + np.array([0.0, 0.0, 0.02]) # set offset
            #self.om_node.execute_place(place_position)

            self.om_node.execute_pick_and_place(pick_position,
                                                place_position,
                                                np.deg2rad(40),
                                                np.deg2rad(40)
                                                )

            # Small delay before next iteration
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Adjust joint names as needed
        joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
        node = ContinuousOMNode(joint_names)
        node.run_set_object()
        #node.run_set_pick()
    except rospy.ROSInterruptException:
        pass
