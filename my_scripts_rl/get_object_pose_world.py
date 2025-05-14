#!/usr/bin/env python3
import rospy
import tf

def get_object_pose_world(target_frame="world", object_frame="detected_object"):
    rospy.init_node('get_object_pose_world', anonymous=True)
    listener = tf.TransformListener()

    rospy.loginfo("Waiting for transform from %s to %s...", target_frame, object_frame)

    try:
        listener.waitForTransform(target_frame, object_frame, rospy.Time(0), rospy.Duration(10.0))
        (trans, rot) = listener.lookupTransform(target_frame, object_frame, rospy.Time(0))

        rospy.loginfo("Object position in %s frame:", target_frame)
        rospy.loginfo("  Translation: x=%.3f, y=%.3f, z=%.3f", *trans)
        rospy.loginfo("  Orientation (quaternion): x=%.3f, y=%.3f, z=%.3f, w=%.3f", *rot)

        return trans, rot

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Transform failed: %s", str(e))
        return None, None

if __name__ == '__main__':
    get_object_pose_world()