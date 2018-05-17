import rospy
import numpy as np
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point

GPS_FREQUENCY = 50 # [BroadcastsPerSecond]
MODEL_NAME = "Marker"

def apply_noise(noise_free_point):
    # Noise model based on wikipedia saying that GPS accuracy is 5 meters
    # Assuming that what they mean is 99% Spherical Accuracy Standard = 5 [m]
    # 99SAS = 1.122(sigma_x + sigma_y + sigma_z) = 5 [m]
    # http://www.novatel.com/assets/Documents/Bulletins/apn029.pdf

    # Also standard deviation for Z axis is gonna be higher, due to less
    # variance in satelites heights; sigma_z = 2*sigma_x = 2*sigma_y.

    # Constant offset is a result of the receiver not being aligned with
    # a marker perfectly; offset has been chosen arbitrarily.

    offset_x = 0.2
    offset_y = 0.7
    offset_z = 1.2

    sigma_x = 5/(1.122*4)
    sigma_y = sigma_x
    sigma_z = 2*sigma_x

    noisy_point = noise_free_point

    noisy_point.x = np.random.normal(noise_free_point.x,sigma_x,1) + offset_x
    noisy_point.y = np.random.normal(noise_free_point.y,sigma_y,1) + offset_y
    noisy_point.z = np.random.normal(noise_free_point.z,sigma_z,1) + offset_z

    return noisy_point

def get_position(blockName="Marker", relative_entity_name="link"):
    try:
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates(blockName, relative_entity_name)
        point = Point(resp_coordinates.pose.position.x,
	              resp_coordinates.pose.position.y,
    		      resp_coordinates.pose.position.z)

    except rospy.ServiceException as e:
        rospy.loginfo("Get Model State service call failed: {0}".format(e))

    return point

def gps_position_publishers():
    pos_pub = rospy.Publisher('/marker_model/gps_position', Point, queue_size=10)
    noisy_pos_pub = rospy.Publisher('/marker_model/gps_position_noised', Point, queue_size=10)
    rate = rospy.Rate(GPS_FREQUENCY)

    while not rospy.is_shutdown():
	point = get_position(MODEL_NAME)
	pos_pub.publish(point)
	noisy_pos_pub.publish(apply_noise(point))
	rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('bus_model', anonymous=True)
        gps_position_publishers()
    except rospy.ROSInterruptException:
        pass
