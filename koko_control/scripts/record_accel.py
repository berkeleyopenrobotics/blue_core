#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from koko_hardware_drivers.msg import MotorState
import numpy as np
import tf.transformations as transformations

UPDATE_FREQ = 20
EXP_CONST = 0.99

global x_accum
global y_accum
global z_accum
global num

x_accum = [0.0, 0.0]
y_accum = [0.0, 0.0]
z_accum = [0.0, 0.0]
num = [0,0]

# def rotation_matrix(axis, theta):
#     axis = np.asarray(axis)
#     axis = axis/math.sqrt(np.dot(axis, axis))
#     a = math.cos(theta/2.0)
#     b, c, d = -axis*math.sin(theta/2.0)
#     aa, bb, cc, dd = a*a, b*b, c*c, d*d
#     bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
#     return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
#                      [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
#                      [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])

def get_accel(msg):
    global x_accum
    global y_accum
    global z_accum
    global num
    right_name = 'right_motor'
    left_name = 'left_motor'
    index = -1
    loc = -1
    for i, name_test in enumerate(msg.name):
        if right_name == name_test:
            index = i
            loc = 0
        if left_name == name_test:
            index = i
            loc = 1
            # print(name_test)

        acc_vect = msg.accel[index]
        x_acc = acc_vect.x
        y_acc = acc_vect.y
        z_acc = acc_vect.z
        if num[loc] == 0:
            num[loc] += 1
            x_accum[loc] = x_acc
            y_accum[loc] = y_acc
            z_accum[loc] = z_acc
        else:
            num[loc] += 1
            x_accum[loc] = x_accum[loc] * EXP_CONST + x_acc * (1.0 - EXP_CONST)
            y_accum[loc] = y_accum[loc] * EXP_CONST + y_acc * (1.0 - EXP_CONST)
            z_accum[loc] = z_accum[loc] * EXP_CONST + z_acc * (1.0 - EXP_CONST)


def find_optimal_transform(p, q):

    # de-mean the points to calculate the rotation
    p_mean = np.mean(p, axis=0)
    q_mean = np.mean(q, axis=0)

    # SVD of covariance
    cov = np.dot((p - p_mean).T, (q - q_mean))
    U, S, VT = np.linalg.svd(cov, full_matrices=True)

    # negative determinant => reflection => needs fixing
    rectify = np.identity(U.shape[0])
    rectify[-1][-1] = np.linalg.det(np.dot(VT.T, U.T))

    # compute R and t
    R = VT.T.dot(rectify).dot(U.T)
    t = q_mean - np.dot(R, p_mean)

    # compute maximum error
    error = 0.0
    for pi, qi in zip(p, q):
        qi_pred = np.dot(R, pi) + t
        error = max(error, np.linalg.norm(qi - qi_pred))

    # R matrix => homogeneous transform => quaternion
    R = np.hstack((R, np.zeros((3, 1))))
    R = np.vstack((R, np.zeros((1, 4))))
    R[-1, -1] = 1.0
    q = transformations.quaternion_from_matrix(R)

    return t, q, R, error

#################################################################################################

def main():
    global x_accum
    global y_accum
    global z_accum
    global num

    rospy.init_node('acc_recorder', anonymous=True)
    rospy.Subscriber("koko_hardware/motor_states", MotorState, get_accel, queue_size=1)
    grav_pub0 = rospy.Publisher("koko_hardware/gravity0", Vector3, queue_size=1)
    grav_pub1 = rospy.Publisher("koko_hardware/gravity1", Vector3, queue_size=1)
    r = rospy.Rate(UPDATE_FREQ)

    while not rospy.is_shutdown():

        #rospy.logerr("{}".format(num))
        #rospy.logerr("{}".format(num))
        #axis = [0.0, 0.0, 1.0]
        # correction z rotation
        #theta = 4.301 - np.pi
        # best_z = np.linalg.inv(rotation_matrix(axis, theta))
        #best_z = transformations.rotation_matrix(-theta, axis)[:3,:3]

        #print 'iteration {}'.format(num)
        #print 'x_accum: {}'.format(x_accum)
        #print 'y_accum: {}'.format(y_accum)
        #print 'z_accum: {}'.format(z_accum)

        # Find transform
        raw_right = np.array([x_accum[0],y_accum[0],z_accum[0]])
        # Rotate left raw into right frame
        axis = [0.0, 0.0, 1.0]
        theta = np.pi
        correction_transform = transformations.rotation_matrix(theta, axis)[:3,:3]
        raw_left = np.array([[x_accum[1]],[y_accum[1]],[z_accum[1]]])
        raw_left = correction_transform.dot(raw_left)

        axis = [1.0, 0.0, 0.0]
        theta = np.pi
        correction_transform = transformations.rotation_matrix(theta, axis)[:3,:3]
        raw_left = correction_transform.dot(raw_left)

        raw = (raw_right + raw_left.T[0]) / 2.0

        y_raw = [938.42967069, 332.7497597, -21.65929025]
        z_raw = [14.13468672, -24.36888851, 986.29506329]
        x_raw = [245.41154232, -966.56424899, -19.55734769]
        x_raw2 = [-237.13234641, 948.83481383, 19.7845224]
        z_raw = z_raw / np.linalg.norm(z_raw)
        y_raw = y_raw / np.linalg.norm(y_raw)
        x_raw = x_raw / np.linalg.norm(x_raw)
        x_raw2 = x_raw2 / np.linalg.norm(x_raw2)
        
        p = np.array([y_raw, z_raw, x_raw, x_raw2])
        q = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0], [-1, 0, 0]])
        t, q, R, err = find_optimal_transform(p, q)
        R = R[:3,:3]
        print(R)
        print(err)
        g = R.dot(raw)
        grav_msg = Vector3()
        grav_msg.x = g[0]
        grav_msg.y = g[1]
        grav_msg.z = g[2]
        grav_pub0.publish(grav_msg)

        transform = np.array([[ 0.26860026, -0.96283056, -0.02848168], [ 0.96299097,  0.2690981,  -0.01531682], [ 0.02241186, -0.0233135,   0.99947696]])


        # right
        # raw = np.array([[x_accum[0]],[y_accum[0]],[z_accum[0]]])
        # grav_msg = Vector3()
        # grav_msg.x = raw[0]
        # grav_msg.y = raw[1]
        # grav_msg.z = raw[2]
        # grav_pub0.publish(grav_msg)

        # # left
        # axis = [0.0, 0.0, 1.0]
        # theta = np.pi
        # correction_transform = transformations.rotation_matrix(theta, axis)[:3,:3]
        # raw = np.array([[x_accum[1]],[y_accum[1]],[z_accum[1]]])
        # raw = correction_transform.dot(raw)

        # axis = [1.0, 0.0, 0.0]
        # theta = np.pi
        # correction_transform = transformations.rotation_matrix(theta, axis)[:3,:3]
        # raw = correction_transform.dot(raw)

        # grav_msg = Vector3()
        # grav_msg.x = raw[0]
        # grav_msg.y = raw[1]
        # grav_msg.z = raw[2]
        # grav_pub1.publish(grav_msg)
        # print("published")
        r.sleep()

if __name__ == '__main__':
    main()
