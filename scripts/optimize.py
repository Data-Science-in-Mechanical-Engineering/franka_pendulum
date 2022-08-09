#!/usr/bin/python3
import rospy
import numpy as np
from franka_pole.msg import Sample, CommandParameters
import threading

def callback(sample):
    global cost, time, lock, condition
    lock.acquire()
    cost += (sample.pole_angle[0] ** 2)
    time += 0.01
    condition.notify_all()
    lock.release()

if __name__ == '__main__':
    # Setup ROS
    rospy.init_node('reconfigure')
    lock = threading.Lock()
    condition = threading.Condition(lock)
    cost = 0.0
    time = 0.0
    subscriber = rospy.Subscriber("/franka_pole/sample", Sample, callback)

    # Setup logic
    previous_parameters = np.array([6.41061247e+01, 1.57597186e+01, 12.00000000e+00, 18.67472718e+00])
    previous_cost = np.Infinity

    # Read
    print("Waiting for previous parameters")
    parameters = rospy.wait_for_message("/franka_pole/command_parameters", CommandParameters)

    while True:
        # Edit
        print("Editing parameters")
        current_parameters = previous_parameters + previous_parameters * 0.25 * (np.random.rand(4) - 0.5)
        parameters.control = np.concatenate((np.zeros(4), current_parameters))
        print("Parameters:", current_parameters)

        # Publish
        print("Publishing new parameters")
        publisher = rospy.Publisher("/franka_pole/command_parameters", CommandParameters, queue_size=10)
        rospy.sleep(1.0)
        publisher.publish(parameters)
        rospy.sleep(1.0)

        # Measure
        print("Measuring performance")
        lock.acquire()
        cost = 0.0
        time = 0.0
        lock.release()
        while True:
            lock.acquire()
            condition.wait()
            end = (time > 10.0)
            lock.release()
            if end:
                current_cost = cost
                break
        print("Cost: ", current_cost)
        if current_cost < previous_cost:
            previous_cost = current_cost
            previous_parameters = current_parameters
    