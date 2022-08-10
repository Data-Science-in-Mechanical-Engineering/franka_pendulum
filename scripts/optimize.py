#!/usr/bin/python3
from franka_pole.msg import Sample, CommandParameters
import rospy
import threading
import numpy as np
import sys

# Callback for receiving samples
def callback(sample):
    global cost, time, lock, condition
    lock.acquire()
    cost += (sample.pole_angle[0] ** 2)
    time += 0.01
    condition.notify_all()
    lock.release()

# Main
if __name__ == '__main__':
    # Read arguments
    namespace = "franka_pole"
    next_namespace = False
    for i in sys.argv:
        if next_namespace: namespace = i
        next_namespace = (i == "-N")

    # Initialize
    rospy.init_node(namespace + '_optimizer')
    lock = threading.Lock()
    condition = threading.Condition(lock)
    cost = 0.0
    time = 0.0
    sample_subscriber = rospy.Subscriber("/" + namespace + "/sample", Sample, callback)
    parameters_publisher = rospy.Publisher("/" + namespace + "/command_parameters", CommandParameters, queue_size=10)
    parameters = rospy.wait_for_message("/" + namespace + "/command_parameters", CommandParameters)

    # Setup starting point
    previous_parameters = np.array([6.41061247e+01, 1.57597186e+01, 12.00000000e+00, 18.67472718e+00])
    previous_cost = np.Infinity

    while True:
        # Publish
        print("Setting parameters", current_parameters)
        current_parameters = previous_parameters * (1 + 0.5 * (np.random.rand(4) - 0.5))
        parameters.control = np.concatenate((np.zeros(4), current_parameters))
        parameters_publisher.publish(parameters)
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
        
        # Update
        print("Cost: ", current_cost)
        if current_cost < previous_cost:
            previous_cost = current_cost
            previous_parameters = current_parameters