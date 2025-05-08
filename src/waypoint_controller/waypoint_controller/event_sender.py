#!/usr/bin/env python3

from geometry_msgs.msg import PointStamped


def main():
    node = rclpy.Node()

    while True:
        x = input("x: ")
        y = input("y: ")
        z = input("z: ")
        robot_id_1 = input("robot_id 1: ")
        robot_id_2 = input("robot_id 2: ")
        
        pub1 = node.create_publisher()
        pub2 = node.create_publisher()

        msg = PointStamped()
        pub1.publish(msg)
        pub2.publish(msg)



if __name__ == '__main__':
    main()
