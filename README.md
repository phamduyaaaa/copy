# copy
```bash
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import sys
import select
import termios
import tty
import time


class GoalSequencer(Node):

    def __init__(self):
        super().__init__('goal_sequencer')

        self.navigator = BasicNavigator()

        self.goals = [
            (1.40, -0.14),
            (0.42, -0.26),
            (0.15, 0.71)
        ]

        self.get_logger().info('⏳ Waiting for Nav2...')
        self.navigator.waitUntilNav2Active()

        self.get_logger().info('🚀 Nav2 ACTIVE')


    def run(self):
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)

        try:
            while rclpy.ok():
                for x, y in self.goals:
                    self.get_logger().info(f'📍 Publish goal: ({x:.2f}, {y:.2f})')

                    goal = PoseStamped()
                    goal.header.frame_id = 'map'
                    goal.header.stamp = self.navigator.get_clock().now().to_msg()
                    goal.pose.position.x = x
                    goal.pose.position.y = y
                    goal.pose.orientation.w = 1.0

                    self.navigator.goToPose(goal)

                    while not self.navigator.isTaskComplete():
                        if self._key_pressed():
                            key = sys.stdin.read(1)
                            if key.lower() == 'q':
                                self.get_logger().warn('🛑 Stop requested by user')
                                self.navigator.cancelTask()
                                return
                        time.sleep(0.1)

                self.get_logger().info('🔁 Loop goals again')

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)


    def _key_pressed(self):
        return select.select([sys.stdin], [], [], 0)[0]


def main():
    rclpy.init()
    node = GoalSequencer()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```
