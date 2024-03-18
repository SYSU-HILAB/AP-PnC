import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from rclpy.executors import MultiThreadedExecutor
import threading

cond = threading.Condition()

class Consumer(Node):
    def __init__(self):
        super().__init__('consumer')
        self.timer = self.create_timer(0.001, self.listener_callback)  # 20ms 对应 50Hz
        self.con_pub = self.create_publisher(String, 'cons_topic', 10)


    def listener_callback(self):
        msg = String()
        msg.data = 'Con: %s' % self.get_clock().now().to_msg()
        with cond:
              print(2)
              cond.wait()
        self.con_pub.publish(msg)

class Producer(Node):
    def __init__(self):
        super().__init__('producer')
        self.publisher_ = self.create_publisher(String, 'pro_topic', 10)
        self.timer = self.create_timer(1, self.timer_callback)  # 20ms 对应 50Hz

    def timer_callback(self):
        msg = String()
        msg.data = 'Pro: %s' % self.get_clock().now().to_msg()
        with cond:
          print(1)
          cond.notify_all()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    producer = Producer()
    consumer = Consumer()
    executor.add_node(producer)
    executor.add_node(consumer)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    producer.destroy_node()
    consumer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
