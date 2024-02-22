import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import threading

class DataVisualizer(Node):
    def __init__(self):
        super().__init__('data_visualizer')
        self.subscription = self.create_subscription(
            Float32,
            'xxxSpeed',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.data = []

        # Set up the plot
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot(self.data)
        self.ax.set_ylim([0, 10])  # Set y-axis limits

        # Start a separate thread for the plot
        self.plot_thread = threading.Thread(target=self.plot_data)
        self.plot_thread.start()

    def listener_callback(self, msg):
        self.data.append(msg.data)
        if len(self.data) > 50:  # Limit the size of data
            self.data.pop(0)

    def plot_data(self):
        while rclpy.ok():
            self.line.set_ydata(self.data)
            self.line.set_xdata(range(len(self.data)))
            self.ax.relim()
            self.ax.autoscale_view()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    visualizer = DataVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
