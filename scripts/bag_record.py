mport os
import datetime
import subprocess

def create_output_directory():
        # Get the current time
            now = datetime.datetime.now()
                
                    # Format the timestamp to include only hours and minutes
                        timestamp = now.strftime("%H-%M")
                            
                                # Create the directory path
                                    output_dir = os.path.join('/path/to/base/directory', timestamp)
                                        
                                            # Create the directory if it doesn't exist
                                                os.makedirs(output_dir, exist_ok=True)
                                                    
                                                        return output_dir

                                                    def record_ros2_bag(output_dir, topics):
                                                            # Prepare the command to record the ROS 2 bag
                                                                command = ['ros2', 'bag', 'record', '-o', output_dir] + topics
                                                                    
                                                                        # Execute the command
                                                                            subprocess.run(command)

                                                                            if __name__ == "__main__":
                                                                                    # Define the topics you want to record
                                                                                        topics = ['/topic1', '/topic2']
                                                                                            
                                                                                                # Create the output directory with the custom timestamp format
                                                                                                    output_dir = create_output_directory()
                                                                                                        
                                                                                                            # Record the ROS 2 bag file into the created directory
                                                                                                                record_ros2_bag(output_dir, topics)
                                                                                                    
