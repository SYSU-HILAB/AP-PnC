import threading
import subprocess
import datetime
import os
import multiprocessing
import time
import argparse
def run_px4_sim(env, tailsitter=False):
  PX4_ROOT = os.getenv('PX4_ROOT')
  os.chdir(PX4_ROOT)
  # subprocess.run(['export', 'HEADLESS=1'])
  if tailsitter:
    subprocess.run(['make', 'px4_sitl', 'gz_tailsitter'], env= env)
  else:
    subprocess.run(['make', 'px4_sitl', 'gz_x500'], env= env)


def com_udp():
  # print(os.curdir())
  subprocess.run(['MicroXRCEAgent', 'udp4', '-p', '8888'])

def com_serial():
  # print(os.curdir())
  subprocess.run(['MicroXRCEAgent', 'udp4', '-p', '8888'])

def sim_time_on():
  subprocess.run(['ros2', 'run', 'com_gazebo', 'com_gazebo'])

# def takeoff():
#   time.sleep(10)
#   subprocess.run("cd /home/rec/ws_sensor_combined/ && source ./install/local_setup.bash && ros2 run px4_ros_com auto_takeoff",shell=True,executable='/bin/bash')

def connection():
  subprocess.run(['conn'])

def qgc_on():
  subprocess.run(['qgc'])


if __name__ == "__main__":
  pid = os.getpid()
# 设置进程的CPU亲和性，限制在CPU核心0和1上
  os.sched_setaffinity(pid, {0, 1, 2})
  parser = argparse.ArgumentParser(description='Autostart All your PX4 environment')
  parser.add_argument('-s', '--serial', help='Serial Link to PX4', action='store_true')
  parser.add_argument('-c', '--connection', help='Connection to ROS2', action='store_true')
  parser.add_argument('-q', '--qgc', help='If turn on qgc', action='store_true')
  parser.add_argument('-n', '--no-gui', help='turn off gazebo gui', action='store_true')
  parser.add_argument('-t', '--sim-time', help='turn on gazebo sim time for ros2', action='store_true')
  parser.add_argument('-tailsitter', help='if airframe is tailsitter ', action='store_true')
  args = parser.parse_args()
  pool = multiprocessing.Pool(processes=3)

  serieal = args.serial
  qgc = args.qgc
  no_gui = args.no_gui
  sim_time = args.sim_time
  conn = args.connection
  tailsitter = args.tailsitter
  env = os.environ.copy()
  if no_gui:
    env['HEADLESS'] = "1"

  if serieal:
    print("serial not setup, TODO!")

  if qgc:
    pool.apply_async(qgc_on)
  if sim_time:
    pool.apply_async(sim_time_on)
  if conn:
    pool.apply_async(connection)
  pool.apply_async(com_udp)
  pool.apply_async(run_px4_sim(env, tailsitter))
    # pool.apply_async(qgc_on)
    # pool.apply_async(takeoff)
  pool.close()
  pool.join()
