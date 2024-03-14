from utils.quaternions import q_dot_q, q_to_rot_mat
import numpy as np

# since enu - ned bi-transform is the same ,
q_FLU_to_FRD = q_FRD_to_FLU = np.array([0., 1, 0, 0])
q_ENU_to_NED = q_NED_to_ENU = np.array([0., 0.707106781186548, 0.707106781186548, 0.])

vector_len = 3
quaternion_len = 4

# it's terribly bad design, since attitude trans is not rude

quaternion_tailsitter = np.array([0.0, 0.70710678, 0.0, 0.70710678])

def FRDtoFLU(point:np.ndarray):
    assert point.shape[0] ==  vector_len 
    return q_to_rot_mat(q_FRD_to_FLU) @ point

def FLUtoFRD(point:np.ndarray):
    assert point.shape[0] ==  vector_len 
    return q_to_rot_mat(q_FLU_to_FRD) @ point


def NEDtoENU(point:np.ndarray):
    assert point.shape[0] ==  vector_len or point.shape[0] == quaternion_len
    if point.shape[0] == 3:
      return q_to_rot_mat(q_NED_to_ENU) @ point
    else:
      return q_dot_q(q_dot_q(q_NED_to_ENU, point) , q_FRD_to_FLU)

def ENU_to_NED(point:np.ndarray):
    assert point.shape[0] ==  vector_len or point.shape[0] == quaternion_len
    if point.shape[0] == 3:
      return q_to_rot_mat(q_ENU_to_NED) @ point
    else:
       return q_dot_q(q_dot_q(q_NED_to_ENU, point) , q_FLU_to_FRD)
    


def tailsiiter_pos_or_vel(point:np.ndarray):
  return q_to_rot_mat(quaternion_tailsitter) @ point

def tailsiiter_quat(point:np.ndarray):
  return q_dot_q(point, quaternion_tailsitter)

def tailsitter_rate(point:np.ndarray):
  return q_to_rot_mat(quaternion_tailsitter) @ point 