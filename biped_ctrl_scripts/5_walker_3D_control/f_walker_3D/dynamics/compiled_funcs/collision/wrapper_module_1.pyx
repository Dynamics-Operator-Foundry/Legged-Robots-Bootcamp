cdef extern from 'wrapped_code_1.h':
    double autofunc(double roll, double pitch, double yaw, double roll_lh, double pitch_lh, double yaw_lh, double pitch_lk, double roll_rh, double pitch_rh, double yaw_rh, double pitch_rk, double w, double l1, double l2)

def autofunc_c(double roll, double pitch, double yaw, double roll_lh, double pitch_lh, double yaw_lh, double pitch_lk, double roll_rh, double pitch_rh, double yaw_rh, double pitch_rk, double w, double l1, double l2):

    return autofunc(roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l1, l2)