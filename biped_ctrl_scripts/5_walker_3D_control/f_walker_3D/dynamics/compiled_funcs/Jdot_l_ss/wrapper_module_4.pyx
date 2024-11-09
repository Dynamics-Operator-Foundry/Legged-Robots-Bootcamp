import numpy as np
cimport numpy as np

cdef extern from 'wrapped_code_4.h':
    void autofunc(double roll, double pitch, double yaw, double roll_lh, double pitch_lh, double yaw_lh, double pitch_lk, double droll, double dpitch, double dyaw, double droll_lh, double dpitch_lh, double dyaw_lh, double dpitch_lk, double w, double l1, double l2, double *out_4847258859791311282)

def autofunc_c(double roll, double pitch, double yaw, double roll_lh, double pitch_lh, double yaw_lh, double pitch_lk, double droll, double dpitch, double dyaw, double droll_lh, double dpitch_lh, double dyaw_lh, double dpitch_lk, double w, double l1, double l2):

    cdef np.ndarray[np.double_t, ndim=2] out_4847258859791311282 = np.empty((3,14))
    autofunc(roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, w, l1, l2, <double*> out_4847258859791311282.data)
    return out_4847258859791311282