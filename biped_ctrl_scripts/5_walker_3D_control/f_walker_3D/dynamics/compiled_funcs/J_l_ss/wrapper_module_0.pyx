import numpy as np
cimport numpy as np

cdef extern from 'wrapped_code_0.h':
    void autofunc(double roll, double pitch, double yaw, double roll_lh, double pitch_lh, double yaw_lh, double pitch_lk, double w, double l1, double l2, double *out_2048035323446825405)

def autofunc_c(double roll, double pitch, double yaw, double roll_lh, double pitch_lh, double yaw_lh, double pitch_lk, double w, double l1, double l2):

    cdef np.ndarray[np.double_t, ndim=2] out_2048035323446825405 = np.empty((3,14))
    autofunc(roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, w, l1, l2, <double*> out_2048035323446825405.data)
    return out_2048035323446825405