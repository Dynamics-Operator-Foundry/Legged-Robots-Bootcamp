import numpy as np
cimport numpy as np

cdef extern from 'wrapped_code_0.h':
    void autofunc(double roll, double pitch, double yaw, double roll_lh, double pitch_lh, double yaw_lh, double pitch_lk, double l1, double l2, double F, double *out_8000384635006239031)

def autofunc_c(double roll, double pitch, double yaw, double roll_lh, double pitch_lh, double yaw_lh, double pitch_lk, double l1, double l2, double F):

    cdef np.ndarray[np.double_t, ndim=2] out_8000384635006239031 = np.empty((3,1))
    autofunc(roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, l1, l2, F, <double*> out_8000384635006239031.data)
    return out_8000384635006239031