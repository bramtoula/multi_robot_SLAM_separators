import numpy as np
import matplotlib.pyplot as plt
from compare_est_gt import compare_est_gt
import settings

t_errors = []
Rot_errors = []
i = 0
rejected = 0
t_lim = 10.0
Rot_lim = 45

id1 = 2417
id2 = 3361

Rot_err, t_err = compare_est_gt(id1,id2,print_res=True)
