import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

PI = 3.1416
tao = 0.001

r = 0.07
m = 1
l = 0.2
I = 1

x = np.array([[0.0,0.0,0.0,0.0,0.0]]).T

A = np.array([[0,1], [-9.8,0]])
B = np.array([[0,1]]).T

t0 = np.array([[0]])
out = np.concatenate((t0,x.T), axis=1)

for i in np.arange(tao,10,tao):
    u = np.array([[1,0]]).T
    xdot = x[3]*np.cos(x[2])
    ydot = x[3]*np.sin(x[2])
    tdot = x[4]
    vdot = (u[0] + u[1])/(r*m)
    odot = l*(u[0]-u[1])/(r*I)

    x[0] = x[0] + tao*(xdot)
    x[1] = x[1] + tao*(ydot)
    x[2] = x[2] + tao*(tdot)
    x[3] = x[3] + tao*(vdot)
    x[4] = x[4] + tao*(odot)

    #x = x + tao*(A@x + B*u)

    t0 = np.array([[i]])
    out_dummy = np.concatenate((t0,x.T),axis=1)
    out = np.concatenate((out, out_dummy), axis=0)

df = pd.DataFrame(out, columns = ["time", "x0", "x1", "x2", "x3", "x4"])
print(df)

df.plot(x="x0", y=["x1"])

plt.show()

