#Pendulum simple code no-lineal x'=f(x,u=0)
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

#Parameters
PI = 3.1416
Tao = 0.001
g = 9.8

x = np.array([[PI/4,0]]).T

t0 = np.array([ [0] ])
out = np.concatenate((t0, x.T), axis=1)

A = np.array([[0, 1], [-g, 0]])
B = np.array([[0, 1]]).T
z = x.copy() #States from sensors
H = np.array([[1, 0], [0,1]]) # Vector space from sensors

Q = np.array([[1.0, 0], [0,1.0]]) # Covariance from model noise
R = np.array([[1.0, 0], [0,1.0]]) # Covariance from sensor input


P = np.array([[0, 0], [0,0]])

for i in np.arange(Tao,30,Tao):
    u = 0
    x = x + Tao*(A@x + B*u + P@H.T@np.linalg.inv(R)@(z-H@x))
    P = P + Tao*(A@P + P@A.T + Q - P@H.T@np.linalg.inv(R)@H@P)

    t0 = np.array([ [i] ])
    out_dummy = np.concatenate((t0, x.T), axis=1)
    out = np.concatenate((out, out_dummy), axis=0)

df = pd.DataFrame(out, columns=["time", "x1", "x2"])

df.plot(x="time", y=["x1"])

plt.show()