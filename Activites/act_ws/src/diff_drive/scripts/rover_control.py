import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

PI = 3.1416
tao = 0.001

r = 0.04
m = 1
h = 0.15
d = 0.10
I = 1

q_deseada = np.array([[10.0,  10.0,  0.0]]).T

# Aka. q (estado inicial)
x = np.array([[1.0,  1.0,  0.0]]).T

K = np.array([[1, 0, 0], [0, 2, 0], [0, 0, 0]])


t0 = np.array([[0]])
out = np.concatenate((t0,x.T), axis=1)

for i in np.arange(tao,10,tao):
    error = q_deseada - x
    u = np.matmul(K, error)

    A = np.array([[r/2*np.cos( x[2,0] )-h*r/d*np.sin( x[2,0] ), r/2*np.cos( x[2,0] )+h*r/d*np.sin( x[2,0] ), 0],
              [r/2*np.sin( x[2,0] )-h*r/d*np.cos( x[2,0] ), r/2*np.sin( x[2,0] )+h*r/d*np.cos( x[2,0] ), 0],
              [r/d, -r/d, 0]])

    # Estado actual
    x = x + tao*(np.matmul(A, x) + u)
    

    t0 = np.array([[i]])
    out_dummy = np.concatenate((t0,x.T),axis=1)
    out = np.concatenate((out, out_dummy), axis=0)

df = pd.DataFrame(out, columns = ["time", "x0", "x1", "x2"])
print(df)

df.plot(x="x0", y=["x1"])

plt.show()

