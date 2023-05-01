import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

PI = 3.1416
tao = 0.001

mu, sigma = 0, 0.01

r = 0.04
m = 1
h = 0.15
d = 0.10
I = 1

q_deseada = np.array([[5.0,  20.0,  0.0]]).T

# Aka. q (estado inicial)
x = np.array([[1.0,  1.0,  0.0]]).T

xP = np.array([[0.0, 0.0, 0.0]]).T

K = np.array([[1, 0, 0], [0, 2, 0], [0, 0, 0]])

t0 = np.array([[0]])
out = np.concatenate((t0,x.T, xP.T), axis=1)

# Aka z
    # In this case our sensors are exactly the dynamics
sensorInputs = x.copy() 

# Aka H
    # Identity-ish matrix for sensor input
    # Vector space from sensor readouts to statesX
statesSensor = np.array([  [0.0, 0.0, 0.0], 
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]]) 

# Aka Q
    # Covariance from model noise
model_Cov_Mat = np.array([ [1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]]) 

# Aka R
    # Covariance from sensor input
sensor_Cov_Mat = np.array([ [1.0, 0.0, 0.0], 
                            [0.0, 1.0, 0.0],
                            [0.0, 0.0, 1.0]]) 

# Aka P
    # Covariance from state estimation 
prediction_Cov_Mat = np.array([ [0.0, 0.0, 0.0], 
                                [0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0]])


for i in np.arange(tao,20,tao):
    # Create noise in the system
    noise = np.random.normal(mu, sigma, (3,1))
    
    # Calculate noise
    error = q_deseada - x 
    
    # Calculate control (actuator force)
    u = np.matmul(K, error)

    # System dynamics
    A = np.array([[r/2*np.cos( x[2,0] )-h*r/d*np.sin( x[2,0] ), r/2*np.cos( x[2,0] )+h*r/d*np.sin( x[2,0] ), 0],
              [r/2*np.sin( x[2,0] )-h*r/d*np.cos( x[2,0] ), r/2*np.sin( x[2,0] )+h*r/d*np.cos( x[2,0] ), 0],
              [r/d, -r/d, 0]])
    
    
    # Estado actual
    x = x + tao*(np.matmul(A, x) + u ) + noise

    # Update sensor readout
    sensorInputs = x.copy() + noise

    # Update prediction 
    xP = xP + tao*( 
            A@xP + u +
            # Kalman filter stuff
            prediction_Cov_Mat@statesSensor.T@np.linalg.inv(sensor_Cov_Mat)@(sensorInputs-statesSensor@xP))

    # Update prediction covariance matrix
    prediction_Cov_Mat = prediction_Cov_Mat + tao*(
            A@prediction_Cov_Mat + prediction_Cov_Mat@A.T +
            model_Cov_Mat -
            prediction_Cov_Mat@statesSensor.T@np.linalg.inv(sensor_Cov_Mat)@statesSensor@prediction_Cov_Mat)
            

    t0 = np.array([[i]])
    out_dummy = np.concatenate((t0,x.T,xP.T),axis=1)
    out = np.concatenate((out, out_dummy), axis=0)

df = pd.DataFrame(out, columns = ["time", "x0", "x1", "x2", "p0", "p1", "p2"])
#print(df)

df.plot(x="time", y=["x0", "p0"])
df.plot(x="time", y=["x1", "p1"])

plt.show()

