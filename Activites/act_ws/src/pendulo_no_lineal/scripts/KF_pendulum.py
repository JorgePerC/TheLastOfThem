#Pendulum simple code no-lineal x'=f(x,u=0)
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

#Parameters
PI = 3.1416
Tao = 0.001
g = 9.8

statesX = np.array([[PI/4, 0.0]]).T
xPredic = np.array([[0.0, 0.0]]).T

t0 = np.array([ [0.0] ])
out = np.concatenate((t0, statesX.T, xPredic.T), axis=1)

# Aka A
systemDynamics = np.array([[0.0, 1.0], [-g, 0.0]])

# Aka B
externalForces = np.array([[0.0, 1.0]]).T

# Aka z
    # In this case our sensors are exactly the dynamics
sensorInputs = statesX.copy() 

# Aka H
    # Identity-ish matrix for sensor input
    # Vector space from sensor readouts to statesX
statesSensor = np.array([   [1.0, 0.0], 
                            [0.0, 1.0]]) 

# Aka Q
    # Covariance from model noise
    # How it grows over time
model_Cov_Mat = np.array([ [1.0, 0.0], 
                            [0.0, 1.0]]) 

# Aka R
    # Covariance from sensor input
sensor_Cov_Mat = np.array([ [1.0, 0.0], 
                            [0.0, 1.0]]) 

# Aka P
    # Covariance from state estimation 
prediction_Cov_Mat = np.array([ [0.0, 0.0], 
                                [0.0, 0.0]])

for i in np.arange(Tao,10,Tao):
    u = 0 # control 
    G = 1

    # Update state vector
    # Normal system dynamics
    statesX = statesX + Tao*(np.matmul(systemDynamics, statesX) + externalForces*u) 
    # Update sensor readout
    sensorInputs = statesX.copy() 

    # Update prediction 
    xPredic = xPredic + Tao*( 
            np.matmul(systemDynamics, xPredic) + externalForces*u +
            # Kalman filter stuff
            np.matmul(
                np.matmul(
                    np.matmul(prediction_Cov_Mat, statesSensor.T), 
                    np.linalg.inv(sensor_Cov_Mat)), 
                (sensorInputs - np.matmul(statesSensor, xPredic)))
                )
    
    # Update prediction covariance matrix
    prediction_Cov_Mat = prediction_Cov_Mat + Tao*(
            # Covariance of our system
            np.matmul(systemDynamics, prediction_Cov_Mat) + 
            # Y tho?
            np.matmul(prediction_Cov_Mat, systemDynamics.T) +
            # Since G is identity, it's like multiplying by one
            model_Cov_Mat - 
            
            np.matmul(
                np.matmul(
                    np.matmul(
                        np.matmul(prediction_Cov_Mat, statesSensor.T), 
                    np.linalg.inv(sensor_Cov_Mat)), 
                statesSensor), 
            prediction_Cov_Mat)
        
            )
    
    # Insert to final graph
    t0 = np.array([ [i] ])
    out_dummy = np.concatenate((t0, statesX.T, xPredic.T), axis=1)
    out = np.concatenate((out, out_dummy), axis=0)


df = pd.DataFrame(out, columns=["time", "x1", "x2", "p1", "p2"])

df.plot(x="time", y=["x1", "p1"])
df.plot(x="time", y=["x2", "p2"])

plt.show()