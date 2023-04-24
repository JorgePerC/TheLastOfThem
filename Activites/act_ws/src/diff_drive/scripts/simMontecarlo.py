
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

PI = 3.1416
tao = 0.001
mu, sigma = 0, 0.01

reps =  int(input("Cuantos experimentos necesitas? "))
tries =  int(input("De cuantas repeticiones? "))

# Simulations
randomVals = np.random.normal(mu, sigma, ((tries - 1, reps)))
# Array para el tiempo
time = np.zeros((tries, 1))
# Array para hacer la integral
x0 =  np.zeros((tries, reps))        

# Integrando-ando
for i in np.arange(1, tries):
    time[i] = time[i-1] + 1
    x0[i] = x0[i-1] + (randomVals[i-1])

# Resultado, pegamos el tiempo y la integral
out = np.concatenate((time, x0), axis=1)

df = pd.DataFrame(out)
print(df)

df.plot(x= 0, y= [i for i in range(1, reps+1)])#, kind="scatter")

plt.show()