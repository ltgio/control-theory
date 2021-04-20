# X caro James :-)

import pandas as pd
import numpy as np
import casadi as ca
import matplotlib.pyplot as plt

np.random.seed(42)

# Number of samples and frequency [Hz]
(N, fs) = 10000, 500

# Declare model variables
(nx, nu, ntheta) = (6, 3, 2)    # system dimension

# create states
x = ca.SX.sym('x', nx)
u = ca.SX.sym('u', nu)
theta = ca.SX.sym('u', ntheta)

# change name
(x1, y1, z1) = x[0], x[1], x[2]
(x2, y2, z2) = x[3], x[4], x[5]
(ux, uy, uz) = u[0], u[1], u[2]
(q, p) = theta[0], theta[1]

# create non-linearity
f_x1 = (2 * x1**3 - x1) / 7
f_x2 = (2 * x2**3 - x2) / 7

# Model equations
rhs = ca.vertcat(
    p * (y1 - f_x1),
    x1 - y1 + z1,
    -q * y1,
    p * (y2 - f_x2) + ux,
    x2 - y2 + z2 + uy,
    -q * y2 + uz
    )

# Form an ode function
ode = ca.Function('ode', [x, u, theta], [rhs])

############ Creating a simulator ##########
N_steps_per_sample = 10
dt = 1/fs/N_steps_per_sample

# Build an integrator for this system: Runge Kutta 4 integrator
k1 = ode(x, u, theta)
k2 = ode(x + dt/2.0*k1, u, theta)
k3 = ode(x + dt/2.0*k2, u, theta)
k4 = ode(x + dt*k3, u, theta)

states_final = x + dt/6.0*(k1+2*k2+2*k3+k4)

# Create a function that simulates one step propagation in a sample
one_step = ca.Function('one_step', [x, u, theta],[states_final])
X = x

for i in range(N_steps_per_sample):
  X = one_step(X, u, theta)

# Create a function that simulates all step propagation on a sample
one_sample = ca.Function('one_sample', [x, u, theta], [X])

############ Simulating the system ##########
all_samples = one_sample.mapaccum("all_samples", N)

# Define input, parameters and initial condition
u_data = ca.DM(np.zeros(shape=(nu, N)))
x0 = ca.DM([0.02, 0.05, -0.04, 0.6, -0.5, -0.0004])
theta_truth = ca.DM([10, 100 / 7])


X_measured = all_samples(x0, u_data, ca.repmat(theta_truth, 1, N))

Xsim = pd.DataFrame(data=X_measured.toarray().T,
                    columns=['x1', 'y1', 'z1', 'x2', 'y2', 'z2'],
                    index=np.linspace(start=0, stop=N, num=N, dtype=int, endpoint=True))


# You may add some noise here
# Xsim+= 0.001*numpy.random.random(N)
# When noise is absent, the fit will be perfect.

Xsim.plot()
plt.show()

fig = plt.figure()
ax = plt.axes(projection="3d")
ax.plot3D(Xsim['x1'], Xsim['y1'], Xsim['z1'], 'gray')
plt.show()