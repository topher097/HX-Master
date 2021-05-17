import scipy
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np
import numpy.polynomial.polynomial as poly
import pandas as pd

df = pd.read_csv('thermistor_interpolation_table.csv', sep=',',header=0)
temp = df.loc[:, 'temp'].values                 # x
thermistor = df.loc[:, 'resistance'].values     # y

# Input Variables
v_in = 3.3
R2 = 4000 * np.ones(len(temp))
#print(R2)

v_out = v_in * R2/(R2+thermistor)


# Python:
# 0.3588335561202086 + 0.01706464153538489 x**1 + 0.0002883657030878859 x**2 - 2.1424897793690012e-07 x**3 - 3.8134280753535603e-08 x**4 + 1.8991877578569086e-10 x**5
# Google Sheets:
#-42 + 178x + -216x**2 + 148x**3 + -48.1x**4 + 6.03x**5

google_coef = [0.359, 0.0171, 2.88e-4, -2.17e-7, -3.8e-8, 1.89e-10]
python_coef = poly.polyfit(temp, v_out, 11)
print(python_coef.size)
python_coef_edit = np.zeros(python_coef.size)
for i in np.arange(0, python_coef.size):
    python_coef_edit[i] = float('{:0.5e}'.format(python_coef[i]))

google_interp = poly.Polynomial(google_coef)
python_interp = poly.Polynomial(python_coef)
python_interp_edit = poly.Polynomial(python_coef_edit)
google_error = v_out - google_interp(temp)
python_error = v_out - python_interp(temp)
python_edit_error = v_out - python_interp_edit(temp)

print(f"Google: y = {google_interp}\n")
print(f"Python: y = {python_interp}\n")
print(f"Python Edit: y = {python_interp_edit}")


xnew = np.arange(-40, 126)

plt.figure()
plt.title('$V_{out}$ vs. Temperature')
plt.plot(temp, v_out, 'k-', label='Data')
plt.plot(temp, google_interp(temp), 'r--', label='Google')
plt.plot(temp, python_interp(temp), 'y--', label='Python')
plt.plot(temp, python_interp_edit(temp), 'b--', label='Python Edit')
plt.legend(loc='best')
plt.draw()

plt.figure()
plt.title('Interpolation Error vs Temperature')
plt.plot(temp, google_error, 'r-', label='Google')
plt.plot(temp, python_error, 'y-', label='Python')
plt.plot(temp, python_edit_error, 'b-', label='Python Edit')
plt.plot(temp, np.zeros(temp.size), 'k--')
plt.legend(loc='best')
plt.show()
