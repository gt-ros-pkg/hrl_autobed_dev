import numpy as np
from scipy.optimize import curve_fit
from pylab import *


def func(x, p1, p2, p3):
    return p1*(x+p2)**p3

#Sensor 2 readings 
ydata2 = np.array([4.2, 5.2, 6.2, 7.2, 8.2, 9.2, 10.2, 11.2, 12.2, 13.2, 14.2, 15.2, 16.2, 17.2, 18.2, 19.2, 20.2, 21.2, 22.2, 23.2, 24.2, 25.2, 26.2, 27.2, 28.2, 29.2])
xdata2 = np.array([2.80908441544, 2.30323195457, 1.98696041107, 1.75092673302, 1.59977054596, 1.48251223564, 1.37235939503, 1.28201079369, 1.18297898769, 1.11068308353, 1.03592383862, 0.979664564133, 0.916261255741, 0.870419919491, 0.821878433228, 0.78410255909, 0.745644509792, 0.708567857742, 0.684701681137, 0.647076666355, 0.628009259701,0.606827139854, 0.580972194672, 0.561412096024,0.562303245068,0.556237816811])
   
popt, pcov = curve_fit(func, xdata2, ydata2,p0=(10.34297387,  -0.20397751,  -0.94007202))

residuals = ydata2 - func(xdata2, popt[0],popt[1],popt[2])
print 'Sensor 2 optimal values:{}'.format(popt)
print 'Sensor 2 error: ={}'.format(sum(residuals**2))

xcal2 = xdata2
ycal2 = func(xdata2, popt[0], popt[1], popt[2])

plot(xdata2, ydata2, color="blue", linewidth=2.5, linestyle="-", label="observed")
plot(xcal2, ycal2, color="red", linewidth=2.5, linestyle="-", label="calibrated")
legend(loc='upper left')
plt.title('SHARP IR Sensor 2 calibration result')


show()

#Sensor 3 readings


ydata3 = np.array([3.2, 4.2, 5.2, 6.2, 7.2, 8.2, 9.2, 10.2, 11.2, 12.2, 13.2, 14.2, 15.2, 16.2, 17.2, 18.2, 19.2, 20.2, 21.2, 22.2, 23.2, 24.2, 25.2, 26.2, 27.2, 28.2, 29.2])

xdata3 = np.array([2.80128526688, 2.41414546967, 2.07105374336, 1.84222650528, 1.67513823509, 1.55958688259, 1.45705270767, 1.36662840843, 1.28706932068, 1.20564258099, 1.14064598083, 1.07249760628, 1.02545118332, 0.970901846886, 0.915000498295, 0.875486791134, 0.821129858494, 0.785059571266, 0.740025401115, 0.715548336506, 0.684522926807, 0.657568871975, 0.628832995892, 0.608412623405, 0.587841808796, 0.585767090321, 0.569034159184])

   
popt, pcov = curve_fit(func, xdata3, ydata3,p0=(10.34297387,  -0.20397751,  -0.94007202))

residuals = ydata3 - func(xdata3, popt[0],popt[1],popt[2])

print 'Sensor 3 optimal values: {}'.format(popt)
print 'Sensor 3 error: = {}'.format(sum(residuals**2))

xcal3 = xdata3
ycal3 = func(xcal3, popt[0], popt[1], popt[2])

plot(xdata3, ydata3, color="blue", linewidth=2.5, linestyle="-", label="observed")
plot(xcal3, ycal3, color="red", linewidth=2.5, linestyle="-", label="calibrated")
legend(loc='upper left')
plt.title('SHARP IR Sensor 3 calibration result')

show()
#Sensor 4 readings


ydata4 = np.array([3.2, 4.2, 5.2, 6.2, 7.2, 8.2, 9.2, 10.2, 11.2, 12.2, 13.2, 14.2, 15.2, 16.2, 17.2, 18.2, 19.2, 20.2, 21.2, 22.2, 23.2, 24.2, 25.2, 26.2, 27.2, 28.2, 29.2])

xdata4 = np.array([2.42777347565, 2.30287098885, 1.93803715706, 1.71649897099, 1.55765628815, 1.44210934639, 1.34966313839, 1.26225590706, 1.18595707417, 1.12297856808, 1.06394040585, 0.994614243507, 0.958061516285, 0.91522949934, 0.869423806667, 0.820302724838, 0.783217787743, 0.761391580105, 0.72705078125, 0.700239241123, 0.671318352222, 0.651708960533, 0.628867208958, 0.607758760452, 0.586113274097, 0.563579082489, 0.592934548855])

   
popt, pcov = curve_fit(func, xdata4, ydata4,p0=(10.34297387,  -0.20397751,  -0.94007202))

residuals = ydata4 - func(xdata4, popt[0],popt[1],popt[2])

print 'Sensor 4 optimal values: {}'.format(popt)
print 'Sensor 4 error: = {}'.format(sum(residuals**2))

xcal4 = xdata4
ycal4 = func(xcal4, popt[0], popt[1], popt[2])

plot(xdata4, ydata4, color="blue", linewidth=2.5, linestyle="-", label="observed")
plot(xcal4, ycal4, color="red", linewidth=2.5, linestyle="-", label="calibrated")
legend(loc='upper left')
plt.title('SHARP IR Sensor 4 calibration result')
show()


