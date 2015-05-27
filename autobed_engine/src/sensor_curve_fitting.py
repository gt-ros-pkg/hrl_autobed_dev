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

xdata4 = np.array([2.7822508812, 2.39318370819, 2.02812504768, 1.76050293446, 1.60502445698, 1.49184572697, 1.38230955601, 1.30806148052, 1.22291994095, 1.15875482559, 1.08435058594, 1.02672362328, 0.985815405846, 0.925473630428 , 0.868305683136, 0.83203125, 0.796376943588,0.760312497616,0.739624023438,0.69851565361,
0.680000007153,0.658740222454,0.634843766689,0.620532214642,0.604648411274,
0.585292994976,0.58474612236])

   
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


