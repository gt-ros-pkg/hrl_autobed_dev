#!/usr/bin/env python

import sys
import serial
import numpy as np

import roslib; roslib.load_manifest('autobed_engine')
import rospy, rosparam
import serial_driver
import sharp_prox_driver

from std_msgs.msg import Float32 
from geometry_msgs.msg import Transform, Vector3, Quaternion

from numpy import sin, linspace, pi
from pylab import plot, show, title, xlabel, ylabel, subplot
from scipy import fft, arange


def spectrum_analyzer_callback(data):
    '''Accepts incoming data from the SHARP sensor publisher, and throws it back to you.
    Be sure to catch it'''
    Fs = 4    
    if spectrum_analyzer_callback.i < spectrum_analyzer_callback.time_slot_for_fft:
        spectrum_analyzer_callback.data_array[spectrum_analyzer_callback.i] = data.data
        spectrum_analyzer_callback.i += 1
        iteration_count = spectrum_analyzer_callback.i
        rospy.loginfo('[sharp_listener] Iteration number: %d', iteration_count)

    elif spectrum_analyzer_callback.i == spectrum_analyzer_callback.time_slot_for_fft:
        plotSpectrum(spectrum_analyzer_callback.data_array, Fs)
        spectrum_analyzer_callback.i += 1

    else:
        rospy.loginfo('[sharp_listener] Should be finished')



def plotSpectrum(y,Fs):
    """
    Plots a Single-Sided Amplitude Spectrum of y(t)
    """
    n = len(y) # length of the signal
    k = arange(n)
    T = n/Fs
    frq = k/T # two sides frequency range
    frq = frq[range(n/2)] # one side frequency range
    
    Y = fft(y)/n # fft computing and normalization
    Y = Y[range(n/2)]
    plot(frq,abs(Y),'r') # plotting the spectrum
    xlabel('Freq (Hz)')
    ylabel('|IR_Spectrum(freq)|')
    rospy.loginfo('[sharp_listener] DONE PLOTING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    show()

spectrum_analyzer_callback.i = 0
spectrum_analyzer_callback.data_array = np.zeros(40)
spectrum_analyzer_callback.time_slot_for_fft = len(spectrum_analyzer_callback.data_array) # Unit : samples. Sampling rate is 4.

def prox_driver_listener_for_fft():
    '''Basic listener node that listens into the sharp IR driver code publisher,
    and publishes an FFT of the whole thing after 10  seconds of recording at 4 Hz'''


    rospy.init_node('sharp_listener', anonymous = True)

    rospy.Subscriber("/pst0", Float32, spectrum_analyzer_callback)

    rospy.spin()

if __name__ == "__main__":
    prox_driver_listener_for_fft()

