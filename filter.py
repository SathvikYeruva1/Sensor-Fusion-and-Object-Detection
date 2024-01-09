# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        dt = params.dt
        F = np.matrix([[1, 0, 0, dt, 0, 0],
                    [0, 1, 0, 0, dt, 0],
                    [0, 0, 1, 0, 0, dt],
                    [0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 1]])
        return F
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############

        dt = params.dt # Time step
        q = params.q # Process noise magnitude
        # Process noise covariance matrix for 3D constant velocity model
        Q = q * np.matrix([[dt**4/4, 0, 0, dt**3/2, 0, 0],
                        [0, dt**4/4, 0, 0, dt**3/2, 0],
                        [0, 0, dt**4/4, 0, 0, dt**3/2],
                        [dt**3/2, 0, 0, dt**2, 0, 0],
                        [0, dt**3/2, 0, 0, dt**2, 0],
                        [0, 0, dt**3/2, 0, 0, dt**2]])
        return Q
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        F = self.F()
        Q = self.Q()
        # Predict state and covariance
        track.x = F * track.x
        track.P = F * track.P * F.T + Q
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############

        H = meas.sensor.get_H(track.x) # Get measurement matrix
        R = meas.R # Measurement noise covariance
        gamma = self.gamma(track, meas) # Residual
        S = self.S(track, meas, H) # Covariance of residual
        K = track.P * H.T * np.linalg.inv(S) # Kalman gain
        track.x = track.x + K * gamma # Update state
        track.P = (np.eye(track.x.shape[0]) - K * H) * track.P # Update covariance

        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############

        hx = meas.sensor.get_hx(track.x)
        gamma = meas.z - hx
        return gamma
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        R = meas.R 
        S = H * track.P * H.T + R 
        return S
        
        ############
        # END student code
        ############ 