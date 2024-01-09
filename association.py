# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params 

class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''
    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []
        
    def associate(self, track_list, meas_list, KF):
             
        ############
        # TODO Step 3: association:
        # - replace association_matrix with the actual association matrix based on Mahalanobis distance (see below) for all tracks and all measurements
        # - update list of unassigned measurements and unassigned tracks
        ############
        
        # the following only works for at most one track and one measurement
        num_tracks = len(track_list)
        num_meas = len(meas_list)

        # Initialize association matrix with infinity
        self.association_matrix = np.full((num_tracks, num_meas), np.inf)
        self.unassigned_tracks = list(range(num_tracks))
        self.unassigned_meas = list(range(num_meas))

        # Calculate Mahalanobis distance for each track-measurement pair
        for t_idx, track in enumerate(track_list):
            for m_idx, meas in enumerate(meas_list):
                MHD = self.MHD(track, meas, KF)
                if self.gating(MHD, meas.sensor):
                    self.association_matrix[t_idx, m_idx] = MHD

        
        ############
        # END student code
        ############ 
                
    def get_closest_track_and_meas(self):
        ############
        # TODO Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned_tracks and unassigned_meas
        # - return this track and measurement
        ############

        if self.association_matrix.size == 0:
            return np.nan, np.nan

        # Find the minimum entry in association matrix
        min_val = np.min(self.association_matrix)
        if np.isinf(min_val):
            return np.nan, np.nan

        # Get the indices of the minimum value
        ind_track, ind_meas = np.unravel_index(self.association_matrix.argmin(), self.association_matrix.shape)

        # Delete the corresponding row and column
        self.association_matrix = np.delete(self.association_matrix, ind_track, 0)
        self.association_matrix = np.delete(self.association_matrix, ind_meas, 1)

        # Remove from unassigned lists
        update_track = self.unassigned_tracks.pop(ind_track)
        update_meas = self.unassigned_meas.pop(ind_meas)
            
        ############
        # END student code
        ############ 
        return update_track, update_meas     

    def gating(self, MHD, sensor): 
        ############
        # TODO Step 3: return True if measurement lies inside gate, otherwise False
        ############
        
        if MHD < chi2.ppf(params.gating_threshold, df=sensor.dim_meas):
            return True
        else:
            return False
        
        ############
        # END student code
        ############ 
        
    def MHD(self, track, meas, KF):
        ############
        # TODO Step 3: calculate and return Mahalanobis distance
        ############
        
        H = meas.sensor.get_H(track.x)
        gamma = KF.gamma(track, meas)
        S = KF.S(track, meas, H)
        return gamma.T * np.linalg.inv(S) * gamma
        
        ############
        # END student code
        ############ 
    
    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)
    
        # update associated tracks with measurements
        while self.association_matrix.shape[0]>0 and self.association_matrix.shape[1]>0:
            
            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]
            
            # check visibility, only update tracks in fov    
            if not meas_list[0].sensor.in_fov(track.x):
                continue
            
            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])
            
            # update score and track state 
            manager.handle_updated_track(track)
            
            # save updated track
            manager.track_list[ind_track] = track
            
        # run track management 
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)
        
        for track in manager.track_list:            
            print('track', track.id, 'score =', track.score)