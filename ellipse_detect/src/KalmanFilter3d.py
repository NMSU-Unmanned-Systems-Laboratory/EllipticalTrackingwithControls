
from cmath import pi
import numpy as np
import imutils
import cv2
import math
import time
import csv
from filterpy.kalman import KalmanFilter




class KalmanBox3dTracker(object):
    """
    This class represents the internel state of individual tracked objects
    observed as bbox.
    """
    count = 0

    def __init__(self, coord3d):
        """
        Initialises a tracker using initial bounding box.
        """
        # define constant velocity model
        # X,Y,Z, dX, dY, dZ
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.F = np.array(
            [[1, 0, 0, .033, 0, 0],
             [0, 1, 0, 0, .033, 0],
             [0, 0, 1, 0, 0, .033],
             [0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array(
            [[1, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]])


        self.kf.R[2:, 2:] *= 7.5 # 7.0 seems to be a good balance for just noise rejection
        self.kf.P[
        3:,
        3:] *= 1000.  # give high uncertainty to the unobservable initial
        # velocities
        self.kf.P *= 1000.
        self.kf.Q[-1, -1] *= .01
        self.kf.Q[3:, 3:] *= .01
        self.kf.R *= 20.0

        # X, Y, Z, s (area), r
        self.kf.x[:3] = coord3d.reshape(-1, 1)
        self.time_since_update = 0
        self.id = KalmanBox3dTracker.count
        KalmanBox3dTracker.count += 1
        self.history = [coord3d.reshape(-1, 1)]
        self.hits = 0
        self.hit_streak = 0
        self.age = 0
        self.aff_value = 0
        self.occ = False
        self.lost = False

    def update(self, coord3d):
        """
        Updates the state vector with observed bbox.
        """
        self.time_since_update = 0
        self.history = [coord3d.reshape(-1, 1)]
        self.hits += 1
        self.hit_streak += 1
        self.occ = False
        self.lost = False
        # X, Y, Z, s (area), r
        self.kf.update(coord3d.squeeze())

    def predict(self):
        """
        Advances the state vector and returns the predicted bounding box
        estimate.
        """
        self.kf.predict()
        self.age += 1
        if (self.time_since_update > 0):
            self.hit_streak = 0
        self.time_since_update += 1
        self.history.append(self.kf.x[:3])
        return self.history[-1]

    def get_state(self):
        """
        Returns the current bounding box estimate.
        """
        return self.kf.x

    def get_history(self):
        """
        Returns the history of estimates.
        """
        return self.history


