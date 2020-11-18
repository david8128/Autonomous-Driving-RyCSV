#!/usr/bin/env python

import math
import numpy as np


if __name__ == '__main__':
    n = 9
    rad = 15
    th = np.deg2rad(90/n)

    corner_dots_1 = np.zeros((n,3))

    for u in range(n):
        x = 12 + rad*np.sin(th*u)
        y = -1*rad*np.cos(th*u)
        corner_dots_1[u] = np.array([x,y,np.rad2deg(th*u)])   

    corner_dots_2 = np.zeros((n,3))

    for u in range(n):
        x = 12 + rad*np.cos(th*u)
        y = rad*np.sin(th*u)
        corner_dots_2[u] = np.array([x,y,np.rad2deg(th*u)+90])

    corner_dots_3 = np.zeros((n,3))

    for u in range(n):
        x = -12 - (rad*np.sin(th*u))
        y = rad*np.cos(th*u)
        corner_dots_3[u] = np.array([x,y,np.rad2deg(th*u)+180])   

    corner_dots_4 = np.zeros((n,3))

    for u in range(n):
        x = -12 - (rad*np.cos(th*u))
        y = -1*rad*np.sin(th*u)
        corner_dots_4[u] = np.array([x,y,np.rad2deg(th*u)+270])  

    oval = np.concatenate((corner_dots_1, 
                           corner_dots_2, 
                           corner_dots_3, 
                           corner_dots_4), 
                           axis=0)
    print(oval)