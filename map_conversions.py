import math
import numpy as np

def sub2ind(array_shape, rows, cols): #PASSED!!!
    # sub2ind coverts subscript (row, column) pairs into linear indices in
    # row-major order
    #
    # inputs:
    #   array_shape array with [# of rows, # of columns]
    #   rows        numpy array of row indices
    #   cols        numpy array of column indices
    # outputs:
    #   numpy array of integer indices
    #       Note: (row, column) pairs that are not valid should have a 
    #       corresponding output index of -1
    #
    
    ##### YOUR CODE STARTS HERE #####
    ind = rows * array_shape[1] + cols
    inv_ind = (rows < 0) | (rows >= array_shape[0]) | (cols < 0) | (cols >= array_shape[1])
    for i in range(len(ind)):
        if inv_ind[i]:
            ind[i] = -1
    return ind #outputs = numpy array of integer indices
    ##### YOUR CODE ENDS HERE   #####

def ind2sub(array_shape, ind):  #PASSED!!!
    # ind2sub converts linear indices in a row-major array to subscript
    # (row, column) pairs
    #
    # inputs:
    #   array_shape array with [# of rows, # of columns]
    #   ind         numpy array of integer indices
    # outputs:
    #   numpy array of row indices
    #   numpy array of column indices
    #       Note: any indices that are not valid should have row and column
    #       subscripts outputs of -1
    #
    
    ##### YOUR CODE STARTS HERE #####
    rows = ind // array_shape[1]
    cols = ind % array_shape[1]
    inv_ind = (ind < 0) | (rows >= array_shape[0]) | (cols < 0) | (cols >= array_shape[1])
    for i in range(len(ind)):
        if inv_ind[i]:
            rows[i] = -1
            cols[i] = -1
    return rows, cols
    ##### YOUR CODE ENDS HERE   #####
    
def xy2sub(boundary, res, x, y):
    # xy2sub converts (x,y) coordinate pairs into (row, column) subscript pairs
    #
    # inputs:
    #   boundary    array with [xmin, ymin, xmax, ymax]
    #   res         cell size
    #   x           numpy array of x values
    #   y           numpy array of y values
    # outputs:
    #   numpy array of row indices
    #   numpy array of column indices
    #       Note: any (x,y) pairs that are not valid should have subscript
    #       outputs of -1
    #
    
    ##### YOUR CODE STARTS HERE #####
    rows = np.floor((y-boundary[1]) / res).astype(int)
    cols = np.floor((x-boundary[0]) / res).astype(int)
    
    #for points on top right of boundary
    rows[y == boundary[3]] = int((boundary[3] - boundary[1]) / res) - 1
    cols[x == boundary[2]] = int((boundary[2] - boundary[0]) / res) - 1


    #for np.nan points
    rows[np.isnan(x) | np.isnan(y)] = -1
    cols[np.isnan(x) | np.isnan(y)] = -1

    #for all other points
    inv_ind = (x < boundary[0]) | (x > boundary[2]) | (y < boundary[1]) | (y > boundary[3])
    for i in range(len(x)):
        if inv_ind[i]:
            rows[i] = -1
            cols[i] = -1
  
    return rows, cols
    ##### YOUR CODE ENDS HERE   #####
    
def sub2xy(boundary, res, rows, cols): #PASSED!!!
    # sub2xy converts (row, column) subscript pairs into (x,y) coordinate pairs
    #
    # inputs:
    #   boundary    array with [xmin, ymin, xmax, ymax]
    #   res         cell size
    #   rows        numpy array of row indices
    #   cols        numpy array of column indices
    # outputs:
    #   numpy array of x coordinates of center of each cell
    #   numpy array of y coordinates of center of each cell
    #       Note: any (row, col) pairs that are not valid should have outputs
    #       of numpy NaN
    #
    
    ##### YOUR CODE STARTS HERE #####
    x = boundary[0] + res * (cols + 0.5)
    y = boundary[1] + res * (rows +0.5)
    inv_ind = (x < boundary[0]) | (x > boundary[2]) | (y < boundary[1]) | (y > boundary[3])
    for i in range(len(x)):
        if inv_ind[i]:
            x[i] = np.NaN
            y[i] = np.NaN
    return x, y
    
    ##### YOUR CODE ENDS HERE   #####

def xy2ind(boundary, res, array_shape, x, y):
    # xy2ind converts (x,y) coordinate pairs into linear indices in row-major
    # order
    #
    # inputs:
    #   boundary    array with [xmin, ymin, xmax, ymax]
    #   res         cell size
    #   array_shape numpy array with [# of rows, # of columns]
    #   x           numpy array of x values
    #   y           numpy array of y values
    # outputs:
    #   numpy array of row indices
    #   numpy array of column indices
    #
    
    rows, cols = xy2sub(boundary, res, x, y)
    ind = sub2ind(array_shape, rows, cols)
    return ind

def ind2xy(boundary, res, array_shape, ind):
    # ind2xy converts linear indices in row-major order into (x,y) coordinate
    # pairs
    #
    # inputs:
    #   boundary    array with [xmin, ymin, xmax, ymax]
    #   res         cell size
    #   array_shape numpy array with [# of rows, # of columns]
    #   ind         numpy array of indices
    # outputs:
    #   numpy array of x coordinates
    #   numpy array of y coordinates
    #
    
    rows, cols = ind2sub(array_shape, ind)
    x, y = sub2xy(boundary, res, rows, cols)
    return (x, y)

