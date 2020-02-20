import math
import logging

import numpy as np
import pyrr.matrix44 as m4
import pyrr.matrix33 as m3

# Not required if model is assumed frictionless between ring and tendon
def evalCapstan(tensionEnd: float, fricCoef: float, totalAngle: float) -> float:
    """
        Evaluate Capstan equ.
        @param tensionEnd = Resultant tension
    """
    return tensionEnd*math.exp(fricCoef*totalAngle)
    

"""
    To eliminate numeraical errors for equility
"""
def allWithin(a,b,threshold=0.0001) -> bool:
    return np.all(np.abs(np.array(a) - np.array(b)) < threshold)


"""
    Transformation matrix wrapper
    - pyrr is column major, while Numpy is row major
     - In Numpy, if m is a matrix, m[i,j] outputs the entry in i row and j column, whereas
       in pyrr, m[i,j] outputs the entry in j row and i column
    - Therefore, transpose is applied
    - Warning:
    -   np.array(vec)*1.0 is to the vector is converted to either float32 or float64 
        to be compiled with pyrr's requirement (if the vector's type is any int, inaccurate result will be returned)
        Also, [array(vec)] enables user to define the input in the form of either list or tuple
        Moreover, [*1.0] instead of [np.array(vec,dtype=np.float32)] is to maintain its type if the vector is already in float type
"""
def m4MatrixTranslation(vec):
    return m4.create_from_translation(np.array(vec)*1.0).transpose() 

def m3MatrixRotation(axis, radian):
    return m3.create_from_axis_rotation(np.array(axis)*1.0, radian).transpose()

def m4MatrixRotation(axis, radian):
    return m4.create_from_axis_rotation(np.array(axis)*1.0, radian).transpose()


# """
#     Singleton instance logger
#      - Functions
#       - Enable automatic spacing depending on the hierarchy of function calls 
#        - Requires decorating functions with Logger.hierarchy() (i.e. @Logger.hierarchy)
#      - Purpose:
#       - Prevent unanticipated logs created by other modules, such as matplotlib
#      - Usage:
#       - Call the following two lines in the main file:
#        - Logger.setLevel(logging.DEBUG) # Set level
#          logging.log(logging.NOTSET, "") # To activate logging (Required for some reason)
# """
# class Logger():
#     _spaceLevel = 0
#     _instance = None
    
#     @staticmethod
#     def hierarchy(func):
#         """
#             Decorator
#         """
#         def __c(*args, **kwargs):
#             Logger.addSpace()
#             res = func(*args, **kwargs)
#             Logger.reduceSpace()
#             return res
#         return __c
    
#     @classmethod
#     def initLogger(cls):
#         if cls._instance is None:
#                 cls._instance = logging.getLogger()
    
#     @classmethod
#     def setLevel(cls, level):
#         cls._instance.setLevel(level)
    
#     @classmethod
#     def D(cls, s, extraSpace=0):
#         cls._instance.log(logging.DEBUG, cls.spaceStr(s, extraSpace))
        
#     @classmethod
#     def I(cls, s, extraSpace=0):
#         cls._instance.log(logging.INFO, cls.spaceStr(s, extraSpace))
        
#     @classmethod
#     def spaceStr(cls, s, extraSpace=0):
#         return f"{' '*(cls.spaceLevel+extraSpace)}{s}"
    
#     @classmethod    
#     def addSpace(cls):
#         cls.spaceLevel += 1
    
#     @classmethod    
#     def reduceSpace(cls):
#         cls.spaceLevel -= 1
        
#     @classmethod
#     def switchLogger(cls, name):
#         cls._instance = logging.getLogger(name)
    
#     @classmethod
#     def get(cls, name=None):
#         return logging.getLogger(name) if name else cls._instance