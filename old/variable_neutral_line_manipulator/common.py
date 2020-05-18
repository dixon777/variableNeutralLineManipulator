import logging, os
from datetime import datetime

# def preCond(f):
#     def __inner(func):
#         def __run(*args, **kwargs):
#             f()
#             return func(*args, **kwargs)
#         return __run
#     return __inner

# File operation
def enforceFileSuffix(baseName, suffix=None):
    if not baseName:
        raise ValueError("Base Name must not be None")
    if suffix and not baseName.endswith(suffix):
        baseName = (baseName.rsplit(".",1)[0]) + suffix
    return baseName
        
        
def ensurePath(path=None, suffix=None, defaultBaseName=None):
    """
        dirName = dirname(path) if path else realpath(getcwd())
        baseName = (basename(path) if path else/or (defaultBaseName if defaultBaseName else now())) + suffix
        newPath = dirName + baseName
    """
    path = path or ""
    baseName = (os.path.basename(path) if path else None) or (defaultBaseName or datetime.now().strftime(('%d-%m-%Y_%H-%M-%S')))
    baseName = enforceFileSuffix(baseName, suffix)
    dirName = os.path.dirname(path) or os.path.realpath(os.getcwd())
    path = os.path.join(dirName, baseName)
    os.makedirs(dirName, exist_ok=True)
    
    if(os.path.exists(path)):
        os.remove(path)
    return path


class Singleton(type):
    __instances = {}
        
    def __call__(cls,*args,**kwarg):
        if cls not in cls.__instances:
            cls.__instances[cls] = super(Singleton, cls).__call__(*args, **kwarg)
        return cls.__instances[cls]

"""
    Singleton instance logger
     - Functions
      - Enable automatic spacing depending on the hierarchy of function calls 
       - Requires decorating functions with Logger.hierarchy() (i.e. @Logger.hierarchy)
     - Purpose:
      - Prevent unanticipated logs created by other modules, such as matplotlib
     - Usage:
      - Call the following two lines in the main file:
       - Logger.setLevel(logging.DEBUG) # Set level
         logging.log(logging.NOTSET, "") # To activate logging (Required for some reason)
"""
class Logger(metaclass=Singleton):
    _spaceLevel = 0
    _instance = logging.getLogger("default")
    logging.log(logging.NOTSET, "") # To activate logging (Required for some reason)
    
    @staticmethod
    def hierarchy(func):
        """
            Decorator
        """
        def __c(*args, **kwargs):
            Logger.addSpace()
            res = func(*args, **kwargs)
            Logger.reduceSpace()
            return res
        return __c
                
    @classmethod
    def setLevel(cls, level):
        cls._instance.setLevel(level)
        return cls
    
    @classmethod
    def D(cls, s, extraSpace=0):
        cls._instance.debug(cls.spaceStr(s, extraSpace))
        return cls
        
    @classmethod
    def I(cls, s, extraSpace=0):
        cls._instance.info(cls.spaceStr(s, extraSpace))
        return cls
    
    @classmethod
    def W(cls, s, extraSpace=0):
        cls._instance.warning(cls.spaceStr(s, extraSpace))
        return cls
        
    @classmethod
    def spaceStr(cls, s, extraSpace=0):
        return f"{' '*(cls._spaceLevel+extraSpace)}{s}"
    
    @classmethod    
    def addSpace(cls):
        cls._spaceLevel += 1
        return cls
    
    @classmethod    
    def reduceSpace(cls):
        cls._spaceLevel -= 1
        return cls
        
    @classmethod
    def switchLogger(cls, name):
        cls._instance = logging.getLogger(name)
        return cls
    
    @classmethod
    def get(cls, name=None):
        return logging.getLogger(name) if name else cls._instance
    
