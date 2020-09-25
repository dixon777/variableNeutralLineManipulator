import logging, os, time
from datetime import datetime
from abc import ABC
import json

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


class ErrorDict():
    def __init__(self, init_d={}):
        self.dict = {}
        if init_d:
            self.dict.update(init_d)
        
    def add(self, key, err):
        if key not in self.dict:
            self.dict[key] = []
        self.dict[key].append(err)
        
    def update(self, d):
        self.dict.update(d)
    
    def clear(self):
        self.dict.clear()
        
    def dict_copy(self):
        return self.dict.copy()
    
    @property
    def item_iterable(self):
        return self.dict.items()
    
    def has_errors(self):
        return len(self.dict) > 0
        
    def __str__(self):
        return "\n".join([f"{k}: {e}" for k,e in self.dict.items()])


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
        def __inner(*args, **kwargs):
            Logger.addSpace()
            res = func(*args, **kwargs)
            Logger.reduceSpace()
            return res
        return __inner
                
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
    def E(cls, s, extraSpace=0):
        cls._instance.error(cls.spaceStr(s, extraSpace))
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
    

class Timer:
    def __init__(self):
        super().__init__()
        self.start = 0
        self.end = 0
        
    def __enter__(self):
        self.start = time.perf_counter()
        self.end = self.start
    
    def __exit__(self, *args):
        self.end = time.perf_counter()
        
    @property
    def duration(self):
        return self.end - self.start

class GrowingList(list):
    def __setitem__(self, index, value):
        if index >= len(self):
            self.extend([None]*(index + 1 - len(self)))
        list.__setitem__(self, index, value)
        
    def shrink(self):
        temp = [x for x in self]
        self.clear()
        for item in temp:
            if item is not None:
                self.append(item)

def indices_entity_pairs_to_ordered_list(indices_entity_pairs):
    growing_list = GrowingList()
    for indices, entity in indices_entity_pairs:
        for i in indices:
            growing_list[i] = entity
    growing_list.shrink()
    return growing_list


class BaseDataClass(ABC):
    @classmethod
    def all_subclasses(cls):
        return set(cls.__subclasses__()).union([s for c in cls.__subclasses__() for s in c.all_subclasses()])

    @property
    def attr_keys(self):
        keys = []
        for base in [*self.__class__.__bases__, self.__class__]:
            keys += base.local_attr_keys(self) or []
        return keys

    @property
    def eq_attr_keys(self):
        keys = []
        for base in [*self.__class__.__bases__, self.__class__]:
            keys += base.local_eq_attr_keys(
                self) or base.local_attr_keys(self) or []
        return keys

    def local_attr_keys(self):
        return []

    def local_eq_attr_keys(self):
        return None

    def __eq__(self, other):
        return isinstance(other, self.__class__) and all(getattr(self,key) == getattr(other,key) for key in self.eq_attr_keys)

    def __hash__(self):
        return hash((getattr(self,k) for k in self.eq_attr_keys))

    def __iter__(self):
        yield "__class_name__", self.__class__.__name__
        for k in self.attr_keys:
            if hasattr(self, k):
                j = getattr(self, k)
                yield k, j

    def __repr__(self):
        return str(dict(self))

    class Encoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, BaseDataClass):
                return dict(obj)
            return super().default(obj)

    class Decoder(json.JSONDecoder):
        def __init__(self, *args, **kwargs):
            super().__init__(object_hook=self.object_hook, *args, **kwargs)

        @staticmethod
        def object_hook(obj):
            if '__class_name__' not in obj:
                return obj
            cls_str = obj['__class_name__']
            del obj['__class_name__']
            cl = next((c for c in BaseDataClass.all_subclasses()
                       if c.__name__ == cls_str), None)
            if cl is None:
                print("Error in json ")
                return None
            return cl(**obj)

