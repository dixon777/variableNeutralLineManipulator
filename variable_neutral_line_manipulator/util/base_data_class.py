from abc import ABC
import json

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
                
    def get_data_pairs(self, include_meta=False):
        if include_meta:
            return dict(self).items()
        
        for k,v in dict(self).items():
            if k == "__class_name__":
                continue
            yield k,v
                

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

