class Singleton(type):
    __instances = {}
        
    def __call__(cls,*args,**kwarg):
        if cls not in cls.__instances:
            cls.__instances[cls] = super(Singleton, cls).__call__(*args, **kwarg)
        return cls.__instances[cls]
        