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