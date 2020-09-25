from time import perf_counter
class Timer:
    def __init__(self):
        self.start = 0
        self.end = 0
        
    def start(self):
        self.start = perf_counter()
        self.end = self.start
        return self
        
    def end(self):
        self.end = perf_counter()
        
    def __enter__(self):
        return self.start()
    
    def __exit__(self, *args):
        return self.end()
        
    @property
    def duration(self):
        return self.end - self.start
