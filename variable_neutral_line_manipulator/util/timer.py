from time import perf_counter
class Timer:
    def __init__(self):
        self.start_time = 0
        self.end_time = 0
        self.is_running = False
        
    def start(self):
        self.start_time = perf_counter()
        self.is_running = True
        return self
        
    def stop(self):
        self.end_time = perf_counter()
        self.is_running = False
        
    def __enter__(self):
        return self.start()
    
    def __exit__(self, *args):
        return self.stop()
        
    @property
    def duration(self):
        return perf_counter() - self.start_time if self.is_running else self.end_time - self.start_time
