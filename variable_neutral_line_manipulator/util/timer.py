from time import perf_counter
import datetime
class Timer:
    def __init__(self):
        self.real_start_time = datetime.datetime.now()
        self.start_time = 0
        self.end_time = 0
        self.is_running = False
        
        
    def start(self):
        self.real_start_time = datetime.datetime.now()
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
    def duration_sec(self):
        return perf_counter() - self.start_time if self.is_running else self.end_time - self.start_time
    
    @property
    def real_end_time(self):
        return self.real_start_time + datetime.timedelta(seconds=self.duration_sec)
