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