
        
        
def removeFromLayout(layout, i):
    """
        Remove i-th item
    """
    count = layout.count()
    i = i if i >= 0 else count + i
    if i >= count or i < 0:
        return False
    item = layout.itemAt(i)
    if item is None:
        return False
    widget = item.widget()
    if widget is None:
        layout.removeItem(item)
    else:
        layout.removeWidget(widget)
        widget.setParent(None)
    return True

def removeAllWidgetsFromLayout(layout, types=None):
    l = [layout.itemAt(i).widget() for i in range(layout.count())]
    
    for w in l:
        if not types or isinstance(w, types):
            w.setParent(None)
            layout.removeWidget(w)