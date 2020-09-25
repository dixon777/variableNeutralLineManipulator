import os, datetime


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