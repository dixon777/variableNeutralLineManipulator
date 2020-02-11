import os

import numpy as np

from OpenGL.GL import *
from OpenGL.GL.shaders import compileProgram, compileShader

class UniformVariable():
    def __init__(self, loc, initVal: np.ndarray):
        self.loc = loc
        self.value = initVal
        self.hasChanged = True
        if self.value.shape == (4,4):
            t = self.value.dtype
            if t == np.float32 or t == np.float64:
                self.input = lambda val: glUniformMatrix4fv(self.loc, 1, GL_FALSE, val)
        
    @property
    def val(self):
        return self.value
    
    def setVal(self, val):
        if not np.array_equal(val, self.value):
            self.value = val
            self.hasChanged = True
            
    def resetChangeFlag(self, val):
        self.hasChanged = val
    
    def inputGPU(self, shouldResetChanged=True):
        if self.hasChanged:
            self.input(self.value)
            if shouldResetChanged:
                self.hasChanged = False
        

class Renderer():
    def __init__(self, meshes, shaderProgram, uniformMap={}):
        self.shaderProgram = shaderProgram
        self.uniformMap = uniformMap
        self.meshes = meshes
        
        self.VBO = None
        self.IBO = None
        self.vertexOffsets = []
        
        self.setupBuf()
        
    def updateUniform(self, key, value, indices=None):
        if indices is None:
            uv = self.uniformMap.get(key)
            if uv is not None:
                uv.setVal(value)
                return
            
        if indices is None:
            for m in self.meshes:
                m.updateUniform(key, value)
        elif isinstance(indices, int):
            self.meshes[indices].updateUniform(key,value)
        elif isinstance(indices,(list, tuple)):
            for i in indices:
                self.meshes[i].updateUniform(key,value)
        else:
            raise ArgumentError()
        
    
    def setupBuf(self):
        meshes = self.meshes
        
        self.VBO = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self.VBO)
        
        self.IBO = glGenBuffers(1)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.IBO)
        
        # Store vertex data buffer
        vertexData = [m.getVertexBufferData() for m in meshes]
        glBufferData(GL_ARRAY_BUFFER, sum([d.nbytes for d in vertexData]), None, GL_STATIC_DRAW)
        vertexOffset = 0
        for vtd in vertexData:
            glBufferSubData(GL_ARRAY_BUFFER, vertexOffset, vtd.nbytes, vtd)
            self.vertexOffsets.append(vertexOffset)
            vertexOffset += vtd.nbytes
        
        # Store index data buffer
        indexData = [m.getIndexBufferData() for m in meshes]
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sum([d.nbytes for d in indexData]), None, GL_STATIC_DRAW)
        indexOffset = 0
        for idd in indexData:
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, indexOffset, idd.nbytes, idd)
            indexOffset += idd.nbytes
        
        # Configure array attri
        for m, offset in zip(meshes, self.vertexOffsets):
            glBindBuffer(GL_ARRAY_BUFFER, self.VBO)
            m.setupArrayAttrib(offset)
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.IBO)
    
    
    def useProgram(self):
        glUseProgram(self.shaderProgram)
        
    def _inputUniform(self):
        for uv in self.uniformMap.values():
            uv.inputGPU(False)
        
    def draw(self):
        glUseProgram(self.shaderProgram)
        for m, offset in zip(self.meshes, self.vertexOffsets):
            m.draw(offset, lambda: self._inputUniform())
        
        for uv in self.uniformMap.values():
            uv.resetChangeFlag(False)
        glUseProgram(0)

class Mesh():
    def __init__(self, vertices, 
                 indices:np.ndarray, 
                 uniformMap={}):
        self.VAO = None
        self.vertices = vertices
        self.indices = indices
        self.uniformMap = uniformMap
        
    def getVertexBufferData(self) -> np.ndarray:
        return np.concatenate(self.vertices, axis=1).flatten()
    
    def getIndexBufferData(self) -> np.ndarray:
        return self.indices.flatten()
    
    def setupArrayAttrib(self, vertexByteOffset):
        self.VAO = self.VAO or  glGenVertexArrays(1)
        glBindVertexArray(self.VAO)
        
        itemsize = self.vertices[0].itemsize
        propertyItemCounts = [v.shape[1] for v in self.vertices]
        vertexItemCount = sum(propertyItemCounts)
        
        propertyByteOffset = vertexByteOffset
        for i, propertyItemCount in enumerate(propertyItemCounts):
            glEnableVertexAttribArray(i)
            glVertexAttribPointer(i, propertyItemCount, GL_FLOAT, GL_FALSE, vertexItemCount*itemsize, ctypes.c_void_p(propertyByteOffset))
            propertyByteOffset += propertyItemCount*itemsize

    def draw(self, indexByteOffset=0, activity=None):
        if self.VAO is None:
            raise Exception()
        glBindVertexArray(self.VAO)
        if activity is not None:
            activity()
        for uv in self.uniformMap.values():
            uv.inputGPU()
        glDrawElements(GL_TRIANGLES, self.indices.size, GL_UNSIGNED_INT, ctypes.c_void_p(indexByteOffset))
        
    def updateUniform(self, key, value):
        uv = self.uniformMap.get(key)
        if uv is not None:
            uv.setVal(value)
            return        
        raise ArgumentError()
        
        
       
def getGLInfo():
    return f"""
        Vendor: {glGetString(GL_VENDOR).decode("utf-8")}
        Renderer: {glGetString(GL_RENDERER).decode("utf-8")},
        OpenGL version: {glGetString(GL_VERSION).decode("utf-8")},
        Shader version: {glGetString(GL_SHADING_LANGUAGE_VERSION).decode("utf-8")}
    """


def loadObj(path):
    v = []
    n = []
    vi = []
    ni = []
    for line in open(path, 'r'):
        if line.startswith('#'):
            continue
        values = line.split()
        if not values:
            continue
        
        prefix = values[0]
        if prefix == 'v':
            v.append(values[1:4])
        elif prefix == 'vn':
            n.append(values[1:4])  
        elif prefix == 'f':
            for v in values[1:4]:
                w = v.split('//')
                vi.append(int(w[0])-1)
                ni.append(int(w[1])-1)
        
    return v,n,vi,ni

def loadShader(path):
    with open(path, "rb") as f:
        return f.read()

def compileShadersFromBytes(vBytes, fBytes):
    return compileProgram(
        compileShader(vBytes, GL_VERTEX_SHADER),
        compileShader(fBytes, GL_FRAGMENT_SHADER),
    )

def compileShadersFromFiles(vPath, fPath):
    return compileShadersFromBytes(
        loadShader(vPath),
        loadShader(fPath)
    )
    

  


# def main():
#     import glfw
#     dirname = os.path.dirname(__file__)
#     shader = compileShadersFromFiles(
#         os.path.join(dirname, "shaders", "ring.vs"),
#         os.path.join(dirname, "shaders", "ring.fs")
#         )
    
#     v,n,vi,ni = loadObj("ring0.obj")
    
#     # VBO = glGenBuffers(1)
#     # glBindBuffer(GL_ARRAY_BUFFER, VBO)
#     # glBufferData(GL_ARRAY_BUFFER, )

# if __name__ == "__main__":
#     main()