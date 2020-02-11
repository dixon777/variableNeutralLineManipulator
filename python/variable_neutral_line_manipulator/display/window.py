import math, os,logging

import numpy as np
import pyrr

from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (QApplication, QVBoxLayout, QOpenGLWidget, QSlider,
                             QWidget)


from OpenGL.GL import *

from opengl_helper import *

logging.getLogger().setLevel(logging.INFO)

class GLWidget(QOpenGLWidget):
    xChange = pyqtSignal(int)
    yChange = pyqtSignal(int)
    def __init__(self, parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        self.x = 0
        self.y = 0
        self.z = 0
        self.rot = np.identity(4, dtype=np.float32)
        self.acc_x = 0
        self.acc_y = 0
        
        self.setFocusPolicy(Qt.StrongFocus)
        
    def setVal(self, attr, val):
        if not np.array_equal(val, self.__dict__[attr]):
            self.__dict__[attr] = val
            self.update()
            logging.debug(f"{attr} = {val}")
            
        
    def setX(self, val):
        self.setVal("x", val)
    
    def setY(self, val):
        self.setVal("y", val)
            
    def setZ(self, val):
        self.setVal("z", val)
            
    def setRotation(self, mat):
        self.setVal("rot", np.matmul(mat,self.rot))
        
    def mousePressEvent(self, event):
        self.lastPos = event.pos()
        
    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()
        
        if event.buttons() & Qt.LeftButton:
            v = np.array((dy,dx,0))
            v = v/np.linalg.norm(v)
            mat = pyrr.matrix44.create_from_axis_rotation(
                v,
                math.sqrt(dx**2 + dy**2)/20,
            )
            self.setRotation(mat)
            
        elif event.buttons() & Qt.RightButton:
            self.acc_x += dx
            self.acc_y -= dy
            translateRatio = 20
            self.xChange.emit(self.x*20 + self.acc_x // translateRatio)
            self.yChange.emit(self.y*20 + self.acc_y // translateRatio)
            self.acc_x %= translateRatio
            self.acc_y %= translateRatio

        self.lastPos = event.pos()
        
        

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Shift:
            self.inTranslateMode = True
    
    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Shift:
            self.inTranslateMode = False
        
    def initializeGL(self):
        logging.debug("initializeGL")
        logging.info(getGLInfo())
        
        shaderProgram = self.compileShaderProgram()
        v,c,e = self.createTriangleBuffer()
        self.configBuffer([v,c], e)
        self.indexArraySize = e.size
        
        uniformMap = {key: UniformVariable(glGetUniformLocation(shaderProgram, key), m) for key, m in zip((
            "transform", "view", "projection"
        ), (
            np.identity(4, dtype="float32"),
            pyrr.matrix44.create_from_translation(
                               np.array((0,0, -3)), dtype=np.float32
                           ),
            np.identity(4, dtype="float32"),
        ))}
        self.renderer = Renderer([Mesh(vertices=[v,c],indices=e,),], shaderProgram, uniformMap)

        self.renderer.useProgram()
        glClearColor(0,0.1,0.1,1)
        
    def paintGL(self):
        logging.debug("paintGL")
        self.renderer.useProgram()
        glClear(GL_COLOR_BUFFER_BIT)
        self.renderer.updateUniform("transform", pyrr.matrix44.create_from_translation(
            np.array((self.x, self.y, self.z), dtype=np.float32)
        ))
        self.renderer.draw()

    def resizeGL(self, width, height):
        logging.debug("resizeGL")
        self.renderer.updateUniform("projection", pyrr.matrix44.create_perspective_projection(
                               30,
                               width/height,
                               0.1,
                               10, dtype=np.float32
                           ))
        
    def sizeHint(self):
        return QSize(1280,720)
      
    def minimumSizeHint(self):
        return QSize(50,50)
    
    def compileShaderProgram(self):
        dirName = os.path.dirname(__file__)
        return compileShadersFromFiles(
            os.path.join(dirName, "shaders", "simple.vs"),
            os.path.join(dirName, "shaders", "simple.fs")
        )
    
    def createTriangleBuffer(self):
        v = 0.1*np.array((-1, -1, 0,
                      1, -1, 0,
                      -1, 1, 0,
                      1, 1, 0), dtype=np.float32)
        c = np.array((
            1, 0, 0,
            0, 1, 0,
            0, 0, 1,
            1, 0, 0
        ), dtype=np.float32)
        e = np.array((
            0,1,2,2,1,3,
        ), dtype=np.uint32)
        
        return np.reshape(v,(-1,3)),np.reshape(c,(-1,3)),e
    
    def configBuffer(self, vs, e):
        VBO = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, VBO)
        concatenated = np.concatenate(vs, axis=1)
        glBufferData(GL_ARRAY_BUFFER, concatenated.nbytes, concatenated.flatten(), GL_STATIC_DRAW)
        
        
        
        itemsize = concatenated.itemsize
        strife = itemsize*concatenated.shape[1]
        rowItemOffset = 0
        for i, v in enumerate(vs):
            glEnableVertexAttribArray(i)
            glVertexAttribPointer(i, v.shape[1], GL_FLOAT, GL_FALSE, strife, ctypes.c_void_p(rowItemOffset*itemsize))
            rowItemOffset += v.shape[1]
            
        IBO = glGenBuffers(1)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, e.nbytes, e, GL_STATIC_DRAW)
        
 

class Window(QWidget):
    def __init__(self, parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        self.bound = (-10,10)
        
        self.glWidget = GLWidget()
        self.sliders = [self.createSlider() for _ in range(3)]
        
        diff = self.bound[1] - self.bound[0]
        self.sliders[0].valueChanged.connect(lambda v: self.glWidget.setX(float(v)/ diff))
        self.sliders[1].valueChanged.connect(lambda v: self.glWidget.setY(float(v)/ diff))
        self.sliders[2].valueChanged.connect(lambda v: self.glWidget.setZ(float(v)/ diff))
        self.glWidget.xChange.connect(self.sliders[0].setValue)
        self.glWidget.yChange.connect(self.sliders[1].setValue)

        mainLayout = QVBoxLayout()
        mainLayout.addWidget(self.glWidget)
        for s in self.sliders:
            mainLayout.addWidget(s)
        self.setLayout(mainLayout)    
        
        self.setWindowTitle("OpenGL Pure Translation Example")
        
        for s in self.sliders:
            s.setValue(0)
            
            
        
    def createSlider(self):        
        slider = QSlider(Qt.Horizontal)
        slider.setRange(*self.bound)
        slider.setSingleStep(1)
        slider.setPageStep(slider.singleStep())
        slider.setTickInterval(slider.singleStep())
        slider.setTickPosition(QSlider.TicksBothSides)
        return slider
        
if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    win = Window()
    win.show()
    sys.exit(app.exec_())
