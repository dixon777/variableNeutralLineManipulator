import logging
logging.getLogger("l").setLevel(logging.INFO)
logging.getLogger("l").log(logging.INFO, "A")
# import numpy as np
# import pygame
# from pygame.locals import *

# from OpenGL.GL import *
# from OpenGL.GLU import *

# def p(x):
#     print(x)
    
# m = np.mgrid[-1:2:2, -1:2:2, -1:2:2]

# v = [(m[0,i,j,k], m[1,i,j,k], m[2,i,j,k]) for i in range(2) for j in range(2) for k in range(2)]

# e = [
#     (0,1),
#     (0,2),
#     (0,4),
#     (1,3),
#     (1,5),
#     (2,3),
#     (2,6),
#     (3,7),
#     (4,5),
#     (4,6),
#     (5,7),
#     (6,7)
# ]

# def cube():
#     glBegin(GL_LINES)
#     for edge in e:
#         for i in edge:
#             glVertex3fv(v[i])
#     glEnd()
    
# def meshgrid
    
# def main():
#     pygame.init()
#     display = (800, 600)
#     pygame.display.set_mode(display,DOUBLEBUF|OPENGL)
    
#     gluPerspective(45, (display[0]/display[1]), 0.1, 50)
#     glTranslatef(0,0,-5)
    
#     while True:
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT:
#                 pygame.quit()
#                 quit()
                
#         glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
#         cube()
#         glRotatef(1,2,2,2)
#         glRotatef(1,3,2,2)
#         pygame.display.flip()
#         pygame.time.wait(10)
        
# main()
