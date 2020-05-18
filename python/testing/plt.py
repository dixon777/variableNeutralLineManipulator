from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot

figure = pyplot.figure()
axes = mplot3d.Axes3D(figure)

# Load the STL files and add the vectors to the plot
your_mesh = mesh.Mesh.from_file('tmp\\a.stl')

# Auto scale to the mesh size
scale = your_mesh.points.flatten('A')
axes.auto_scale_xyz(scale, scale, scale)

# Show the plot to the screen
pyplot.show()