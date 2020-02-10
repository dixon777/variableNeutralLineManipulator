import tkinter as tk

import matplotlib as plt
plt.use("TkAgg")
import matplotlib.backends.backend_tkagg as plttk
import matplotlib.figure as pltf

def createPlot(master):
    f = pltf.Figure(figsize=(3,3), dpi=100)
    a = f.add_subplot(111)
    a.plot([1,2,3,4,5,6,7,8],[5,6,1,3,8,9,3,5])
    
    canvasTkagg = plttk.FigureCanvasTkAgg(f, master)
    # canvasTkagg.draw()
    
    toolbar = plttk.NavigationToolbar2Tk(canvasTkagg, master)
    # toolbar.update()
    
    canvas = canvasTkagg.get_tk_widget()
    canvas.config(bd=0, highlightthickness=0)
    canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

def createDynamicConfigLayout(master):
    pass

def createConfigLayout(master):
    pass

def createWidgets(master):
    topFrame = tk.Frame(master, bg='cyan')
    topFrame.pack(fill=tk.BOTH, expand=2)
    bottomFrame = tk.Frame(master, bg='green', height=150)
    bottomFrame.pack(fill=tk.X, expand=0)
    
    topRightFrame = tk.Frame(topFrame, bg='red', width=250)
    topRightFrame.pack(fill=tk.Y,side=tk.RIGHT)
    topLeftFrame = tk.Frame(topFrame, bg='yellow')
    topLeftFrame.pack(fill=tk.BOTH,side=tk.RIGHT, expand=4)
    
    createConfigLayout(bottomFrame)
    createPlot(topLeftFrame)
    createDynamicConfigLayout(topRightFrame)
    
    
    
 
def main():
    master = tk.Tk()
    master.minsize(600,400) 
    createWidgets(master)
    master.mainloop()

if __name__ == "__main__":
    main()
    