from Scene import *
from Brain import *
import tkinter as tk

class custom_path():
    def __init__(self):
        self.path = []

    def add_to_path(self, new_path):
        self.path.append(new_path)

    def remove_path(self, row = -1):
        del self.path[row]

    def get_path(self):
        return(self.path)

    def clear_path(self):
        self.parth = []
    
def updatePreview(varRow,varCol,Brain,labelText):
    row = int(varRow.get())-1
    col = int(varCol.get())-1
    temp = Brain.get_obstacles()[col][row]
    labelText.set("{:03.1f} , {:03.1f}".format(temp[0],temp[1]))

def add_path(varRow,varCol,Brain,TextField,MyPath,labelText):

    updatePreview(varRow,varCol,Brain,labelText)
    TextField.config(state=NORMAL)
    
    row = int(varRow.get())-1
    col = int(varCol.get())-1
    new_path = Brain.get_obstacles()[col][row]
    MyPath.add_to_path(new_path)
    TextField.insert(END,"{:03.1f} , {:03.1f}\n".format(new_path[0],new_path[1]))

    TextField.config(state=DISABLED)

def clear(TextField,MyPath):
    TextField.config(state=NORMAL)

    MyPath.clear_path()
    TextField.delete(1.0,END)
    
    TextField.config(state=DISABLED)
    
def createNewWindow(root, Scene, Brain, MyPath):
    newWindow = tk.Toplevel()
    #newWindow.wait_window(newWindow)
    
    #Drop Down Prep
    DropRowOPS = []
    DropColOPS = []
    for i in range(np.shape(Brain.get_obstacles())[1]):
        DropRowOPS.append(str(i+1))
    for i in range(np.shape(Brain.get_obstacles())[0]):
        DropColOPS.append(str(i+1))
    
    labelInstruct = tk.Label(newWindow, text = "Manual Path Select Window")
    labelInstruct.grid(row=0, column=0, padx=10 ,pady=10 )
    labelnewCoords = tk.Label(newWindow, text = "New Coords")
    labelnewCoords.grid(row=2, column=0, padx=10 ,pady=10 )
    labelRow = tk.Label(newWindow, text = "Node")
    labelRow.grid(row=1, column=1, padx=10 ,pady=10 )
    labelCol = tk.Label(newWindow, text = "Storage")
    labelCol.grid(row=1, column=2, padx=10 ,pady=10 )
    labelPreview = tk.Label(newWindow, text = "Preview")
    labelPreview.grid(row=1, column=3, padx=10 ,pady=10 )
    labelText = StringVar()
    CoordsPreview = tk.Label(newWindow, textvariable=labelText)
    CoordsPreview.grid(row=2, column=3, padx=10, pady=10) #, state='disabled'

    varRow = StringVar(root)
    varRow.set(DropRowOPS[0])
    varCol = StringVar(root)
    varCol.set(DropColOPS[0])
    DropRow = tk.OptionMenu(newWindow, varRow, *DropRowOPS)#, command =lambda: updatePreview(varRow,varCol,Brain,labelText))
    DropRow.grid(row=2, column=1, padx=10 ,pady=10 )
    DropCol = tk.OptionMenu(newWindow, varCol, *DropColOPS)#, command =lambda: updatePreview(varRow,varCol,Brain,labelText))
    DropCol.grid(row=2, column=2, padx=10 ,pady=10 )

    buttonUpdate = tk.Button(newWindow, text= "Update Preview",command =lambda: updatePreview(varRow,varCol,Brain,labelText))
    buttonUpdate.grid(row=2, column=4, padx=10 ,pady=10 )
    
    TextField = tk.Text(newWindow, width=20, height = 5)
    TextField.grid(row=3,column=0, padx=10, pady=10)
    TextField.config(state=DISABLED)

    buttonADD = tk.Button(newWindow, text = "Add",command =lambda: add_path(varRow,varCol,Brain,TextField,MyPath,labelText))
    buttonADD.grid(row=2, column=5, padx=10 ,pady=10 )

    buttonClear = tk.Button(newWindow, text = "Clear",command =lambda: clear(TextField,MyPath))
    buttonClear.grid(row=3, column=1, padx=10 ,pady=10 )

    buttonCust_TSP = tk.Button(newWindow, text = "Do TSP",command =lambda: newWindow.destroy())
    buttonCust_TSP.grid(row=3, column=2, padx=10 ,pady=10 )

    root.wait_window(newWindow)

def do_TSP(Scene, Brain, bot, do_rand, root):
    do_plot= True
    if do_rand:#generate points for random
        points = []
        tsp_coords = [extract_random_points(Scene.Bay)]
        destinations = 5
        for dest in range(destinations-1): # -1 for start 
            rand = extract_random_points(Scene.Warehouse)
            tsp_coords.append( rand )

    else:
        MyPath = custom_path()
        createNewWindow(root, Scene, Brain, MyPath)
        print("I am out of newWindow and back in mainn loop ")
        tsp_coords = MyPath.get_path()
        
    newpath,success = random_TSP(tsp_coords, do_plot, Scene, Brain.get_all_points()
                         , Brain.get_distance(), Brain.get_C())
    if(success==False):
        print("Error Occured when attempting this path")
        print(tsp_coords)
    for sub_path in newpath:
        bot.do_route(sub_path,Brain.get_all_points())
        bot.stamp()
    

def plot_me():
    plt.show()
    plot_butt.configure(state=DISABLED)
    

def close_window(root):
    plt.close()
    root.destroy()

# Initialise GUI
root = Tk()
root.title("Amazon Warehouse Simulator")
            
# Build Scenery
args = [[700,900], [0.8], [5,5], [3,1]]
Scene = Scenery(root, args[0], args[1][0], args[2], args[3])
Scene.build()
Scene.clean_builders()

# Build Bot and Brain
Bot_Carl = Bot(Scene.get_canvas())
Brain = brain()
do_plot= True

# Generate Points, get and set
do_plot= True
all_points,obstacles = generate_points(Scene,do_plot)
Brain.set_all_points(all_points)
Brain.set_obstacles(obstacles)

# Generate Roadmap    
do_plot= True
C = Roadmap_Gen(Brain.get_all_points(), Brain.get_obstacles(), do_plot, args)
Brain.set_C(C)

# Find Shortest Path
distance,predecessors = shortest_path(Brain.get_C(), return_predecessors=True)
Brain.set_distance(distance)
Brain.set_predecessors(predecessors)

#Radio Buttons
tk.Label(root, text="Choose TSP destinations:",justify = tk.LEFT,padx = 20).grid(row=1, column =1)
MODES = [("Random", True),("Specified", False)]
do_rand = BooleanVar()
do_rand.set(True)
i=2
for text, mode in MODES:
    b = Radiobutton(root, text=text, variable=do_rand, value=mode)
    b.grid(row=i, column=1)
    i +=1

# Buttons 
rand_TSP_butt = Button(root, text="TSP", command =lambda: do_TSP(Scene, Brain, Bot_Carl, do_rand.get(), root ))
rand_TSP_butt.grid(row=0, column=1, padx=10)

clear_butt = Button(root, text="Clear", command =lambda: Bot_Carl.clear_paths() )
clear_butt.grid(row=i+1, column=1,  padx=10)

plot_butt = Button(root, text="Plot", command =lambda:  plot_me())
plot_butt.grid(row=i+2, column=1,  padx=10)

close_butt = Button(root, text="Close", command =lambda: close_window(root) )
close_butt.grid(row=i+3, column=1,  padx=10)

root.mainloop()
