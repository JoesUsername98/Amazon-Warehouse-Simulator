import turtle
#from turtle import Turtle, Screen, ScrolledCanvas, RawTurtle, TurtleScreen
from tkinter import *
import numpy as np
#https://docs.python.org/3/library/turtle.html

#Prof Arthur Richards
#Drone Application, Systems and Control Guest Lecturer
#https://github.com/arthurrichards77/smply/blob/master/visibility.ipynb

class shape:
    def __init__(self, turtle, centre, shade):
        self.loc = centre
        self.turtle = turtle
        #self.R = shade[0]
        #self.G = shade[1]
        #self.B = shade[2]
        self.color = shade
        
    def get_centre(self):
        return self.loc

class circle(shape):
    def __init__(self, turtle, centre, shade, radius):
        super().__init__(turtle, centre, shade)
        self.r = radius
        self.loc = centre

    def draw(self):
        self.turtle.pu()
        self.turtle.goto(self.loc-np.array([0,self.r]))
        self.turtle.pd()
        
        self.turtle.setheading(0)
        #self.turtle.color(self.R,self.G,self.B)
        self.turtle.color(self.color)
        self.turtle.begin_fill()

        self.turtle.circle(self.r)

        self.turtle.end_fill()
        self.turtle.pu()

class rectangle(shape):
    def __init__(self, turtle, centre, shade, width, height):
        super().__init__(turtle, centre, shade)
        self.w = width
        self.h = height

        self.edge_loc = [[ self.loc[0]-0.5*self.w, self.loc[1] ],
                         [ self.loc[0], self.loc[1]+0.5*self.h ],
                         [ self.loc[0]+0.5*self.w, self.loc[1] ],
                         [ self.loc[0], self.loc[1]-0.5*self.h ]]
        #0.51 for visibility
        self.vrtx_loc = [[ self.loc[0]-0.5001*self.w, self.loc[1]+0.5001*self.h ],
                         [ self.loc[0]+0.5001*self.w, self.loc[1]+0.5001*self.h ],
                         [ self.loc[0]+0.5001*self.w, self.loc[1]-0.5001*self.h ],
                         [ self.loc[0]-0.5001*self.w, self.loc[1]-0.5001*self.h ]]
        
    def get_edge(self):
        return self.edge

    def get_vrtx(self):
        return self.vrtx
    
    def draw(self):
        self.turtle.pu()
        self.turtle.goto(self.loc-np.array([self.w/2,self.h/2]))
        self.turtle.pd()
        
        #self.turtle.color(self.R,self.G,self.B)
        self.turtle.color(self.color)
        self.turtle.begin_fill()

        self.turtle.setheading(0)
        self.turtle.fd(self.w)
        self.turtle.lt(90)
        self.turtle.fd(self.h)
        self.turtle.lt(90)
        self.turtle.fd(self.w)
        self.turtle.lt(90)
        self.turtle.fd(self.h)
        
        self.turtle.end_fill()
        self.turtle.pu()

class storage(rectangle):
    copyNO = 0
    def __init__(self, turtle, centre, shade, width, height, radius, parent):
        super().__init__(turtle, centre, shade, width, height)
        self.r = radius
        self.parent = parent
        self.ID = storage.copyNO
        storage.copyNO += 1
        
        self.edge_nodes=[]
        self.vrtx_nodes=[]
        for i in np.arange(4):
            #self.edge_nodes.append(edge(turtle, self.edge_loc[i], np.array([0,200,0]), (self.w+self.h)/60 ,self ))
            #self.vrtx_nodes.append(vrtx(turtle, self.vrtx_loc[i], np.array([0,0,200]), (self.w+self.h)/60 ,self ))
            self.edge_nodes.append(edge(turtle, self.edge_loc[i], "blue"  , (self.w+self.h)/60 ,self ))
            self.vrtx_nodes.append(vrtx(turtle, self.vrtx_loc[i], "lime" , (self.w+self.h)/60 ,self ))
            
    def draw(self):
        
        self.turtle.pu()
        self.turtle.goto(self.loc-np.array([self.w/2,self.h/2 - self.r]))
        self.turtle.pd()
        
        #self.turtle.color(self.R,self.G,self.B)
        self.turtle.color(self.color)
        self.turtle.begin_fill()

        self.turtle.setheading(-90)
        self.turtle.circle(self.r,90)
        self.turtle.fd(self.w-2*self.r)
        self.turtle.circle(self.r,90)
        self.turtle.fd(self.h-2*self.r)
        self.turtle.circle(self.r,90)
        self.turtle.fd(self.w-2*self.r)
        self.turtle.circle(self.r,90)
        self.turtle.fd(self.h-2*self.r)
        
        self.turtle.end_fill()
        self.turtle.pu()

        #draw nodes
        for i in np.arange(4):
            self.edge_nodes[i].draw()
            self.vrtx_nodes[i].draw()
            
    def get_edges(self):
        return self.edge_nodes

    def get_vrtxs(self):
        return self.vrtx_nodes
    
    def get_ID(self):
        return self.ID

    def get_parent(self):
        return self.parent

class node(circle):
    copyNO = 0
    info = []
    def __init__(self, turtle, centre, shade, radius, parent):
        super().__init__(turtle, centre, shade, radius)
        self.parent = parent
        self.loc = centre
        self.ID = node.copyNO
        node.copyNO += 1
        node.info.append(self)

    def get_ID(self):
        return self.ID

    def get_parent(self):
        return self.parent
    
    def get_info():
        return node.info

class edge(node):
    copyNO = 0
    info = []
    def __init__(self,turtle, centre, shade, radius, parent):
        super().__init__(turtle, centre, shade, radius, parent)
        self.ID = edge.copyNO
        edge.copyNO += 1
        edge.info.append(self)

    def get_ID(self):
        return self.ID

    def get_info():
        return edge.info

class vrtx(node):
    copyNO = 0
    info = []
    def __init__(self, turtle, centre, shade, radius, parent):
        super().__init__(turtle, centre, shade, radius, parent)
        self.ID = vrtx.copyNO
        vrtx.copyNO += 1
        vrtx.info.append(self)

    def get_ID(self):
        return self.ID

    def get_info():
        return vrtx.info

class Scenery():
    copyNO = 0
    def __init__(self, root, size, floor_ratio, storage_layout, bay_layout ):
        #Variables
        self.root = root
        self.size = size
        self.f_ratio = floor_ratio
        self.strg_layout = storage_layout
        self.bay_layout = bay_layout

        self.ID = Scenery.copyNO
        Scenery.copyNO += 1
        
    def build(self):
        
        self.canvas = turtle.Canvas(self.root)
        self.canvas.grid(row=0, rowspan=10, column=0)#side=LEFT)
        self.canvas.config(width=self.size[0], height=self.size[1])
        self.screen = turtle.TurtleScreen(self.canvas)
        
        self.builder = turtle.RawTurtle(self.canvas)
        self.builder.speed(0)
        self.builder.ht()
        self.builder.getscreen().tracer(0)
                                                #Colour #Box Colour #Box Layout #Path Width.
        self.Warehouse = Area(self, self.f_ratio, "red", "LightSalmon3", self.strg_layout, 0.4)
        self.Warehouse.draw()
        self.Bay =       Area(self, 1-self.f_ratio, "purple", "LightSalmon3", self.bay_layout, 0.4)
        self.Bay.draw()

        self.builder.getscreen().update()
        self.builder.getscreen().tracer(1,0)
    def clean_builders(self):
         del self.builder

    def get_canvas(self):
        return self.canvas
        
class Area(Scenery):
    copyNO = 0
    def __init__(self, parent, f_ratio, colour, box_colour, box_layout, path_width):
        super().__init__(parent.root, parent.size, parent.f_ratio, parent.strg_layout, parent.bay_layout)
        # in warehouse.
        self.colour = colour            
        self.box_colour = box_colour    
        self.box_layout = box_layout       
        self.path_w = path_width
        self.f_ratio = f_ratio
        
        if (Area.copyNO <1):
            shift = 1-self.f_ratio
        else:
            shift = self.f_ratio-1
            
        self.floor = rectangle(parent.builder, [0,(shift/2)*parent.size[1]], self.colour, parent.size[0], parent.size[1]*self.f_ratio)
        Area.copyNO += 1
        
        self.f_size = [parent.size[0], self.f_ratio* parent.size[1]]
        self.B_w = self.f_size[0]/(self.box_layout[0]+self.path_w*(self.box_layout[0]+1))
        self.B_h = self.f_size[1]/(self.box_layout[1]+self.path_w*(self.box_layout[1]+1))
        self.box = []
        for horz_box in range(self.box_layout[0]):
             for vert_box in range(self.box_layout[1]):
                 self.centre = [self.B_w*(horz_box+0.5)+(horz_box+1)*self.path_w*self.B_w - self.f_size[0]/2
                              ,self.B_h*(vert_box+0.5)+(vert_box+1)*self.path_w*self.B_h - self.f_size[1]/2 + (shift)/2*self.size[1]]
                 self.box.append(storage(parent.builder, self.centre ,self.box_colour, self.B_w , self.B_h, 5, self))

    def draw(self):
        self.floor.draw()
        for boxes in range(len(self.get_box())):
                self.box[boxes].draw()

    def get_edges(self):
        return self.box.get_edges()

    def get_vrtxs(self):
        return self.box.get_vrtxs()

    def get_box(self):
        return self.box

class Bot(Scenery):
    copyNO = 0
    def __init__( self , canvas):
        self.ID = Bot.copyNO
        Bot.copyNO += 1
        
        self.trtl = turtle.RawTurtle(canvas)
        self.trtl.pencolor("gold")
        self.trtl.pensize(3)
        self.trtl.speed(1)

    def do_route(self, path, coords):
        self.trtl.pu()
        self.trtl.goto(coords[path[0]][0],coords[path[0]][1])
        self.trtl.pd()

        for i in range(len(path)-1):
            self.trtl.setheading( self.trtl.towards(coords[path[i+1]][0], coords[path[i+1]][1] ) )
            self.trtl.fd( self.trtl.distance( coords[path[i+1]][0], coords[path[i+1]][1] ))
        #self.trtl.pu()

    def clear_paths(self):
        self.trtl.clear()

    def __delete__(self):
        self.clear_paths(self)
        del self.trtl
        del self

    def stamp(self):
        self.trtl.stamp()
        
if __name__ == "__main__":
    root = Tk()
    root.title("Amazon Warehouse Simulator")
    args = [[350,450], [0.8], [5,5], [3,1]]
    Scene = Scenery(root ,args[0], args[1][0], args[2], args[3])
    Scene.build()
    Scene.clean_builders()
