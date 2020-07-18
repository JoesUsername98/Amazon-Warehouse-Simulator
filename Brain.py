#Path finding techniques credit to:
# Prof. A Richards UoB
# Drone Control, Systems and application Guest Lecturer
# https://github.com/arthurrichards77/smply
# Marked by #**

import matplotlib.pyplot as plt
#%matplotlib inline
import numpy as np
from scipy.sparse.csgraph import shortest_path
import random
from pulp import *
import time
import os

from Scene import *
# My module.

#**
def plot_poly(points,fmt='b-',**kwargs):
    plt.plot(np.append(points[:,0],points[0,0]),np.append(points[:,1],points[0,1]),fmt)

#**
def lines_cross(a,b,c,d):
    M = np.array([[b[0]-a[0],c[0]-d[0]],[b[1]-a[1],c[1]-d[1]]])
    if np.linalg.det(M)==0.:
        return(False)
    v = np.array([[c[0]-a[0]],[c[1]-a[1]]])
    w = np.linalg.solve(M,v)
    #If any value in w between 0 and 1 then it crosses. 
    if w[0]<=0:
        return(False)
    elif w[0]>=1:
        return(False)
    elif w[1]<=0:
        return(False)
    elif w[1]>=1:
        return(False)
    else:
        return(True)

#** small modification
def line_crosses_obst(a,b,obst):            
    num_crosses = 0
    for ii in range(len(obst)): 
        if lines_cross(a,b,obst[ii-1],obst[ii]):
            return(True)
        if lines_cross(0.5*(a+b),[max(obst[:,0])+0.01,max(obst[:,1])+0.01],obst[ii-1],obst[ii]):
            num_crosses = num_crosses+1
    if num_crosses%2==1:
        return(True) # Don't draw. 
    return(False)

#My own
def are_adjacent(a,b,obst):
    a_index = where_in(a,obst)
    b_index = where_in(b,obst)
    if (a_index != -999) and (b_index != -999):
        if (a_index == b_index-1) or (b_index == a_index-1):
            return(True)
        elif(a_index == len(obst)-1 and b_index == 0) or (b_index == len(obst)-1 and a_index == 0):
            return(True)
    return(False)

#My own
def where_in(point,obst):
    for i in range(len(obst)):
        if(((obst[i]==point)[0] == True and (obst[i]==point)[1] == True)):
            return(i)
    return(-999)

#My own
##def passes_thru(a,b,ob):
##    tol = 10
##    if(a[0]-b[0] != 0):
##        m = (a[1]-b[1])/(a[0]-b[0])
##        c = a[1]-a[0]*m
##        centre = (np.average(ob,0))
##        return abs(centre[1]-(m*centre[0]+c)) < tol
##    return False

#My own
def thru_centre(a,b,ob):
    tol = 0.001
    centre= np.average(ob,0)
    if (where_in(a, ob)!=-999) or (where_in(b, ob)!=-999): #a or b belong to this ob.
        if (a[0]== b[0]): #Same x : Vertical.
            if (( centre[1] < a[1] and centre[1] > b[1]) or ( centre[1] > a[1] and centre[1] < b[1])):
                # if centre is imbetween a and b
                if ( abs(a[0] - centre[0]) <tol):
                    return(True)
        if (a[1]== b[1]):
            if (( centre[0] < a[0] and centre[0] > b[0]) or ( centre[0] > a[0] and centre[0] < b[0])):
                if ( abs(a[1] - centre[1]) <tol):
                    return(True)
    offset= np.array([15,-4])
    return ((lines_cross(a,b,centre+offset,centre-offset))or(lines_cross(a,b,centre-offset,centre+offset))) 


#** Modified
def is_visible(a,b,obstacles):
    for ob in obstacles:
        if are_adjacent(a,b,ob):
            return(True)
        if thru_centre(a,b,ob):
           return(False) 
##        if passes_thru(a,b,ob):
##            return(False) 
        if line_crosses_obst(a,b,ob): 
            return(False)
    return(True)

#My own
def extract_points(area, extract_edges = True, extract_vrtxes = True):
    obstacles = []
    temp = []
    for boxes in range(len(area.get_box())):
        box_coords = []
        for points in range(len(area.get_box()[boxes].get_vrtxs())):
            if extract_edges:
                box_coords.append(area.get_box()[boxes].get_edges()[points].get_centre())
            if extract_vrtxes:
                box_coords.append(area.get_box()[boxes].get_vrtxs()[points].get_centre())
        obstacles.append(np.array( box_coords ))
    return obstacles
#My own
def extract_random_points(area, extract_edges = True, extract_vrtxes = False):
    all_box_nodes = extract_points(area,extract_edges,extract_vrtxes)
    a_boxes_nodes = all_box_nodes[random.randint(0,len(all_box_nodes)-1)]
    a_node = a_boxes_nodes[random.randint(0,len(a_boxes_nodes)-1)]
    return a_node

def Oned_to_Twod(index, TwoD):
    NoInArray = len(TwoD[0])
    firstIndex = int( (index-index%NoInArray) /NoInArray )
    secondIndex = int( index%NoInArray )
    return( TwoD[firstIndex][secondIndex] )

def get_path(C ,all_points, do_plot ,start ,goal): #= extract_random_points(Scene.Warehouse)         
    distance,predecessors = shortest_path(C, return_predecessors=True)

    #Random Start and finish points.
    #goal = extract_random_points(Scene.Bay)
    #start = extract_random_points(Scene.Warehouse)
    if do_plot:
        plt.plot(start[0],start[1],'mo', markersize=12, linewidth=3)
        plt.plot(goal[0],goal[1],'mx', markersize=12, linewidth=3)

    #curr_node = 1 #Start Index
    start_node = where_in(start,all_points)
    end_node = where_in(goal,all_points)
    curr_node = start_node
    path = [start_node]
    for kk in range(len(all_points)):
        next_node = predecessors[end_node,curr_node]
        path.append(next_node)
        if do_plot:
            plt.plot(all_points[[curr_node,next_node],0],all_points[[curr_node,next_node],1],'m-', linewidth=3)
        curr_node=next_node
        if curr_node==end_node: #Finish Index
            break
    return path, distance,predecessors 


## Moved in to Scene...
##def route(path,all_points):        
##    Mover = turtle.Turtle()
##    Mover.pu()
##    Mover.goto(all_points[path[0]][0],all_points[path[0]][1])
##    Mover.pd()
##    Mover.pencolor((150, 150, 30))
##    Mover.pensize(10)
##    Mover.speed(3)
##    for i in range(len(path)-1):
##        Mover.setheading( Mover.towards(all_points[path[i+1]][0], all_points[path[i+1]][1] ) )
##        Mover.fd( Mover.distance( all_points[path[i+1]][0], all_points[path[i+1]][1] ))
##    Mover.pu()
    #del Mover

def sort_path(path):
    sorted_items = [path[0]]
    unsorted_items = []
    i = 0
    ii = 0 
    while i <len(path)-1:
        if path[ii][-1]== path[i+1][0]:
            sorted_items.append(path[i+1])
            ii+=1
        else:
            unsorted_items.append(path[i+1])
            perfect = False
        i+=1
    if len(unsorted_items)==0:
        return path
    else :
        new_order = sorted_items + unsorted_items
        return sort_path(new_order)

def random_TSP(start, do_plot):
    pickup_no = 5
    Feasibility="Infeasible"

    while(Feasibility=="Infeasible"):
        tsp_coords = []
        tsp_nodes = []
        for pickups in range(pickup_no-1): # - 1 for start 
            rand = extract_random_points(Scene.Warehouse)
            tsp_coords.append( rand )# Random wherehouse edge
            tsp_nodes.append(where_in(tsp_coords[pickups],all_points) )# Where are they in all_points

        #start = extract_random_points(Scene.Bay) # Random start
        tsp_coords = [start] + tsp_coords
        tsp_nodes  = [where_in(start,all_points)] + tsp_nodes 

        links =[]
        links = [(i,j) for i in tsp_nodes for j in tsp_nodes if (j!=i)]# and not( (j==tsp_nodes[0] and i==tsp_nodes[pickup_no-1]) or (j==tsp_nodes[pickup_no-1] and i==tsp_nodes[0]) )) ]
        # do not link same nodes to eachother or do not link the start and end.
        
        prob = LpProblem('tsp',LpMinimize)
        x = LpVariable.dicts("x",links,0,1,LpInteger)
        prob.setObjective(sum([distance[i,j]*x[i,j] for (i,j) in links]))
        for i in tsp_nodes:
            prob += (sum(x[ic,j] for (ic,j) in links if ic==i)==1)
        for j in tsp_nodes:
            prob += (sum(x[i,jc] for (i,jc) in links if jc==j)==1)
        v = LpVariable.dicts("v",tsp_nodes,0,pickup_no)
        for (i,j) in links:
            if j!=tsp_nodes[0]:# or j==tsp_nodes[-1]:
                prob += (v[j] >= v[i]+1 - pickup_no*( 1 - x[i,j] ))

        prob.solve()
        Feasibility = LpStatus[prob.status]
        # remove me when happy. 
        #print("Status:", Feasibility)

    path = []
    path_coords=[]
    for (ii,jj) in links:
        if (x[ii,jj].value()==1):
            start = all_points[ii]
            end = all_points[jj]
            pp,_,_ = get_path(C,all_points,do_plot, start , end)
            path.append(pp)
            path_coords.append(start)
            if do_plot:
                for points in range(len(pp)):
                    plt.plot(all_points[pp[points]][0],all_points[pp[points]][1],'b-')

    newpath = sort_path(path)
    return(newpath)

def generate_points(Scene, do_plot):
    obstacles = []
    Warehouse_pts = extract_points(Scene.Warehouse)
    Bay_pts = extract_points(Scene.Bay)
    for i in range(len(Warehouse_pts)): obstacles.append(Warehouse_pts[i]) 
    for i in range(len(Bay_pts)): obstacles.append(Bay_pts[i])

    all_points = np.zeros([1,2])
    for ob in obstacles:
        all_points = np.append(all_points,ob[:],0)
        if do_plot:
            plot_poly(ob,'r-')
    all_points = np.delete(all_points, 0, 0)

    return(all_points,obstacles)

def Roadmap_Gen(all_points, obstacles, do_plot, args):
    name = ""
    for i in range(len(args)):
        for j in range(len(args[i])):
            name = name + str(args[i][j]) + "-"
    longpath = "C:/Users/joeos/OneDrive/Documents/Amazon Warehouse/"
    path = longpath+"Roadmaps/" +name+".csv"
    if os.path.exists(path):
        C = np.genfromtxt(path, delimiter=',')
        print("Loading Roadmap")
    else:
        path = longpath+"Roadmaps/" +name+".csv"
        print("Generating Roadmap")
        C = np.inf+np.zeros((len(all_points),len(all_points)))
        i=1
        for ii in range(len(all_points)):
            for jj in range(ii+1,len(all_points)):
                if is_visible(all_points[ii],all_points[jj],obstacles):
                    if do_plot:
                        plt.plot(all_points[[ii,jj],0],all_points[[ii,jj],1],'g--') #Plots roadmap paths
                    C[ii,jj]=np.linalg.norm(all_points[ii]-all_points[jj])
                    C[jj,ii]=C[ii,jj]#RoadMap
        np.savetxt(path, C, delimiter=",")
    return(C)
#############################################################################################
#Builds Warehouse areas# put in a main file when I can.
#############################################################################################
args = [[700,900], [0.8], [5,5], [3,1]]
Scene = Scenery(args[0], args[1][0], args[2], args[3])
Scene.build()
Scene.clean_builders()
Bot_Carl = Bot()
do_plot= True

#############################################################################################
#Builds Warehouse areas# put in a "Generate Roadmap Func"  
#############################################################################################
do_plot= True
all_points,obstacles = generate_points(Scene,do_plot)

#############################################################################################
#Roadmap Generation# only run me once | put in a "Generate Roadmap Func" 
#############################################################################################
do_plot= False
C = Roadmap_Gen(all_points, obstacles, do_plot, args)
distance,predecessors = shortest_path(C, return_predecessors=True)
                    
#############################################################################################
#Travelling Sales Man# put me in a bot class
#############################################################################################

do_plot= True
newpath=random_TSP(extract_random_points(Scene.Bay), do_plot)

for sub_path in newpath:
    Bot_Carl.do_route(sub_path,all_points)
        
if do_plot:
    plt.show()
