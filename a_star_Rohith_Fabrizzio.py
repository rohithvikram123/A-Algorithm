import numpy as np
import cv2 as cv
from queue import PriorityQueue
import math

boundry = []    
Pth = {}        #Stores the path for backtracking
UncheckedList = PriorityQueue()     #Used to store unvisited nodes
b_track = []            
CloseList = []
CheckedList = np.zeros((250,600),dtype='uint8')     # Used to store the visited nodes
#Creating the Obstacle Space
#Obtacle with Obstacle, Obstacle Clearance and Robot Radius
def obstacle_space(space):
    h,w,_ = space.shape
    for l in range(h):
        for m in range(w):
            if ((250-l) - 5 < 0) or ((m) - 5 < 0) or ((250-l) - 245 > 0) or ((m) - 595 > 0): #boundary
                space[l][m] = [0,0,255]
                
            if (m > (100-(Obstacle_Clearance+Robot_Radius))) and (m < (151+(Obstacle_Clearance+Robot_Radius))) and (250-l < 101+(Obstacle_Clearance+Robot_Radius)) and (250-l > 2): #rectangle1 with robot radius clearance
                space[l][m] = [20,0,0]
                
            if (m > 100-Obstacle_Clearance) and (m < 151+Obstacle_Clearance) and (250-l < 101+Obstacle_Clearance) and (250-l >2):     #rectangle1 with boundry clearance
                space[l][m] = [0,255,0]
                
            if (m > 100) and (m < 151) and (250-l < 101) and (250-l > 2):   #rectangle1
                space[l][m] = [0,0,255]
                
            if(m > 100-(Obstacle_Clearance+Robot_Radius)) and (m < 151+(Obstacle_Clearance+Robot_Radius)) and (250-l >150-(Obstacle_Clearance+Robot_Radius)) and (250-l < 248):
                space[l][m] = [20,0,0]
                
            if(m > 100-Obstacle_Clearance) and (m < 151 + Obstacle_Clearance) and (250-l >150-Obstacle_Clearance) and (250-l < 248):
                space[l][m] = [0,255,0]
                
            if (m > 100) and (m < 151) and (250-l >150) and (250-l < 248):  #rectangle2
                space[l][m] = [0,0,255]
            new_v1 = 223.205 - Obstacle_Clearance*1.154
            new_v2 = 26.795 + Obstacle_Clearance*1.154
            new_v3 = 373.207 + Obstacle_Clearance*1.154
            new_v4 = 123.207 +Obstacle_Clearance*1.154    
            new_c1 = new_v1 - Robot_Radius*1.154
            new_c2 = new_v2 + Robot_Radius*1.154
            new_c3 = new_v3 + Robot_Radius*1.154
            new_c4 = new_v4 + Robot_Radius*1.154
            if (((0.577*m)+(250-l)-new_c1)>=0) and ((m-235.048+(Obstacle_Clearance+Robot_Radius))>=0) and (((-0.577*m)+(250-l)-new_c2<=0)) and (((0.577*m)+(250-l)-new_c3)<=0) and ((m-364.951-(Obstacle_Clearance+Robot_Radius))<=0) and (((-0.577*m)+(250-l)+new_c4)>=0):
                space[l][m] = [20,0,0]
                
            if (((0.577*m)+(250-l)-new_v1)>=0) and ((m-235.048+Obstacle_Clearance)>=0) and (((-0.577*m) + (250-l)-new_v2<=0)) and (((0.577*m)+(250-l)- new_v3)<=0) and ((m-364.951-Obstacle_Clearance)<=0) and (((-0.577*m)+(250-l)+new_v4)>=0):
                space[l][m] = [0,255,0]
                
            if (((9375*m)+(16238*(250-l))-3624400)>=0) and (((125*m)-29381)>=0) and (((9375*m)-(16238*(250-l))+435100)>=0) and (((37500*m)+(64951*(250-l))-24240200)<=0) and (((1000*m)-364951)<=0) and (((37500*m)-(64951*(250-l))-8002450)<=0):   #hexagon
                space[l][m] = [0,0,255]

            t_v1 = 1145 + Obstacle_Clearance*2.23
            t_v2 = 895 + Obstacle_Clearance*2.23    
            t_c1 = t_v1 + Robot_Radius*2.23
            t_c2 = t_v2 + Robot_Radius*2.23
            if(((m-460+(Obstacle_Clearance+Robot_Radius))>=0)) and (((2*m)+(250-l)-t_c1)<=0) and (((2*m)-(250-l)-t_c2)<=0) and ((250-l)>25-(Obstacle_Clearance+Robot_Radius)) and ((250-l)<(225+Obstacle_Clearance+Robot_Radius)):
                space[l][m] = [20,0,0]
                
            if ((m-460+Obstacle_Clearance)>=0) and (((2*m)+(250-l)-t_v1)<=0) and (((2*m)-(250-l)-t_v2)<=0) and ((250-l)>(25-Obstacle_Clearance)) and ((250-l)<(225+Obstacle_Clearance)):
                space[l][m] = [0,255,0]
                
            if (((m-460)>=0)) and(((2*m)+(250-l)-1145)<=0)and (((2*m)-(250-l)-895)<=0): #triangle
                space[l][m] = [0,0,255]
                
    return 
#taking the obstacle coordinates into a list
def resize_obstacle(space):
    h,w,_ = space.shape
    for l in range(h):
        for m in range(w):
            if space[l][m][2] == 255:
                boundry.append((m,250-l))
            if space[l][m][1] == 255:
                boundry.append((m,250-l))
            if space[l][m][0] == 20:
                boundry.append((m,250-l))
    return boundry




#Getting User Inputs For the Start node from the user
def User_Inputs_Start(Obs_Coords):
    while True:
        x = int(input("Enter the Initial x node: "))
        y = int(input("Enter the Initial y node: "))
        z = int(input("Enter the orientation of robot at start pt.(in degrees): "))
        
        if((x>=0) or (x<=600)) and (y>=0) or (y<=250):
            if (x,y) not in Obs_Coords and ((x + Robot_Radius),y) not in Obs_Coords and ((x-Robot_Radius),y) not in Obs_Coords and (x,(y+Robot_Radius)) not in Obs_Coords  and (x,(y-Robot_Radius)) not in Obs_Coords:
                start_node = (x,y,z)
                return start_node
            else:
                print("The Entered Start Node is in obstacle space")
#Getting User Input for the Goal Node from the user
def User_Inputs_Goal(Obs_Coords):
    while True:
        x = int(input("Enter the Goal x node: "))
        y = int(input("Enter the Goal y node: "))
        z = int(input("Enter the orientation of robot at goal pt.(in degrees): "))
        #goal_node = (x,y)
        if((x>=0) or (x<=600)) and (y>=0) or (y<=250):
            if (x,y) not in Obs_Coords and ((x + Robot_Radius),y) not in Obs_Coords and ((x-Robot_Radius),y) not in Obs_Coords and (x,(y+Robot_Radius)) not in Obs_Coords  and (x,(y-Robot_Radius)) not in Obs_Coords:
                goal_node=(x,y,z)
                break
            else:
                print("The Entered Goal Node is in obstacle space")
    return goal_node

#zero degrees function for A*
def Robot_0(a):
    pos = a[3]
    newPos = (round((pos[0] + Length_of_stepsize * np.cos((pos[2])*(np.pi/180)))),round((pos[1] + Length_of_stepsize * np.sin(pos[2]*(np.pi/180)))),pos[2]) 
    x,y,_ = newPos
    if (0<=x<600) and (0<=y<250):
        if (CheckedList[y][x] != 1) and ((x,y) not in Obs_Coords):
                CloseList.append(newPos)
                Cost = a[2] + Length_of_stepsize
                Eucledian_dist = np.sqrt(((goal_pt[0] - newPos[0])**2)+((goal_pt[1] - newPos[1])**2))
                TotalCost = Cost + Eucledian_dist
                for m in range(UncheckedList.qsize()):
                    if UncheckedList.queue[m][3] == newPos:
                        if UncheckedList.queue[m][0] > TotalCost:
                            UncheckedList.queue[m] = (TotalCost,Eucledian_dist,Cost,newPos)
                            Pth[newPos] = pos
                            return
                        else:
                            return
                UncheckedList.put((TotalCost,Eucledian_dist,Cost,newPos))
                Pth[newPos] = pos 

#30 degrees function for A*
def Robot_30(a):
    pos = a[3]
    newPos = (round((pos[0] + Length_of_stepsize * np.cos((pos[2]+30) * (np.pi/180)))),round((pos[1] + Length_of_stepsize * np.sin((pos[2]+30) * (np.pi/180)))),pos[2]) 
    x,y,_ = newPos
    if (0<=x<600) and (0<=y<250):
        if (CheckedList[y][x] != 1) and ((x,y) not in Obs_Coords):
                CloseList.append(newPos)
                Cost = a[2] + Length_of_stepsize
                Eucledian_dist = np.sqrt(((goal_pt[0] - newPos[0])**2)+((goal_pt[1] - newPos[1])**2))
                TotalCost = Cost + Eucledian_dist
                for m in range(UncheckedList.qsize()):
                    if UncheckedList.queue[m][3] == newPos:
                        if UncheckedList.queue[m][0] > TotalCost:
                            UncheckedList.queue[m] = (TotalCost,Eucledian_dist,Cost,newPos)
                            Pth[newPos] = pos
                            return
                        else:
                            return
                UncheckedList.put((TotalCost,Eucledian_dist,Cost,newPos))
                Pth[newPos] = pos 

#-30 degree function for A*
def Robot_Inv30(a):
    pos = a[3]
    newPos = (round((pos[0] + Length_of_stepsize * np.cos((pos[2]-30)*(np.pi/180)))),round((pos[1] + Length_of_stepsize * np.sin((pos[2]-30)*(np.pi/180)))),pos[2]) 
    x,y,_ = newPos
    if (0<=x<600) and (0<=y<250):
        if (CheckedList[y][x] != 1) and ((x,y) not in Obs_Coords):
                CloseList.append(newPos)
                Cost = a[2] + Length_of_stepsize
                Eucledian_dist = np.sqrt(((goal_pt[0] - newPos[0])**2)+((goal_pt[1] - newPos[1])**2))
                TotalCost = Cost + Eucledian_dist
                for m in range(UncheckedList.qsize()):
                    if UncheckedList.queue[m][3] == newPos:
                        if UncheckedList.queue[m][0] > TotalCost:
                            UncheckedList.queue[m] = (TotalCost,Eucledian_dist,Cost,newPos)
                            Pth[newPos] = pos
                            return
                        else:
                            return
                UncheckedList.put((TotalCost,Eucledian_dist,Cost,newPos))
                Pth[newPos] = pos 

# 60 degrees function for A*
def Robot_60(a):
    pos = a[3]
    newPos = (round((pos[0] + Length_of_stepsize * np.cos((pos[2]+60) * (np.pi/180)))),round((pos[1] + Length_of_stepsize * np.sin((pos[2]+60) * (np.pi/180)))),pos[2]) 
    x,y,_ = newPos
    if (0<=x<=600) and (0<=y<=250):
        if (CheckedList[y][x] != 1) and ((x,y) not in Obs_Coords):
                CloseList.append(newPos)
                Cost = a[2] + Length_of_stepsize
                Eucledian_dist = np.sqrt(((goal_pt[0] - newPos[0])**2)+((goal_pt[1] - newPos[1])**2))
                TotalCost = Cost + Eucledian_dist
                for m in range(UncheckedList.qsize()):
                    if UncheckedList.queue[m][3] == newPos:
                        if UncheckedList.queue[m][0] > TotalCost:
                            UncheckedList.queue[m] = (TotalCost,Eucledian_dist,Cost,newPos)
                            Pth[newPos] = pos
                            return
                        else:
                            return
                UncheckedList.put((TotalCost,Eucledian_dist,Cost,newPos))
                Pth[newPos] = pos

# -60 degree function for A*
def Robot_Inv60(a):
    pos = a[3]
    newPos = (round((pos[0] + Length_of_stepsize * np.cos((pos[2]-60) * (np.pi/180)))),round((pos[1] + Length_of_stepsize * np.sin((pos[2]-60) * (np.pi/180)))),pos[2]) 
    x,y,_ = newPos
    if (0<=x<600) and (0<=y<250):
        if (CheckedList[y][x] != 1) and ((newPos[0],newPos[1]) not in Obs_Coords):
                CloseList.append(newPos)
                Cost = a[2] + Length_of_stepsize
                Eucledian_dist = np.sqrt(((goal_pt[0] - newPos[0])**2)+((goal_pt[1] - newPos[1])**2))
                TotalCost = Cost + Eucledian_dist
                CheckedList[y][x] = 1
                for m in range(UncheckedList.qsize()):
                    if UncheckedList.queue[m][3] == newPos:
                        if UncheckedList.queue[m][0] > TotalCost:
                            UncheckedList.queue[m] = (TotalCost,Eucledian_dist,Cost,newPos)
                            Pth[newPos] = pos
                            return
                        else:
                            return
                UncheckedList.put((TotalCost,Eucledian_dist,Cost,newPos))
                Pth[newPos] = pos 

#Defining the bactracking algorithm 
def B_tracking(Pth, initial_pt, goal_pt):
    b_track = []
    a = list(Pth)[-1]
    K = Pth.get(a)
    b_track.append(a)
    b_track.append(K)
    
    while K != (initial_pt):  
        K = Pth.get(K)
        
        b_track.append(K)
    b_track.reverse()
    return (b_track)
         
space = np.ones((251,601,3),dtype='uint8')  #Creating an matrix with ones, of the shape of boundry shape
Robot_Radius = int(input("Enter the Radius of the robot: "))
Obstacle_Clearance = int(input("Enter the Obstacle Clearance of the Robot: "))
obstacle_space(space)           #Creating the obstacle boundries


Obs_Coords= resize_obstacle(space)
# for val in Obs_Coords:
#     if val == (101,70):
#         print("the val is present in obstacle", val)

initial_pt = User_Inputs_Start(Obs_Coords)  
goal_pt = User_Inputs_Goal(Obs_Coords)

Length_of_stepsize = int(input("Enter the stepsize of the robot in units bet(1<=stepsize<=10): "))

#print(initial_pt)
#print(goal_pt)
start = (0,0,0,initial_pt)
#print(start)
InitialEucledian_dist = np.sqrt(((goal_pt[0] - start[3][0])**2)+((goal_pt[1] - start[3][1])**2))  
InitialTotalCost = InitialEucledian_dist   
start = (InitialTotalCost,InitialEucledian_dist,0,initial_pt)
#print(start)
UncheckedList.put(start)
#print(UncheckedList)
reached=0
while UncheckedList.qsize() != 0:
    a = UncheckedList.get()
    
    #print(goal_pt)
    CheckedList[a[3][1],a[3][0]] = 1
    if a[3] != goal_pt and a[1] > 1.5:
        if (0<= (a[3][0] + (Length_of_stepsize * np.cos((a[3][2]) * (np.pi/180)))) <= 600) and (0<= (a[3][1] + (Length_of_stepsize * np.sin(30 * (np.pi/180)))) <= 250):
            Robot_0(a)
        if (0<= (a[3][0] + (Length_of_stepsize * np.cos((a[3][2]+30) * (np.pi/180)))) <= 600) and (0<= (a[3][1] + (Length_of_stepsize * np.sin((a[3][2]+30) * (np.pi/180)))) <= 250):
            Robot_30(a)
        if (0<= (a[3][0] + (Length_of_stepsize * np.cos((a[3][2]-30) * (np.pi /180))))<= 600) and (0<=(a[3][1] + (Length_of_stepsize * np.sin((a[3][2]-30) * (np.pi / 180))))<=250):
            Robot_Inv30(a)
        if (0<= (a[3][0] + (Length_of_stepsize * np.cos((a[3][2]+60) * (np.pi /180))))<=600) and (0<=(a[3][1] + (Length_of_stepsize * np.sin((a[3][2]+60) * (np.pi / 180))))<=250):
            Robot_60(a)
        if (0<= (a[3][0] + (Length_of_stepsize * np.cos((a[3][2]-60) * (np.pi /180))))<=600) and (0<=(a[3][1] + (Length_of_stepsize * np.sin((a[3][2]-60) * (np.pi / 180))))<=250):
            Robot_Inv60(a)


    else:
        print("success")
        reached=1
        break
if reached ==1:
    b = B_tracking(Pth, initial_pt, goal_pt)
    print("path")
    print(b)

    # for i in CheckedList:
    #     space[250-i[1]][i[0]] = [255,0,0]

    #     cv.imshow("SPACE", space )
    # # cv.waitKey(0)
    #     if cv.waitKey(1) & 0xFF == ord('q'):
    #         break

    for i in CloseList:
        j = Pth.get(i)
        cv.arrowedLine(space,(i[0],250-i[1]),(j[0],250-j[1]),(0,0,255),1)
        cv.imshow("Space",space)
        if cv.waitKey(5) & 0xFF == ord('q'):
            break


    for j in b:
        space[250-j[1]][j[0]] = [0,255,0]
        cv.circle(space,(j[0],250-j[1]), Robot_Radius, (0,255,0), -1)
        cv.imshow("SPACE", space )
        if cv.waitKey(50) & 0xFF == ord('q'):
            break
else:
    print("The Gaol node cannot be reached")

cv.destroyAllWindows()
