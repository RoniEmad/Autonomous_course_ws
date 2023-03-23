#!/usr/bin/env python

import numpy as np #Imports Numpy package full of functions and methods that can be used.
import rospy
import cv2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def Draw_Horizontal(Array,Index1,Index2):   #Draws A horizontal Line from index1 to index2
    Local_Counter = Index1[1]
    while(Local_Counter <= Index2[1]):
        Array[Index1[0],Local_Counter]= 0
        Local_Counter+=1
        
def Draw_Vertical(Array,Index1,Index2):    #Draws A Vertical Line from index1 to index2
    Local_Counter = Index1[0]
    while(Local_Counter <= Index2[0]):
        Array[Local_Counter,Index1[1]]= 0
        Local_Counter+=1

def Draw(Map):    #Draws a Map

    Draw_Horizontal(Map,[0,0],[0,6])
    Draw_Horizontal(Map,[6,0],[6,6])
    Draw_Vertical(Map,[0,0],[6,0])        # EDGES
    Draw_Vertical(Map,[0,6],[6,6])


def BFS(Map,Start,Goal,Direction):  # Direction for CCW -1  for CW 1
    global h,w
    global Visited
    Flag = False # Flag indicating if goal node is found
    
    # Initializing a queue 
    queue = []
    queue.append(Start)

    # Visited Nodes
    Visited = []

    # Parent Node Mapping
    Parents = np.zeros([h,w,2],dtype = int)


    while queue:   # If queue is not empty 

        Node = queue.pop(0)  # Pop first element in the queue

        if Node==Goal:   # Did we reach the goal?
            Flag = True
            break

        Neighbours = get_Neighbours(Node,Direction) # Get neighbours of this node

        for N in Neighbours: # For each neighbour of this node
            if not N in Visited and Map[N[0],N[1]]!=0:   # if this neighbour wasnt visited before and doesnt equal 0 (ie not an obstacle)
                queue.append(N)      # add to the queue
                Visited.append(N)     # add it to the visited
                Parents[N[0],N[1]] = Node  # add its parent node 

    if Flag == False:
        print('Mafesh path :(')

    if Flag == True:
        path = get_Path(Parents,Goal,Start) # this function returns the path to the goal 
        return path


def DFS(Map,Start,Goal,Direction):  # Direction for CCW -1  for CW 1
    global h,w
    global Visited
    Flag = False # Flag indicating if goal node is found
    
    # Initializing a stack 
    stack = []
    stack.append(Start)

    # Visited Nodes
    Visited = []

    # Parent Node Mapping
    Parents = np.zeros([h,w,2],dtype = int)


    while stack:   # If stack is not empty 

        Node = stack.pop(-1)  # Pop last element in the stack

        if Node==Goal:   # Did we reach the goal?
            Flag = True
            break

        Neighbours = get_Neighbours(Node,Direction) # Get neighbours of this node

        for N in Neighbours: # For each neighbour of this node
            if not N in Visited and Map[N[0],N[1]]!=0:   # if this neighbour wasnt visited before and doesnt equal 0 (ie not an obstacle)
                stack.append(N)      # add to the queue
                Visited.append(N)     # add it to the visited
                Parents[N[0],N[1]] = Node  # add its parent node 

    if Flag == False:
        print('Mafesh path :(')

    if Flag == True:
        path = get_Path(Parents,Goal,Start) # this function returns the path to the goal 
        return path


        
    

def get_Path(Parents,Goal,Start):

    path = []
    Node = Goal  # start backward 
    path.append(Goal)
   
    while not np.array_equal(Node, Start):

       path.append(Parents[Node[0],Node[1]])  # add the parent node 
       Node = Parents[Node[0],Node[1]]        # search for the parent of that node 
     
    path.reverse()
    return path

    

def get_Neighbours(Node,Direction):    # Returns neighbours of the parent node   Direction = 1 for Clockwise/ Direction = -1 for CCW 

    Neighbours = []
    
    if Direction==1:
        Neighbours.append([Node[0],Node[1]+1])
        Neighbours.append([Node[0]+1,Node[1]+1])
        Neighbours.append([Node[0]+1,Node[1]])
        Neighbours.append([Node[0]+1,Node[1]-1])
        Neighbours.append([Node[0],Node[1]-1])
        Neighbours.append([Node[0]-1,Node[1]-1])
        Neighbours.append([Node[0]-1,Node[1]])
        Neighbours.append([Node[0]-1,Node[1]+1])
                        
    if Direction==-1:
        Neighbours.append([Node[0],Node[1]+1])
        Neighbours.append([Node[0]-1,Node[1]+1])
        Neighbours.append([Node[0]-1,Node[1]])
        Neighbours.append([Node[0]-1,Node[1]-1])
        Neighbours.append([Node[0],Node[1]-1])
        Neighbours.append([Node[0]+1,Node[1]-1])
        Neighbours.append([Node[0]+1,Node[1]])
        Neighbours.append([Node[0]+1,Node[1]+1])

    return Neighbours

if __name__ == '__main__':     # Main function that is executed

    rospy.init_node('BFS_node')
    path_pub = rospy.Publisher('/path_BFS', Path, queue_size=10)
    rate1 = rospy.Rate(1)
    global h,w
    global Visited
    #img=cv2.imread("/home/ronidodo/autonomous_ws/src/milestone3/scripts/map_grid01.png",0)
    #img255=cv2.imread("/home/ronidodo/autonomous_ws/src/milestone3/scripts/map_grid0255.png",0)
    img=cv2.imread("/home/ronidodo/autonomous_ws/src/milestone3/scripts/map_grideroded01.png",0)
    img255=cv2.imread("/home/ronidodo/autonomous_ws/src/milestone3/scripts/map_grideroded0255.png",0)
    print(img)
    print(img.shape)
    h,w=img.shape
    cv2.imshow("img",img)
    cv2.imshow("img255",img255)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    Map=np.array(img)
    print("map",Map.shape)
    start_real=np.array([1,1]) #Start point in meters
    goal_real=np.array([4.5,-4.5]) #Goal point in meters
    print(start_real)
    print(goal_real)
    start_=np.round_((start_real+5)*h/10).astype(int).tolist()
    goal_=np.round_((goal_real+5)*h/10).astype(int).tolist()
    print(start_)
    print(goal_)
    path_bfsn1 = BFS(Map,start_,goal_,-1)
    img_bfsn1=img255.copy()
    for x_ in Visited:
        img_bfsn1[x_[0],x_[1]]=50
        cv2.imshow("Path_bfsn1",img_bfsn1)
        cv2.waitKey(1)
    if not path_bfsn1==None:
        for point in path_bfsn1:
            print(point)
            img_bfsn1[point[0],point[1]]=128
            cv2.imshow("Path_bfsn1",img_bfsn1)
            cv2.waitKey(1)
        #print(path_bfsn1)
        #print(Map)
        real_path_bfsn1=np.array(path_bfsn1)
        #print(real_path_bfsn1)
        real_path_bfsn1=10/h*real_path_bfsn1-5
        real_path_bfsn1=np.append(real_path_bfsn1,[goal_real],axis=0)
        #print(real_path_bfsn1)
    img_bfsn1[start_[0],start_[1]]=200
    img_bfsn1[goal_[0],goal_[1]]=200
    cv2.imshow("Path_bfsn1",img_bfsn1)
    cv2.waitKey(0)
    #cv2.destroyAllWindows()
    cv2.imwrite("/home/ronidodo/autonomous_ws/src/milestone3/scripts/map_pathbfsn1.png",img_bfsn1)
    path_dfsn1 = DFS(Map,start_,goal_,-1)
    img_dfsn1=img255.copy()
    for x_ in Visited:
        img_dfsn1[x_[0],x_[1]]=50
        cv2.imshow("Path_dfsn1",img_dfsn1)
        cv2.waitKey(1)
    if not path_dfsn1==None:
        for point in path_dfsn1:
            #print(point)
            img_dfsn1[point[0],point[1]]=128
            cv2.imshow("Path_dfsn1",img_dfsn1)
            cv2.waitKey(1)
        #print(path_dfsn1)
        #print(Map)
        real_path_dfsn1=np.array(path_dfsn1)
        #print(real_path_dfsn1)
        real_path_dfsn1=10/h*real_path_dfsn1-5
        real_path_dfsn1=np.append(real_path_dfsn1,[goal_real],axis=0)
        #print(real_path_dfsn1)
    img_dfsn1[start_[0],start_[1]]=200
    img_dfsn1[goal_[0],goal_[1]]=200
    cv2.imshow("Path_dfsn1",img_dfsn1)
    cv2.waitKey(0)
    #cv2.destroyAllWindows()
    cv2.imwrite("/home/ronidodo/autonomous_ws/src/milestone3/scripts/map_pathdfsn1.png",img_dfsn1)
    path_bfs1 = BFS(Map,start_,goal_,1)
    img_bfs1=img255.copy()
    for x_ in Visited:
        img_bfs1[x_[0],x_[1]]=50
        cv2.imshow("Path_bfs1",img_bfs1)
        cv2.waitKey(1)
    if not path_bfs1==None:
        for point in path_bfs1:
            print(point)
            img_bfs1[point[0],point[1]]=128
            cv2.imshow("Path_bfs1",img_bfs1)
            cv2.waitKey(1)
        #print(path_bfs1)
        #print(Map)
        real_path_bfs1=np.array(path_bfs1)
        #print(real_path_bfs1)
        real_path_bfs1=10/h*real_path_bfs1-5
        real_path_bfs1=np.append(real_path_bfs1,[goal_real],axis=0)
        #print(real_path_bfs1)
    img_bfs1[start_[0],start_[1]]=200
    img_bfs1[goal_[0],goal_[1]]=200
    cv2.imshow("Path_bfs1",img_bfs1)
    cv2.waitKey(0)
    #cv2.destroyAllWindows()
    cv2.imwrite("/home/ronidodo/autonomous_ws/src/milestone3/scripts/map_pathbfs1.png",img_bfs1)
    path_dfs1 = DFS(Map,start_,goal_,1)
    img_dfs1=img255.copy()
    for x_ in Visited:
        img_dfs1[x_[0],x_[1]]=50
        cv2.imshow("Path_dfs1",img_dfs1)
        cv2.waitKey(1)
    if not path_dfs1==None:
        for point in path_dfs1:
            #print(point)
            img_dfs1[point[0],point[1]]=128
            cv2.imshow("Path_dfs1",img_dfs1)
            cv2.waitKey(1)
        #print(path_dfs1)
        #print(Map)
        real_path_dfs1=np.array(path_dfs1)
        #print(real_path_dfs1)
        real_path_dfs1=10/h*real_path_dfs1-5
        real_path_dfs1=np.append(real_path_dfs1,[goal_real],axis=0)
        #print(real_path_dfs1)
    img_dfs1[start_[0],start_[1]]=200
    img_dfs1[goal_[0],goal_[1]]=200
    cv2.imshow("Path_dfs1",img_dfs1)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite("/home/ronidodo/autonomous_ws/src/milestone3/scripts/map_pathdfs1.png",img_dfs1)

    #choose the path planning algorithm to be published
    real_path=real_path_bfs1.copy()


    path_msg=Path()
    path_msg.header.frame_id = "bfs"
    path_msg.header.stamp = rospy.Time.now()
    for i in range(len(real_path)-1):
        real_point=real_path[i]
        real_point_next=real_path[i+1]
        print("real_point",real_point,real_point_next)
        delta=real_point_next-real_point
        print("delta",delta)
        pose = PoseStamped()
        pose.header.frame_id = "bfs"
        pose.header.seq = path_msg.header.seq + 1
        pose.header.stamp = path_msg.header.stamp
        pose.pose.position.x = real_point[0]
        pose.pose.position.y = real_point[1]
        pose.pose.position.z = np.arctan2(delta[1],delta[0]) * 180 / np.pi
        print("angle",pose.pose.position.z)
        path_msg.poses.append(pose)
        if i ==len(real_path)-2:
            pose.pose.position.x = real_point_next[0]
            pose.pose.position.y = real_point_next[1]
            path_msg.poses.append(pose)
    
    print(path_msg)
    while not rospy.is_shutdown():
        rospy.loginfo("publishing")
        path_pub.publish(path_msg)
        rate1.sleep()


