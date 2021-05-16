import networkx as nx
import numpy as np
from scipy.spatial import distance
import math
from time import sleep
from time import time

class PathPlanning:

    """
    This class calculates the path that our vehicle is going to follow, given a start and a finish node.

    The path planning is implemented using A* algorithm, utilizing the BFMC map graph.
    """

    #map_path = "/home/papafotit/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src/map.graphml"
    map_path = "src/startup_package/src/map.graphml"
    intersection_nodes = ['346', '270', '32', '36', '34', '6', '25', '27', '23', '370', '81', '2', '15', '16', '77', '72', '70', '79', '311', '314', '61', '467', '4', '59', '54', '52', '63', '68', '423', '303', '305', '45', '43', '41']
    central_nodes = ['347', '271', '37', '39', '38', '11', '29', '30', '28', '371', '84', '9', '20', '20', '82', '75', '74', '83', '312', '315', '65', '468', '10', '64', '57', '56', '66', '73', '424', '304', '306', '48', '47', '46']
    
    ra_enter = ["230", "342", "301"]
    ra_exit = ["231", "272", "343"]    

    turning_left = False
    turn_angle = 0
    G = nx.read_graphml(map_path)

    @staticmethod
    def shortest_path(source, target):
        return list(nx.all_shortest_paths(PathPlanning.G, source=source, target=target))[0]

    @staticmethod
    def find_closest_node(x, y):
        distances_dict = {"Nodes": [], "Distances":[]}
        for node in PathPlanning.G.nodes():
            node_x = PathPlanning.G.nodes[node]['x']
            node_y = PathPlanning.G.nodes[node]['y']

            distance_from_node = distance.euclidean((x,y), (node_x,node_y))
            
            distances_dict["Nodes"].append(node)
            distances_dict["Distances"].append(distance_from_node)

        
        min_distance = min(distances_dict["Distances"])
        min_index  = distances_dict["Distances"].index(min_distance)
        min_node = distances_dict["Nodes"][min_index]
  
        return min_node

    @staticmethod
    def remove_central_nodes(path):
        new_path = path[:]
        for node in PathPlanning.central_nodes:
            if node in new_path:
                new_path.remove(node)
        return new_path

    @staticmethod
    def find_inter_nodes():
        inter_nodes = []
        for node in PathPlanning.G.nodes:
            neighbors_list = list(PathPlanning.G.neighbors(node))
            if len(neighbors_list) > 0 and len(list(PathPlanning.G.neighbors(neighbors_list[0]))) > 1:
                inter_nodes.append(node)
        #print(inter_nodes)

    @staticmethod 
    def find_roundabout_entry(path):
        for i in path:
            if i in PathPlanning.ra_enter:
                return i
        return None
        

    @staticmethod 
    def find_roundabout_exit(path):
        for i in path:
            if i in PathPlanning.ra_exit:
                return i
        return None
        

    @staticmethod
    def find_central_nodes():
        inter_nodes = []
        for node in PathPlanning.G.nodes:
            neighbors_list = list(PathPlanning.G.neighbors(node))
            if len(neighbors_list) > 0 and len(list(PathPlanning.G.neighbors(neighbors_list[0]))) > 1:
                inter_nodes.append(neighbors_list[0])
        #print(inter_nodes)

    @staticmethod
    def check_intersection(path):

        for i in range(3):
            if path[i] in PathPlanning.intersection_nodes:
                return True

        return False

    @staticmethod
    def check_roundabout(path):

        for i in range(3):
            if path[i] in PathPlanning.ra_enter:
                return True

        return False

    @staticmethod
    def find_target(path):
        for i in range(3):
            #print("Node in find target: " + str(path[i]))
            if path[i] in PathPlanning.intersection_nodes:
                
                if i+1 >= len(path):
                    return None, True
                    break

                target_node = path[i+1] 

                return target_node, False
        #if path[0] or path[1] in PathPlanning.intersection_nodes:
        #    return True, path[1]
        return None, False

    @staticmethod
    def update_path(path, x, y, finish):
        closest_node = PathPlanning.find_closest_node(x,y)
        #print("CLOSEST NODE: " + str(closest_node))
        path_updated = False
        if closest_node == finish:
            return path, True
        if(closest_node in path):
            path = path[path.index(closest_node):]
            path_updated = True
        
        if path_updated == False:
            closest_node = PathPlanning.find_closest_node_from_path(x, y, path)
            #print("Closest node in path: ", str(closest_node))
            if abs(PathPlanning.G.nodes[closest_node]['x'] - x) < 0.3 and  abs(PathPlanning.G.nodes[closest_node]['y'] - y) < 0.3:
                path = path[path.index(closest_node):]

        return path, False


    @staticmethod
    def intersection_navigation(path, x, y, target_node, start_yaw, yaw, complete_path, steer_factor, speed, start_time):
        #Ignore next node as it is an central intersection node
        
        reached_target = False
        steering_angle = 0

        right_turn_base = 1
        right_turn_factor = 20 * speed
        max_right_turn = 20
        right_turn_factor_big = 12 * speed
        right_turn_base_big = 0

        left_turn_base = -2
        left_turn_factor = - 10 * speed
        left_turn_factor_small = - 22 * speed
        max_left_turn = -15
        max_left_turn_small = -20

        turn_type = 0
        closest_node = PathPlanning.find_closest_node(x,y)
        #Get next node after target node
        #next_node_index = complete_path.index(target_node) + 1
        #if next_node_index < len(complete_path):
        #    next_node = complete_path[next_node_index]

        if target_node is not None:
            next_node = list(PathPlanning.G[str(target_node)])[0]
            #print(next_node)
        else:
            return steering_angle, True

        
        try:
            if closest_node in complete_path and next_node in complete_path:
                closest_node_index = complete_path.index(closest_node)
                
                next_node_index = complete_path.index(next_node)
                if closest_node_index >= next_node_index:
                    #print("GOT OUT ", closest_node)

                    return steering_angle, True
        except:
            #print("CANT FIND INDEX", closest_node, " " ,next_node)
            return 0, True
        
        cd_x = PathPlanning.G.nodes[closest_node]['x']
        cd_y = PathPlanning.G.nodes[closest_node]['y']
        
        inter_node = complete_path[complete_path.index(target_node) - 2]
        inter_node_x = PathPlanning.G.nodes[inter_node]['x']
        inter_node_y = PathPlanning.G.nodes[inter_node]['y']

        node_x = PathPlanning.G.nodes[target_node]['x']
        node_y = PathPlanning.G.nodes[target_node]['y']

        '''
        print("Node: ", target_node)
        print("Closest Node:", closest_node)
        print("Node_x: " + str(node_x) + " CX: " + str(cd_x))
        print("Node_y: " + str(node_y) + " CY: " + str(cd_y))
        print("My coords ", str(x), " , ", str(y) )
        '''
        time_factor = time() - start_time
        #Upwards
        if( 80 <= start_yaw <= 100 ):#(node_x > cd_x and node_y < cd_y) or (node_x == cd_x and node_y < cd_y) or (node_x < cd_x and node_y < cd_y)):
            central_node = complete_path[complete_path.index(target_node) - 1]
            central_node_x = PathPlanning.G.nodes[central_node]['x']
            central_node_y = PathPlanning.G.nodes[central_node]['y']
            
            distance_from_target = PathPlanning.distance_from_node(node_x, node_y, x, y)

            #LEFT
            if (0.3 < inter_node_x - node_x and 0.3 < inter_node_y - node_y): #and distance_from_target > 1.1: #not PathPlanning.in_range(central_node_x, central_node_y, x, y)):
                #print("Left Turn")
                turn_type = -1

                dy = abs(node_y - inter_node_y)

                #Small Left Turn
                if dy < 0.8:
                    steering_angle = max(left_turn_base + left_turn_factor_small * time_factor, max_left_turn_small)
                #Big Left Turn
                else:
                    steering_angle = max(left_turn_base + left_turn_factor * time_factor, max_left_turn)
                
                if 170 < yaw < 180 or -180 < yaw < -170:
                    steering_angle = 0
                    reached_target = True
                
                #print(steering_angle)

            #RIGHT
            elif (node_x - inter_node_x > 0.3 and 0.3 < inter_node_y - node_y):
                
                turn_type = 1

                dy = abs(node_y - inter_node_y)

                if dy < 0.8:
                    steering_angle = min(right_turn_base+2 + right_turn_factor * time_factor, max_right_turn)
                else:
                    steering_angle = min(right_turn_base_big + 0.45 * right_turn_factor * time_factor, max_right_turn)

                if 0 <= yaw <= 10:
                    steering_angle = 0
                    reached_target = True

            #STRAIGHT
            else:
                #print("Straight")

                turn_type = 0

                #angle_rad = math.atan(float(node_x - x) / float(y - node_y))
                #steering_angle = int(angle_rad * 180.0 / math.pi)
                steering_angle = (yaw - 90)
            #print("Upwards")
        
        #Left-to-right
        elif(-10 <= start_yaw <= 10): #(node_x > cd_x and node_y > cd_y) or (node_x > cd_x and node_y == cd_y) or (node_x > cd_x and node_y < cd_y)):
            central_node = complete_path[complete_path.index(target_node) - 1]
            central_node_x = PathPlanning.G.nodes[central_node]['x']
            central_node_y = PathPlanning.G.nodes[central_node]['y']
            
            distance_from_central =  PathPlanning.distance_from_node(central_node_x, central_node_y, x, y)
            distance_from_target = PathPlanning.distance_from_node(node_x, node_y, x, y)

            #print("distance from target: " + str(distance_from_target))

            if (node_x - inter_node_x > 0.3 and 0.3 < inter_node_y - node_y): 
                #print("YAW IN THIS THING " + str(yaw))

                turn_type = -1

                dx = abs(node_x - inter_node_x)

                #Small Left Turn
                if dx < 0.8:
                    steering_angle = max(left_turn_base + left_turn_factor_small * time_factor, max_left_turn_small)
                #Big Left Turn
                else:
                    steering_angle = max(left_turn_base + left_turn_factor * time_factor, max_left_turn)


                #node_x = central_node_x
                #node_y = central_node_y

                #angle_rad = -math.atan(float(y - node_y) / float(node_x - x)) 
                #steering_angle = int(angle_rad * 180.0 / math.pi) 

                if 80 < yaw < 100:
                    #print("YAW AT STOP: ", yaw)
                    reached_target = True
                    steering_angle = 0
 

            elif node_x - inter_node_x > 0.3 and node_y - inter_node_y > 0.3:

                turn_type = 1
                
                dx = abs(node_x - inter_node_x)

                if dx < 0.8:
                        steering_angle = min(right_turn_base + right_turn_factor * time_factor, max_right_turn)
                else:
                    steering_angle = min(right_turn_base_big + 0.45 * right_turn_factor * time_factor, max_right_turn)
                
                if -100 < yaw < -80:
                    reached_target = True
                    steering_angle = 0

            else:
                steering_angle = yaw

                turn_type = 0

            #print("Left-to-right ", steering_angle)
       
        #Downwards
        elif(-100 <= start_yaw <= -80):#(node_x < cd_x and node_y > cd_y) or (node_x == cd_x and node_y > cd_y) or (node_x > cd_x and node_y > cd_y)):
            central_node = complete_path[complete_path.index(target_node) - 1]
            central_node_x = PathPlanning.G.nodes[central_node]['x']
            central_node_y = PathPlanning.G.nodes[central_node]['y']
            
            distance_from_target = PathPlanning.distance_from_node(node_x, node_y, x, y)

            if (node_x - inter_node_x > 0.3 and node_y - inter_node_y > 0.3):# and distance_from_target > 1.1:

                #print("Left Turn")

                turn_type = -1

                dy = abs(node_y - inter_node_y)

                #Small Left Turn
                if dy < 0.8:
                    steering_angle = max(left_turn_base + left_turn_factor_small * time_factor, max_left_turn_small)

                #Big Left Turn
                else:
                    steering_angle = max(left_turn_base + left_turn_factor * time_factor, max_left_turn)

                if -10 < yaw < 10:
                    steering_angle = 0
                    reached_target = True

            elif 0.3 < inter_node_x - node_x and node_y - inter_node_y > 0.3:
                steering_angle = min(right_turn_base + right_turn_factor * time_factor, max_right_turn)
                
                turn_type = 1

                dy = abs(node_y - inter_node_y)

                if dy < 0.8:
                        steering_angle = min(right_turn_base + right_turn_factor * time_factor, max_right_turn)
                else:
                    steering_angle = min(right_turn_base_big + 0.45 * right_turn_factor * time_factor, max_right_turn)

                if 170 < yaw < 180 or -180 < yaw < -170:
                    steering_angle = 0
                    reached_target = True
            else:

                turn_type = 0
                #angle_rad = -math.atan(float(node_x - x) / float(y - node_y))
                #steering_angle = int(angle_rad * 180.0 / math.pi)
                steering_angle = (yaw + 90)
            #print("Downwards")

        #Right-to-left
        elif(-180 <= start_yaw <= -170 or 170 <= start_yaw <= 180):#(node_x < cd_x and node_y < cd_y) or (node_x < cd_x and node_y == cd_y) or (node_x < cd_x and node_y > cd_y)):

            central_node = complete_path[complete_path.index(target_node) - 1]
            central_node_x = PathPlanning.G.nodes[central_node]['x']
            central_node_y = PathPlanning.G.nodes[central_node]['y']

            distance_from_target = PathPlanning.distance_from_node(node_x, node_y, x, y)

            if (0.3 < inter_node_x - node_x and node_y - inter_node_y > 0.3) and distance_from_target > 1.1:

                turn_type = -1

                dx = abs(node_x - inter_node_x)

                #Small Left Turn
                if dx < 0.8:
                    steering_angle = max(left_turn_base + left_turn_factor_small * time_factor, max_left_turn_small)

                #Big Left Turn
                else:
                    steering_angle = max(left_turn_base + left_turn_factor * time_factor, max_left_turn)

                if -100 < yaw < -80:
                    steering_angle = 0
                    reached_target = True
            
            elif 0.3 < inter_node_x - node_x and 0.3 < inter_node_y - node_y:

                turn_type = 1

                dx = abs(node_x - inter_node_x)
                if dx < 0.8:
                    steering_angle = min(right_turn_base + right_turn_factor * time_factor, max_right_turn)
                else:
                    steering_angle = min(right_turn_base_big +  right_turn_factor_big * time_factor, max_right_turn)
                    #print("Angle", steering_angle)
                if 80 < yaw < 100:
                    steering_angle = 0
                    reached_target = True
            else:

                turn_type = 0

                if yaw > 0:
                    steering_angle = yaw - 180
                else:
                    steering_angle = yaw + 180
            #print("Right-to-left")
        
        #print("Target is: " + str(target_node), " Closest Node: ", str(closest_node))
        #print("And turn_type is: " + str(turn_type))

        return steering_angle, reached_target
   
    @staticmethod
    def distance_from_node(node_x, node_y, x, y):

        distance_node = distance.euclidean((x,y), (node_x,node_y))

        return distance_node

    @staticmethod
    def in_range(node_x,node_y, x, y):
        threshold = 0.1
        if( node_x - threshold <= x < node_x + threshold and node_y - threshold <= y <= node_y + threshold):
            #print(node_x - threshold, node_x, node_x + threshold)
            return True
        else:
            return False

    @staticmethod
    def in_vertical_range(node_y, y):
        threshold = 0.2
        if(node_y - threshold <= y <= node_y + threshold):
            return True
        else:
            False

    
    @staticmethod
    def find_closest_node_from_path(x, y, path):
        distances_dict = {"Nodes": [], "Distances":[]}
        for node in path:
            node_x = PathPlanning.G.nodes[node]['x']
            node_y = PathPlanning.G.nodes[node]['y']

            distance_from_node = distance.euclidean((x,y), (node_x,node_y))
            
            distances_dict["Nodes"].append(node)
            distances_dict["Distances"].append(distance_from_node)

        
        min_distance = min(distances_dict["Distances"])
        min_index  = distances_dict["Distances"].index(min_distance)
        min_node = distances_dict["Nodes"][min_index]
  
        return min_node