print ("hellow orld")


sensorArr = [0,0,0,0]
#sensorArr 1 when on starting from the left most sensor and going to the right most 
rollList = ["","","","","","",""]
#len(set(my_list[-5:])) == 1



#[A1,A2,A3,A4,A5,A6,LB1Turn,LB2,Start11,LB3,LB4Turn,B1,B2,B3,B4,B5,B6]
map = ["A1", "A2", "A3", "A4", "A5", "A6", "LB1T", "LB2", "S8", "LB3", "LB4T", "B1", "B2", "B3", "B4", "B5", "B6"]
#LB is loading bay, LB1T is Loading bay and turn S8 is Start point

 
turns = [1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1]
#labels turns assuming anti clockwise movement 1=L 0=R flip for clockwise movement

pointer = 8
dest = 9
dir = 1
pathViewToggle = True

#dir=1 for anticlockwise movement dir=-1 for clockwise movement 

def decide_movement(sensor_input):
    # S:Straight
    # For all cases read multiple times then wait/slow down heavily to ensure sensor reading correct
    match sensor_input:
        case [1, 1, 1, 1]:
            # Go straight - cooked should never be seen ever
            return "S"
        case [1, 0, 0, 1]:
            # Left or right
            return "LR"
        case [1, 1, 1, 0]:
            # Straight or left
            return "SL"
        case [0, 1, 1, 1]:
            # Straight or right
            return "SR"
        case [0, 1, 1, 0]:
            # Continue forwards NO DELAY
            return "S"
        case [0, 0, 1, 0]:
            # Slight right adjustment
            return "AR"
        case [0, 1, 0, 0]:
            # Slight left adjustment
            return "AL"
        case [1, 0, 0, 0]:
            # Left turn
            return "L"
        case [0, 0, 0, 1]:
            # Right turn
            return "R"
        case [0, 0, 0, 0]:
            # Return help party
            return "party"
        case [1, 1, 0, 0]:
            return "AL"
        case [0, 0, 1, 1]:
            return "AR"
        case [1, 0, 1, 0]:
            return "AR"
        case [0, 1, 0, 1]:
            return "AL"
        case [1, 0, 1, 1]:
            return "AR"
        case [1, 1, 0, 1]:
            return "AL"
        case _:
            return "party"

    #Will use rolling list to ensure that same result has been polled 4 times



nav():
    #input pointer des rollList 
    #tools the match cases
    #output to be turn functions
    #we are going straight right now
    all_equal = len(set(rollList[-5:])) == 1
    if all_equal and rollList[-1] != "S" and pathViewToggle:
        # stop registering new/different path and only register as passing this path while encountering non Straight region
        pointer += dir
        pathViewToggle = False
    elif all_equal and rollList[-1] == "S":
        # allow new path to be registered on nav system if has been seeing straight line
        pathViewToggle = True
    if pointer == dest:
        deploymentFunc(dir,map[pointer])
        # This deployment function is a turn left or turn right or go straight to start going down offshoot
        # deployment function will include dropping or picking up box and return to original position on map
        # to be done:  in ending the deployment function the cargobot should be oriented correctly either clockwise or anticlockwise on the track depending on where to go next
        
        
    


start()
#anticlockwise on track at S8
            
rollListUpdate():
    input = senseTurn(inputArr)
    rollList.append(input)
    rollList.pop(0)

loop():
    

   power set on motor straight
