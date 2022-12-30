#!/usr/bin/python
# coding: utf-8

"""
#############################################################################################################
# Andrea Favero 30 Dec 2022
# 
# From Kociemba solver to robot moves
# This applies to "CUBOTino" autonomous, a simpler Rubik's cube solver robot than my first one:
# (https://www.youtube.com/watch?v=oYRXe4NyJqs)
#
# The Kociemba cube solver returns a solution that has to be translated into robot movements
# Notations for faces are as per URF notations: U=Up, R=Right, F=Front, D=Down, L=Left, B=Back
# Tipically the solver considers the cube orientation to don't change while solving it....
# On this simpler robot, the mechanical constraints (servo rotation limited to 180deg), suggests a different approach:
#  - The face to be turned, according to the solver, will be adapted to reflect the side such face is located
#  - This means a supposed R1 might change L1 if the left cube side is located on the right side at that moment in time
# Amount of robot movements increases by about 85% because of these mechanical contraints
#
#
# Possible moves with this robot
# 1) Spins the complete cube ("S") laying on the bottom face: 1 means CW 90deg turns, while 3 means 90CCW turn
# 2) Flips the complete cube ("F") by "moving" the Front face to Bottom face: Only positive values are possible
# 3) Rotates the bottom layer ("R") while costraining the 2nd and 3rd layer.
# 4) The order of S, F has to be strictly followed
# 5) Example 'F1R1S3' means: 1x cube Flip, 1x (90deg) CW rotation of the 1st (Down) layer, 1x (90deg) CCW cube Spin 
#
#############################################################################################################
"""


"""
Cube orientation at the start, later updated after every cube movement on the robot
Dict key is the the "stationary" side, while the dict value is the cube side

From 23/11/2022 the cube is NOT moved to initial position after scanning the cube status: Out of
ca 1300 cube solving cycles (3 robots, random scrambled, removed dublicated status, etc),
62% of the times the first move is one of the URF sides, where U leads with 23% of the total.
After scanning the 6th cube face, the U face is perfectly on the bottom, so better to start from there.

     
v_faces{}   _______       
           |       |
           | ['U'] |
           |_______|     h_faces{}  _______ _______ _______ 
           |       |               |       |       |       |
           | ['F'] |               | ['L'] | ['F'] | ['R'] |
           |_______|               |_______|_______|_______|
           |       |
           | ['D'] |
           |_______|    

by knowing 5 faces, the 6th (B face) is also known ;-)
""" 


# Global variable
# Below dict has all the possible robot movements, related to the cube solver string
moves_dict = {'U1':'F2R1S3', 'U2':'F2R1S3R1S3', 'U3':'F2S1R3',
              'D1':'R1S3',   'D2':'R1S3R1S3',   'D3':'S1R3',
              'F1':'F1R1S3', 'F2':'F1R1S3R1S3', 'F3':'F1S1R3',
              'B1':'F3R1S3', 'B2':'F3R1S3R1S3', 'B3':'F3S1R3',
              'L1':'S3F3R1', 'L2':'S3F3R1S3R1', 'L3':'S1F1R3',
              'R1':'S3F1R1', 'R2':'S3F1R1S3R1', 'R3':'S1F3R3'}






def starting_cube_orientation():
    """ Defines the starting cube orientation, that has to be recalled in case the robot is operated multiple times in a single session."""
    
    global h_faces,v_faces 
    
    # Cube orientation at the start, later updated after every cube movement on the robot
    h_faces={'L':'F','F':'U','R':'B'}   # dict with faces around the bottom/upper positioned faces
    v_faces={'D':'R','F':'U','U':'L'}   # dict with faces around the left/right positioned faces






def opp_face(face):
    """ This function returns the opposite face of the one in argument."""
    
    if face == 'F': return 'B'
    elif face == 'B': return 'F'
    elif face == 'U': return 'D'
    elif face == 'D': return 'U'
    elif face == 'R': return 'L'
    elif face == 'L': return 'R'
    else:
        return 'Error'






def flip_effect(h_faces,v_faces):
    """ Returns the cube faces orientation after a single Flip action; Only v_faces are affected
        It applies a face shift of these faces, and updates the F face on the h_faces dict."""
    
    v_faces['D']=v_faces['F']
    v_faces['F']=v_faces['U']
    v_faces['U']=opp_face(v_faces['D'])
    h_faces['F']=v_faces['F']






def spinCCW_effect(h_faces,v_faces):
    """ Returns the cube faces orientation after a single CCW spin action; Only h_faces are affected
        It applies a face shiftof these faces, and updates the F face on the v_faces dict."""
    
    h_faces['L']=h_faces['F']
    h_faces['F']=h_faces['R']
    h_faces['R']=opp_face(h_faces['L'])
    v_faces['F']=h_faces['F']
       





def spinCW_effect(h_faces,v_faces):
    """ Returns the cube faces orientation after a single CW spin action; Only h_faces are affected
        It applies a face shiftof these faces, and updates the F face on the v_faces dict."""
    
    h_faces['R']=h_faces['F']
    h_faces['F']=h_faces['L']
    h_faces['L']=opp_face(h_faces['R'])
    v_faces['F']=h_faces['F']






def cube_orient_update(movement):
    """ This function traks the cube orientation based on the applied movements by the robot.
        Arguments is the applied robot movement.
        The function uses the cube orientation global variables  "h_faces" and "v_faces"."""
        
    global h_faces,v_faces
    
    for i in range(len(movement)):                 # iterates over the string of robot movements
        if movement[i] == 'F':                     # case there is a cube flip on robot movements
            repeats=int(movement[i+1])             # retrieves how many flips
            for j in range(repeats):               # iterates over the amount of flip
                flip_effect(h_faces,v_faces)       # re-order the cube orientation on the robot due to the flip
        
        elif movement[i] == 'S':                   # case there is a cube spin on robot movements
            repeats=int(movement[i+1])             # retrieves how many spin
            if repeats=='3':                       # case the spin is CCW
                spinCCW_effect(h_faces,v_faces)    # re-order the cube orientation on the robot due to the CCW spin
            else:                                  # case the spin is CW
                for j in range(repeats):           # iterates over the amount of spin
                    spinCW_effect(h_faces,v_faces) # re-order the cube orientation on the robot due to the CW spin             






def adapt_move(move):
    """ This function adapts the robot move after verifying on wich side the related face is located.
        The solver considers the cube orientation to don't change, but on the robot it does.
        This function will then swap the face name, instead to move the cube back on the original position
        The function returns a dict with all the robot moves and the total amount."""
    
    global h_faces,v_faces
    
    face_to_turn = move[0]                        # face to be turned according to the solver 
    rotations = move[1]                           # rotations (string) to be applied according to the solver 
    
    cube_orientation=h_faces.copy()               # generating a single cube orientation dict with h_faces
    cube_orientation.update(v_faces)              # generating a single cube orientation dict with h_faces and v_faces
    
    solution_in_dict = True                       # boolean for easier code reading... 80% chances the face is in dict...
    
    for side, face in cube_orientation.items():   # iteration over the current cube orientation dict (5 sides)
        if face == face_to_turn:                  # case the face to be turned is in the dictionary value
            return side+rotations                 # the dictionary key is returned, as the effective face location
        else:                                     # case the face to be turned is not in the dictionary value
            solution_in_dict = False              # boolean variable is changed to False
    
    if solution_in_dict == False:                 # case the face to be turned is not in the dictionary value
        return 'B'+rotations                      # the face to be turned must be the 6th one, the B side






def optim_moves1(moves):
    """Removes unnecessary moves that would cancel each other out, to reduce solving moves and time
    These movements are for instance a spin CW followed by a spin CCW, or viceversa."""
    
#     print("initial moves: ", moves)
#     print("len initial moves:", len(moves))
        
    optimization = False                 # boolean to track if optimizations are made
    to_optmize=[]                        # empty list to be populated with string index where optimization is possible
    str_length=len(moves)                # length of the robot move string
    idx=0                                # index variable, of optimizable move string locations, to populate the list
    for i in range(0,str_length,2):      # for loop of with steps = 2
        
        if moves[idx:idx+2]=='S1' and moves[idx+2:idx+4] =='S3':  # case S1 is folloved by S3
#                 print(f'S1 followed by S3 at index {idx}')
            optimization = True          # boolean to track optimization is set true
            to_optmize.append(idx)       # list is populated with the index 
            idx+=4                       # string index is increased by four, to skip the 2nd (already included) move  

        elif moves[idx:idx+2]=='S3'and moves[idx+2:idx+4] =='S1': # case S3 is folloved by S1
#                 print(f'S3 followed by S1 at index {idx}')
            optimization = True          # boolean to track optimization is set true
            to_optmize.append(idx)       # list is populated with the index 
            idx+=4                       # string index is increased by four, to skip the 2nd (already included) move  

        idx+=2                           # index variable is increased by two, to analayse next move
        if idx>=str_length+2:            # case the index variable reaches the string end 
            break                        # for loop is interrupted

    if optimization == False:            # case the moves string had no need to be optimized
#         print("Robot moves string: not optimization type 1 needed")
#         print("moves at opt1: ", moves)
#         print("len moves at opt1:", len(moves))
        return moves                     # original moves are returned

    else:                                # case the moves string has the need to be optimized
        to_remove=[]                     # empty list to be populated with all indididual characters to be removed from the moves
        for i in to_optmize:                        # iteration over the list of moves to be optimized
            to_remove.append((i, i+1, i+2, i+3))    # list is populated with the 4 caracters of the 2 moves
        
        new_moves=''                                # empty string to hold the new robot moves 
        remove = [item for sublist in to_remove for item in sublist] # list of characters is flattened
        for i in range(str_length):                 # iteration over all the characters of original moves
            if i not in remove:                     # case the index is not included in the list of those to be skipped
                new_moves+=moves[i]                 # the character is added to the new string of moves 
        
        print("Robot moves string: applied optimization type 1")
#         print("new_moves at opt1: ", new_moves)
#         print("len new_moves at opt1:", len(new_moves))
        return new_moves                            # the new string of robot moves is returned






def optim_moves2(moves):
    """Removes 2 flips when the second-last flip is F3, the last one is F2 and both are followed by same spins/rotations.
        Under these conditions, the second-last flip (F3) can be changed (to F1)."""
    
    str_length = len(moves)              # length of the robot move string
    F_list = []                          # empty lit collecting all the Flips
    F_count = 0                          # variable F_count (counter) is set to zero                         
    for i in range(str_length-2,-2,-2):  # iteration over the moves string, from the end, in steps of 2
        if moves[i] == 'F':              # case there is a F (flip) into the moves string
            to_add = 'F' + moves[i+1]    # type of flip (cases are 'F1', 'F2'and 'F3')
            F_list.append(to_add)        # F_list is populates with the related flip (cases are 'F1', 'F2'and 'F3')
            F_count += 1                 # variable F_count (counter) is increased by one
            if F_count == 2:             # case F_count (counter) equals 2
                break                    # for loop is interrupted

    if F_count < 2:                      # case there are less than two Flip cases in moves string
        return moves                     # moves in argument are returned
    
    else:                                # case there are at least two Flip cases in moves
        if not (F_list[-1] == 'F3' and F_list[-2] == 'F2'):  # case second-last flip is not 'F3' and last flip is not 'F2'
            # the first condition to remove 2 flips is not met
            return moves                 # moves in argument are returned
        
        else:                                # case the second-last flip is 'F3' and last flip is 'F2'
            # the first condition to remove 2 flips is met
            for i in range(str_length-2,-2,-2):  # iteration over the moves string, from the end,  in steps of 2
                if moves[i:i+2] == 'F2':     # case the moves string has 'F2' (by iterating from the string end...)
                    F2_idx = i               # moves string index for F (of the F2) character
                elif moves[i:i+2] == 'F3':   # case the moves string has 'F3' (by iterating from the string end...)
                    F3_idx = i               # moves string index for F (of the F3) character
                    break                    # for loop is interrupted
        
            chrs = F2_idx-F3_idx -2          # moves characters quantity in betwween F3 and F2
            chrs2 = str_length - F2_idx -2   # moves characters quantity after F2
            if chrs != chrs2:                # case the moves characters between F3 and F2 differ from those after F2
                if 'R' in moves[F2_idx+chrs+2:]:   # case the moves after F2 have an extra Rotation not present after F3
                    # the second condition to remove 2 flips is not met
#                     print("Robot moves string: not optimization type 2 needed")
                    return moves             # moves in argument are returned

            if moves[F3_idx+2: F3_idx+chrs+2] != moves[F2_idx+2: F2_idx+chrs+2]: # case moves in between F3 and F2 differ from those after F2
                # the second condition to remove 2 flips is not met
#                     print("Robot moves string: not optimization type 2 needed")
                return moves                 # moves in argument are returned
            
            else:                            # case the moves in between F3 and F2 equal those after F2
                # the second condition to remove 2 flips is also met
                new_moves=''                 # new_moves empty string to be populated with the new moves
                for i in range(str_length):  # iteration over the moves string
                    if i == F3_idx+1:        # moves index where to apply the change, from 3 (F3) to 1 (F1)
                        new_moves += '1'     # character 1 is added to the new_moves string
                    else:                    # moves index to keep the original moves
                        new_moves += moves[i]  # original moves charactes are appended to new_moves string
                print("Robot moves string: applied optimization type 2")
#                     print("new_moves at opt2: ", new_moves)
#                     print("len new_moves at opt2:", len(new_moves))
                return new_moves             # the new string of robot moves is returned







def count_moves(moves):
    """Counts the total amount of robot movements."""

    robot_tot_moves = 0               # counter for all the robot movements
    for i in range(len(moves)):       # iterates over the string of robot movements
        if moves[i] == 'F':           # case there is a cube flip on robot movements
            flips=int(moves[i+1])     # retrieves how many flips
            robot_tot_moves+=flips    # increases the total amount of robot movements

        elif moves[i] == 'R':         # case there is a layer rotation on robot movements
            robot_tot_moves+=1        # increases by 1 (cannot be more) the total amount of robot movements
        
        elif moves[i] == 'S':         # case there is a cube spin on robot movements
            robot_tot_moves+=1        # increases by 1 (cannot be more) the total amount of robot movements
    
    return robot_tot_moves            # total amount of robot moves is returned






def robot_required_moves(solution, solution_Text):
    """ This function splits the cube manouvre from Kociemba solver string, and generates a dict with all the robot movements.
        Based on the dict with all the robot moves, a string with all the movements is generated.
        The string with the robot movements might differ from the dict, when optimizing is possible."""
    
    global h_faces,v_faces
    
    solution=solution.strip()                     # eventual empty spaces are removed from the string
    solution=solution.replace(" ", "")            # eventual empty spaces are removed from the string
    starting_cube_orientation()                   # Cube orientation at the start, later updated after every cube movement on the robot
    robot={}                                      # empty dict to store all the robot moves
    moves=''                                      # empty string to store all the robot moves
    robot_tot_moves = 0                           # counter for all the robot movements
    
    if solution_Text != 'Error':                  # case the solver did not return an error
        blocks = int(round(len(solution)/2,0))    # total amount of blocks of movements (i.e. U2R1L3 are 3 blocks: U2, R1 and L1)
        
        # cube orientation and robot movement sequence selection
        for block in range(blocks):               # iteration over blocks of movements
            move=solution[:2]                     # move to be applied on this block, according to the solver
            solution=solution[2:]                 # remaining movements from the solver are updated
            adapted_move=adapt_move(move)         # the move from solver is adapted considering the real cube orientation
            robot_seq=moves_dict[adapted_move]    # robot movement sequence is retrieved
            robot[block]=robot_seq                # robot movements dict is updated
            moves+=robot_seq                      # robot movements string is updated
            cube_orient_update(robot_seq)         # cube orientation updated after the robot move from this block
                           
        moves = optim_moves1(moves)               # removes eventual unnecessary moves (that would cancel each other out)
        moves = optim_moves2(moves)               # removes eventual unnecessary flips
        robot_tot_moves = count_moves(moves)      # counter for the total amount of robot movements
        
    return robot, moves, robot_tot_moves  # returns a dict with all the robot moves, string with all the moves and total robot movements
    # NOTE: dict has all the theorethical robot movements, the string might differ due to optimization





if __name__ == "__main__":
    """ This function convert the cube solution string 'U2 L1 R1 D2 B2 R1 D2 B2 D2 L3 B3 R3 F2 D3 L1 U2 F2 D3 B3 D1' in robot moves
        Robot moves are printed on the REPL
        Robot moves are translated to servo moves: Initially are print per ach of the cube solving string manoeuvre
        Afterward all the strings are combined in a single string, for the Cubotino_servo.py module to control the servos."""  
    
    
#     solution = 'U2 L1 R1 D2 B2 R1 D2 B2 D2 L3 B3 R3 F2 D3 L1 U2 F2 D3 B3 D1' # this cube solution allows type 1 optimization (at least 2 Spins removal)
    solution = 'U2 D2 R2 L2 F2 B2'  # this cube solution allows type 2 optimization (2 flips removal)
#     solution = 'R2 L1 D3 F2 L2 B1 L1 U3 R1 F1 L2 D3 F2 D1 F2 B2 D2' # this cube solution allows type 1 optimization (at least 2 Spins removal)
#     solution = 'L1 D2 L1 D2 R2 F2 D2 R1 F3 R1 U1 R2 B3 L3 D1 R1 D2 B2 F3' # this cube solution has only 1st criteria for type2 optimization
#     solution = 'F3 U1 D2 R2 L2 U2 D2 R1 L2' # this cube solution has only only 1st criteria for type2 optimization

    print()
    print("Example of robot movements for solver solution: ", solution)
    print("Robot moves are noted with the 3 letters S, F, R (Spin, Flip, Rotate) followed by a number")
    print("Number '1' for S ans R identifies CW rotation, by loking to the bottom face, while number '3' stands for CCW")
    print("Example 'F1R1S3' means: 1x cube Flip, 1x (90deg) CW rotation of the 1st (bottom) layer, 1x (90deg) CCW cube Spin")
    print()
    

    
    solution_Text = ""
    robot, moves, robot_tot_moves = robot_required_moves(solution, solution_Text)
    print(f'\nnumber of robot movements: {robot_tot_moves}')
    
    print()    
    print(f'robot movements: ')
    
    servo_moves=""
    for step, movements in robot.items():
        print(f'step:{step}, robot moves:{movements}')
        servo_moves+=moves
    
    print(f'\nstring command to the robot servos driver: {moves}\n')
    

