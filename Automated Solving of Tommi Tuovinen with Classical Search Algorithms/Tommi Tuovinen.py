from search import GraphProblem, Node, Problem
from search import astar_search, breadth_first_tree_search, depth_first_tree_search, recursive_best_first_search, uniform_cost_search, genetic_search
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#Convert a nested Tuple to a nested List  : ((a,b, ..),(c,d, ..), ..) ---> [[a,b, ..], [c,d, ..], ..]
def listit(t):
    return list(map(listit, t)) if isinstance(t, (list, tuple)) else t

#Convert a nested List to a nested Tuple  : [[a,b, ..], [c,d, ..], ..] ---> ((a,b, ..),(c,d, ..), ..)
def tupleit(l):
    return tuple(map(tupleit, l)) if isinstance(l, (tuple, list)) else l

#Convert a m * n String (s) to a State List
def toState(s, m, n):
    ml = []
    for i in range(n):
        l = s[i*(m+1):(i+1)*(m+1)].upper()
        ml.append(list(l))
    sl = [x[:-1] for x in ml]
    return sl

Background = '.'
PlayerHead = 'H'
Wall = '#'
Exit = 'E'
Crate = 'C'
PlayerBody = 'B'

BEP = [Background,Exit,PlayerBody]

class Tommi_Tuovinen(Problem):
    def __init__(self, initial, goal=None):
        """ Define goal state and initialize a problem """
        #Set initial state
        super().__init__(initial, goal)
        #Find GOAL Coordinates and save in self.goal variable
        for i in range(len(initial)):
            if initial[i].count(Exit) != 0:
                self.goal = (i, initial[i].index(Exit))
                break

    def find_player(self, state):
        """Return the index of the player in a given state"""
        #Find PlayerHead Coordinates PX , PY
        for i in range(len(state)):
            if state[i].count(PlayerHead) != 0:
                return (i, state[i].index(PlayerHead))
            
    def find_carte(self, state):
        """Return the index of the Crate in a given state"""
        #Find Crate Coordinates PX , PY
        for i in range(len(state)):
            if state[i].count(Crate) != 0:
                return (i, state[i].index(Crate))
        else:
            return False

    def actions(self, state,m):
        """ Return the actions that can be executed in the given state.
        The result would be a list, since there are only four possible actions
        in any given state of the environment """
        #Find Cordinate of Player and put in : PlayerX , PlayerY
        playerX, playerY = self.find_player(state)
    
        #Covariate Variable
        delta = { 'DOWN': (1, 0), 'LEFT': (0, -1), 'RIGHT': (0, 1)}

        possible_actions = []

        #Count up action in best node
        num_of_up_action = 0
        i = 0
        m = m[::-1]
        #Sreach in invers List of Action
        for action in m:
            if i == 3:
                break
            if action == 'UP':
                num_of_up_action += 1
            i += 1

        #If almost num of up action is less than 3 add to possibel_action
        if num_of_up_action < 3:
            #Set delta of X,Y in dx,dy 
            dx, dy = -1,0
            #If next sate[i][j] is just Background : GO ON
            if state[playerX+dx][playerY+dy] != Wall != PlayerBody:
                possible_actions.append('UP')

        for action in delta:
            #Set delta of X,Y in dx,dy 
            dx, dy = delta[action][0], delta[action][1]
            #If next sate[i][j] is just Background : GO ON
            if state[playerX+dx][playerY+dy] != Wall != PlayerBody:
                possible_actions.append(action)

        return possible_actions
    
    def result(self, state, action):
        """ Given state and action, return a new state that is the result of the action.
        Action is assumed to be a valid action in the state """
        # px, py are coordinates of the PLAYER
        px, py = self.find_player(state)
        # the New State
        new_state = listit(state)
        # dx , dy : One Step Toward Action Direction
        delta = { 'DOWN': (1, 0), 'LEFT': (0, -1), 'RIGHT': (0, 1),'UP': (-1, 0)}
        dx = delta[action][0]
        dy = delta[action][1]
        #nx, ny : Coordinates of Next Point 
        nx = px + dx
        ny = py + dy
        if new_state[nx][ny] in BEP:
            #If next state is goal Push the PlayerHead
            if (nx,ny) == self.goal:
                new_state[px][py] = PlayerBody
                new_state[nx][ny] = PlayerHead #Put Player in new Location
            #Elif next state is Back Ground Push it   
            elif new_state[nx][ny] == Background:
                new_state[nx][ny] = PlayerHead
                new_state[px][py] = PlayerBody #Put PlayerBody in pre point
                #Move the lowest place
                if new_state[nx+1][ny] == Background :
                    new_state[nx][ny] = PlayerBody
                    nx = nx + 1
                    #Find Lowest Place : GO ON 
                    while new_state[nx][ny]==Background:
                        new_state[nx][ny] = PlayerBody
                        nx = nx + 1
                    #If the new pint is wall ? : Put PlayerHead in pre point
                    new_state[nx-1][ny-0] = PlayerHead
                #If Lowest Place is Goal : Hold it
                if (nx+1,ny) == self.goal:
                        new_state[nx][ny] = PlayerBody
                        nx +=1
                        new_state[nx][ny] = PlayerHead
        #Elif next state is Carte : GO ON
        elif new_state[nx][ny] == Crate:
            new_state = self.move_Crate(new_state, px, py, delta[action])
        return tupleit(new_state)
    
    def goal_test(self, state):
        """ Given a state, return True if state is a goal state or False, otherwise """
        return self.find_player(state) == self.goal
    
    def h(self, node):
        """ Return the heuristic value for a given state."""
        #Find Player Cordinate
        px,py = self.find_player(node.state)
        #Find Goal Cordinate
        gx,gy = self.goal
        #Count Background item
        remaining_BackGround = 0
        for row in node.state:
            for cell in row:
                if cell == Background:
                    remaining_BackGround += 1
        #If Carte is there in state : Calculate h(n)
        if self.find_carte(node.state) != False:
            distance_Goal = (abs(gx-px) + abs(gy-py))
            remaining_BackGround = 0
            for row in node.state:
                for cell in row:
                    if cell == Background:
                        remaining_BackGround += 1
            return distance_Goal + remaining_BackGround
        #Elif Background number is less than 50 : Calculate h(n)
        elif remaining_BackGround <=50:
            distance_Goal = abs((gx-px)**2 + (gy-py)**2)
            remaining_PlayerBody = 0
            for row in node.state:
                for cell in row:
                    if cell == PlayerBody:
                        remaining_PlayerBody += 1
            return (distance_Goal - remaining_PlayerBody)
        #Else Calculate h(n) for other state
        else:
            distance_Goal = abs((gx-px)**2 + (gy-py)**2)
            return distance_Goal
        
    def move_Crate(self, cstate, px, py, d):
        #px, py : The PLAYER Coordinates
        #cx, cy : The CRATE Coordinates
        cx = px + d[0]
        cy = py + d[1]
        #p=nx, ny : The Next Point Coordinates
        nx = cx + d[0]
        ny = cy + d[1]
        # If the Last Cell is in [BACKGROUND, HOLE, GOAL] ---> Empty
        if cstate[nx][ny] == Background:
            #Move PLAYER to Next Cell
            cstate[px][py] = PlayerBody
            cstate[cx][cy] = PlayerHead
            # if the Last Cell is BACKGROUND : Move CRATE to Next Cell
            if cstate[nx][ny] == Background :
                cstate[nx][ny] = Crate
            if cstate[nx+1][ny] == Background :
            # while the Last Cell nx+1 is BackGround : FILL in The nx+1
                cstate[nx][ny] = Background
                nx += 1
                while cstate[nx][ny] == Background:
                    nx += 1
                #put Crate for nx-1 Befor Wall
                cstate[nx-1][ny] = Crate
            
        return cstate  
#Show the Animation of Action in state
matrices = []
def update(frame):
    matrix0 = matrices[frame]
    matrix = matrix0.state
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == '#':
                color = 'black'
            elif matrix[i][j] == '.':
                color = 'white'
            elif matrix[i][j] == 'C':
                color = 'blue'
            elif matrix[i][j] == 'H':
                color = 'green'     
            elif matrix[i][j] == 'B':
                color = 'orange'
            elif matrix[i][j] == 'E':
                color = 'red'
            
            plt.gca().add_patch(plt.Rectangle((j, i), 1, 1, facecolor=color))
    
    
    ax.set_xlim(0, 19)
    ax.set_ylim(0, 15)
    ax.set_xticks(range(19))
    ax.set_yticks(range(15))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    
if __name__ == '__main__':

########################################################################################
#                                                                                      #
#                                       ?                                              #
#                             CHOSE A LEVEL OF GAME                                    #
#                                       ?                                              #
#                                                                                      #
########################################################################################
    s1 = '''###################
#.................#
#.................#
#............#....#
#............#....#
#.......#...##....#
#..H....#...##..E.#
###################
###################
..#...#...#...#...#
#...#...#...#...#..
###################
###################
###################
###################
'''

    s2 = '''###################
#.................#
#.................#
#.................#
#..H............E.#
#####.........#####
#####.............#
#####.............#
#####.............#
#####.............#
#####.....#########
#####.............#
#####.............#
#####.............#
###################
'''

    s3 = '''###################
#........#........#
#........#........#
###.............###
#.................#
#.....H.....E.....#
#....###...###....#
#..######.######..#
#.......#.#.......#
#.......###.......#
###.............###
#.................#
#.................#
######.......######
###################
'''

    s4 = '''###################
#..#...........#..#
#..#.....E.....#..#
#..#....###....#..#
#..#...........#..#
#..#...........#..#
#..#...........#..#
#..#...........#..#
#..#...........#..#
#..#...........#..#
#..#...........#..#
#..#...........#..#
#..#...........#..#
#..#.....H.....#..#
###################
'''
    
    s5 = '''###################
######....#########
######.E..#########
#########.#########
#########.#########
#########....######
#########....######
#########....######
#########.#########
######....#########
######....#########
######....#########
#########.#########
#########H#########
###################
'''
    
    s6 = '''###################
#.................#
#.................#
#.................#
#...C........C....#
#..###......###...#
#.................#
#...H........E....#
#########.##.#.####
###################
#.................#
#.................#
#.................#
#.................#
###################
'''

    s7 = '''###################
#.................#
#...............C.#
#............####.#
#.................#
#.......#...#.#...#
#..H....#...#.#E..#
#.###############.#
#.###############.#
#.#...#...#...#...#
#...#...#...#...#.#
###################
#.................#
#.................#
###################
'''

    s8 = '''###################
#.................#
#......C...C......#
#.....###.###.....#
#.................#
#......C...C......#
#.....###.###.....#
#.................#
#..E...C...C...H..#
#.#######.#######.#
#.##...##.##...##.#
#.##.#.##.##.#.##.#
#.##.#.#####.#.##.#
#.................#
###################
'''

    s9 = '''###################
#.................#
#...C.............#
#...C.............#
#...C.............#
#...CCC...........#
#...CCC.C.........#
#..CCCC.CC.......E#
#..CCCC.CC.......C#
#..CCCCCCC.....C.C#
#..CCCCCCC...C.C.C#
#.CCCCCCCCC..C.C.C#
#.CCCCCCCCC..C.C.C#
#HCCCCCCCCCCCC.CCC#
###################
'''

    s10 = '''###################
#.................#
#.................#
#.................#
#.................#
#.E..........C....#
####.##.....###...#
#######.....###...#
#.................#
#.................#
#...H.............#
#..#############..#
###################
#.................#
###################
'''

    # Convert S to List then to Tuple
    stolist = toState(s1, 19, 15)
    initstate = tupleit(stolist)

    # Create a Train Braining (Problem) Object
    e1 = Tommi_Tuovinen(initstate)

    # Start the Search Algorithm From Initial Node (e1)
    # s = astar_search(e1)                     #   A*
    # s = depth_first_tree_search(e1)          #   DFS
    # s = breadth_first_tree_search(e1)        #   BFS
    # s = recursive_best_first_search(e1)        #  RBFS



    #Creat a object for showing 
    fig, ax = plt.subplots()

    for node in s.path():
        # print(node.action)
        l = list(node.state)
        l = l[::-1]
        node.state = l
        matrices.append(node)
        # for s in node.state:
        #     print(s)
        # input()

    #FuncAnimation Show for Multi photos
    animation = FuncAnimation(fig, update, frames=len(matrices), repeat = False)
    plt.show()

    #PLEAS After finsh of the Game Colse Animation Page