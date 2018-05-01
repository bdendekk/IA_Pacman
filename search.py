# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

from game import Directions
s = Directions.SOUTH
w = Directions.WEST
e = Directions.EAST
n = Directions.NORTH

# dictionnaire contenant la "boussole" (liste des directions possibles) 
compass = {'South': s, 'West': w, 'East': e, 'North': n}

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    print problem

    "*** YOUR CODE HERE ***"

    

    frontier = util.Stack()  #sommets vus mais pas explores
    frontier.push( (problem.getStartState(), []) ) #dans la pile on associe chaque sommet au chemin parcouru pour y parvenir

    # NB : un sommet est represente par ses coordonnees
    visited = [] #liste des sommets visites

    while not frontier.isEmpty() :
        current = frontier.pop()  # etat actuel du probleme

        if problem.isGoalState(current[0]):
            return current[1]  # si on est arrive, on renvoie le chemin

        else:  #le but n'est pas encore atteint, i.e on est pas arrive
            if current[0] not in visited:
                visited.append(current[0])
                for succ in problem.getSuccessors(current[0]):
                    if succ[0] not in visited:
                        path = current[1]+[compass[succ[1]]]
                        frontier.push( (succ[0], path) ) 

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    #La methode pour la recherche en largeur est la meme que pour la recherche en profondeur,
    #sauf que l'on utilise une file (LIFO) a la place d'une pile (FIFO) pour la recherche en largeur.
    frontier = util.Queue()
    frontier.push((problem.getStartState(), []))
    visited = []
    while not frontier.isEmpty():
        current = frontier.pop()
        if problem.isGoalState(current[0]):
            return current[1]
        else:
            if current[0] not in visited:
                visited.append(current[0])
                for succ in problem.getSuccessors(current[0]):
                    if succ[0] not in visited:
                        path = current[1] + [compass[succ[1]]]
                        frontier.push((succ[0], path))

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    """ init """
    # on utilise une PriorityQueue car on veut traiter les sommets avec un cout interessant en priorite
    frontier = util.PriorityQueue()  #sommets vus mais pas explores
    start = problem.getStartState()
    frontier.update(start, 0)
    came_from = {}  #on veut garder une trace de d'ou on viens pour chaque sommet visite
    cost_so_far = {}  #chaque sommet associe a son cout total depuis la position start 
    came_from[start] = None
    cost_so_far[start] = 0

    """ la recherche commence ! """
    while not frontier.isEmpty():  #tant qu'il reste des sommets vus a explorer
        current = frontier.pop()

        if problem.isGoalState(current):  # si on est arrive
            break  # on va chercher le chemin
        
        else:
            for succ in problem.getSuccessors(current):
                new_cost = cost_so_far[current] + 1 # cout du chemin vers ce successeur
                if succ[0] not in came_from: # on verifie qu'on ne retourne pas sur ses pas
                    cost_so_far[succ[0]] = new_cost  # mise a jour du cout
                    priority = new_cost + heuristic(succ[0], problem) # mise a jour de la priorite en fonction de l'heuristique
                    frontier.update(succ[0], priority)
                    came_from[succ[0]] = current  #maintenant que si on est a succ, on vient de current


    """ Recuperation du chemin """
    path = []
    while current != start:  # on remonte le graphe pour recuperer le chemin parcouru, mais a l'envers !
        before = came_from[current]
        path.append(movement(before,current))
        current = before
    #path.append(start)
    path.reverse()  # pour avoir le chemin dans le bon ordre
    print path
    return path

def movement(a,b):
    """ donne la direction du mouvement de a vers b """
    if a[1] < b[1]: # mouvement vers le haut
        return n  

    if a[1] > b[1]: # mouvement vers le bas
        return s  

    if a[0] < b[0]: # mouvement ver la droite
        return e

    if a[0] > b[0]: # mouvement vers la gauche
        return w



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

