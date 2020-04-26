Command: 
python pacman.py -l <LAYOUT> -z .5 -p SearchAgent -a fn=<SEARCH_ALGORITHM>

LAYOUT (Layouts Available):
    - masterplan
    - tinyMaze
    - testMaze
    - smallMaze
    - openMaze
    - mindbending
    - mediumMaze
    - contoursMaze
    - bigMaze

SEARCH_ALGORITHM (Implemented search Algorithms):
    - astar : The dynamic A* search implementation
    - lastar: The Life Long A* search implementation
    - dstar : The DLite* search implementation

Example usage of the command:
python pacman.py -l masterplan-z .5 -p SearchAgent -a fn=astar