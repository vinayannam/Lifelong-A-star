# Team: Cybot (Team Project Topic 2)
In this project we implemented the “D* Lite”, Sven Koenig and Maxim Likhachev, AAAI 2002,and integrated into the Pacman domain for path-finding problems (from start to a fixed goal location).

## Authors and Contributions:
> - Setu Durgesh Vinay Annam: Implementation of A* search, Lifelong planning A* search, D* Lite, Looking for online periodic, journals, and public resources to inspire the brain storm and make contribution the project
> - Rishti Gupta: Implementation of A* search, Lifelong planning A* search, D* Lite, Looking for online periodic, journals, and public resources to inspire the brain storm and make contribution the project
> - Satrajit Maitra: Abstract, Introduction, Implementation comparison, technical approach, using pacman domain in the A* search, Lifelong planning A* search, D* Lite , comparing the performance of lifelong planning A* search and D* Lite sections of Final Report, Looking for online periodic, journals, and public resources to inspire the brain storm and make contribution the project
> - Hsin-Jung Lee: Abstract, Introduction, Implementation comparison, technical approach, using pacman domain in the A* search, Lifelong planning A* search, D* Lite , comparing the performance of lifelong planning A* search and D* Lite sections of Final Report, Looking for online periodic, journals, and public resources to inspire the brain storm and make contribution the project


## Run the project
> Make sure that the Python version to run the project is Python 2.7. If you have python3 installed in your PC by default and do not wish to change it to python 2.7 you may create a virtual environment with python 2.7 which will not disrupt any existing project or dependencies.
1. Download the project from the git repository using the command below. 
> git clone https://github.com/vinayannam/Lifelong-A-star.git
2. Navigate to the directory (using `cd directory_name`)
3. The `pacman.py` uses the search agents from the `SearchAgent.py` file.
4. The basic command to play with the pacman in the pacman domain is 
> python pacman.py
5. To the pacman we need to specify the layout (using the `-l` flag) and the agent type (using the `-p` flag).
6. The search agent we use is the `AStarPositionSearchProblem` that requires all the helper methods and properties to implement the A* searching algorithms in the paper.
7. So the command used to run the algorithms is
> python pacman.py -l **LAYOUT** -z .5 -p SearchAgent -a fn=**SEARCH_ALGORITHM**

## LAYOUT (Layouts Available)
> - masterplan
> - tinyMaze
> - testMaze
> - smallMaze
> - openMaze
> - mindbending
> - mediumMaze
> - contoursMaze
> - bigMaze

## SEARCH_ALGORITHM (Implemented search Algorithms):
> - astar : The dynamic A* search implementation
> - lastar: The Life Long A* search implementation
> - dstar : The DLite* search implementation

## Example usage of the command:
> python pacman.py -l masterplan -z .5 -p SearchAgent -a fn=astar
