#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

//Grid base
const int row = 6;
const int col = 7;
std::pair<int, int> origin = { 0,1 };
std::pair<int, int> goal = { 3,4 };
std::string grid[row][col];

void initGrid()
{
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            grid[i][j] = "--";
        }
    }
    //Obstacles
    grid[1][2] = "##";
    grid[1][3] = "##";
    grid[1][4] = "##";
    grid[1][5] = "##";
    grid[2][2] = "##";
    grid[2][5] = "##";
    grid[3][1] = "##";
    grid[3][2] = "##";
    grid[3][5] = "##";
    grid[4][4] = "##";
    grid[4][5] = "##";
    
    //Goal
    grid[goal.first][goal.second] = "GG";
}

void draw()
{
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            std::cout << grid[i][j] << "\t";
        }
        std::cout << "\n\n";
    }
    std::cout << "****************************************************\n";
}

bool alreadyVisited(std::vector<std::pair<int,int>> vec, int row, int col)
{
    std::vector<std::pair<int, int>>::iterator it;

    it = std::find(vec.begin(), vec.end(), std::make_pair(row, col));
    if (it != vec.end()) return true;
    else return false;
}

int EstManhattanDistance(int row, int col)
{
    //The estimated path cost the goal from current position
    int h = 0;
    int xCost = 0, yCost = 0;

    //Horizontal distance
    //We're not in the right column, so we need to move left or right
    if (col != goal.second)
    {
        xCost = abs(col - goal.second) * 2; //Path cost of 2 for both left and right case
    }
    //We are in the correct column, no left/right movement necessary
    else
    {
        xCost = 0;
    }

    //Vertical distance
    //Wind condition is 1 for upward movement, but 3 for downward movement
    //Current vertical position on the grid is lower than the goal and must move upward
    if (row > goal.first)
    {
        yCost = (row - goal.first); //Path cost of 1
    }
    //Current vertical position on the grid is higher than the goal and must move downward
    else if (row < goal.first)
    {
        yCost = (goal.first - row) * 3; //Path cost of 3
    }
    //Current vertical position is on the same row as goal, no upward/downward movement necessary
    else
    {
        yCost = 0;
    }

    h = xCost + yCost;

    return h;
}

bool lowestTotalEstCost(std::vector<int> v1, std::vector<int> v2)
{
    //Sort by the lowest total estimated path cost
    if (v1[2] != v2[2])
    {
        return v1[2] < v2[2];
    }
    //If total estimated path costs are equal, sort by stepRank
    else
    {
        return v1[5] < v2[5];
    }
}

int main()
{
    initGrid();
    
    //Keep track of current point
    int currX, currY = 0;

    //DFS flags and vars
    bool dfsDone = false;
    bool dfsFrontierUpdated = false;
    //LIFO stack for node expansion
    std::vector<std::pair<int, int>> dfsFrontier;
    //explored set to avoid repeat path
    std::vector<std::pair<int, int>> dfsVisited;

    //Start at origin
    std::cout << "===== DFS Start =====\n\n";
    dfsFrontier.push_back(std::make_pair(origin.first, origin.second));
    dfsVisited.push_back(std::make_pair(origin.first, origin.second));
    grid[dfsFrontier.back().first][dfsFrontier.back().second] = std::to_string(dfsVisited.size() - 1);
    draw();
    
    //Path towards goal - DFS
    while (dfsDone != true)
    {
        currX = dfsFrontier.back().first;
        currY = dfsFrontier.back().second;
        //Check order: < ^ > v
        int leftBound = currY - 1;
        int rightBound = currY + 1;
        int upBound = currX - 1;
        int downBound = currX + 1;

        bool leftPath, rightPath, upPath, downPath = { false };

        leftPath = alreadyVisited(dfsVisited, currX, leftBound);
        rightPath = alreadyVisited(dfsVisited, currX, rightBound);
        upPath = alreadyVisited(dfsVisited, upBound, currY);
        downPath = alreadyVisited(dfsVisited, downBound, currY);

        //If goal state reached, set done = true to exit loop
        if (currX == goal.first && currY == goal.second)
        {
            dfsDone = true;
        }
        else
        {
            //From current step in the grid, visit all possible moves
            //Next step conditions: Cannot go out of bounds, cannot go into obstacles
            //***Cannot re-visit grid location (avoid infinite loop)
            
            //We know we have visited a node already if the default "--" has been replaced with the exploration step number
            //Alternate approach would use std::find(dfsVisited.begin(), visited.end(), std::make_pair(gridVerticalCoordinate, gridHorizontalCoordinate)) and if it exists, skip. If not, visit and update vectors

            //Is it possible to move left?
            if (leftBound >= 0 && grid[currX][leftBound] != "##" && (grid[currX][leftBound] == "--" || grid[currX][leftBound] == "GG"))
            {
                //Possible move. Assign grid with current step, update visited list, then push onto LIFO stack
                dfsVisited.push_back(std::make_pair(currX, leftBound));
                grid[currX][leftBound] = std::to_string(dfsVisited.size() - 1);
                draw();
                if (currX == dfsFrontier.back().first && currY == dfsFrontier.back().second)
                {
                    dfsFrontier.pop_back();
                }
                dfsFrontier.push_back(std::make_pair(currX, leftBound));
                dfsFrontierUpdated = true;
            }
            //Is it possible to move up?
            if (upBound >= 0 && grid[upBound][currY] != "##" && (grid[upBound][currY] == "--" || grid[upBound][currY] == "GG"))
            {
                dfsVisited.push_back(std::make_pair(upBound, currY));
                grid[upBound][currY] = std::to_string(dfsVisited.size() - 1);
                draw();
                if (currX == dfsFrontier.back().first && currY == dfsFrontier.back().second)
                {
                    dfsFrontier.pop_back();
                }
                dfsFrontier.push_back(std::make_pair(upBound, currY));
                dfsFrontierUpdated = true;
            }
            //Is it possible to move right?
            if (rightBound < col && grid[currX][rightBound] != "##" && (grid[currX][rightBound] == "--" || grid[currX][rightBound] == "GG"))
            {
                dfsVisited.push_back(std::make_pair(currX, rightBound));
                grid[currX][rightBound] = std::to_string(dfsVisited.size() - 1);
                draw();
                if (currX == dfsFrontier.back().first && currY == dfsFrontier.back().second)
                {
                    dfsFrontier.pop_back();
                }
                dfsFrontier.push_back(std::make_pair(currX, rightBound));
                dfsFrontierUpdated = true;
            }
            //Is it possible to move down?
            if (downBound < row && grid[downBound][currY] != "##" && (grid[downBound][currY] == "--" || grid[downBound][currY] == "GG"))
            {
                dfsVisited.push_back(std::make_pair(downBound, currY));
                grid[downBound][currY] = std::to_string(dfsVisited.size() - 1);
                draw();
                if (currX == dfsFrontier.back().first && currY == dfsFrontier.back().second)
                {
                    dfsFrontier.pop_back();
                }
                dfsFrontier.push_back(std::make_pair(downBound, currY));
                dfsFrontierUpdated = true;
            }
            //No possible move from this position, remove it from frontier
            else if(dfsFrontierUpdated == false)
            {
                dfsFrontier.pop_back();
            }
            dfsFrontierUpdated = false;
        }
    }
    std::cout << "===== DFS End =====\n\n";

    initGrid();
    std::cout << "\n\n";

    //A* flags and vars
    bool aStarDone = false;
    bool aStarFrontierUpdated = false;
    bool deleteMark = false;
    int actCost = 0, estCost = 0, totalEstCost = 0, stepRank = 0;
    //priority queue that needs to sort by lowest path cost of node to be expanded
    std::vector<std::vector<int>> aStarFrontier;
    //visited - Keep track of row and col coordinate, and the total estimated path cost f(n) = g(n)+h(n)
    std::vector<std::vector<int>> aStarVisited;

    //Start at origin
    std::cout << "===== A* Start =====\n\n";
    estCost = EstManhattanDistance(origin.first, origin.second);
    totalEstCost = actCost + estCost;
    aStarFrontier.push_back({ origin.first, origin.second, totalEstCost, actCost, estCost, stepRank });
    aStarVisited.push_back({ origin.first, origin.second, totalEstCost, actCost, estCost, stepRank });
    grid[origin.first][origin.second] = std::to_string(aStarVisited.size() - 1);
    draw();

    /*
    A* vector elements:
    [0]: row position
    [1]: col position
    [2]: total estimated cost - f(n)
    [3]: actual cost - g(n)
    [4]: estimated cost - h(n)
    [5]: the step # for breaking rank if necesssary (lowest preferred)
    */
    
    //Wind condition path costs:
    int horizontalCost = 2;
    int upCost = 1;
    int downCost = 3;
    
    //Path towards goal - A*
    while (aStarDone != true)
    {
        //Priority queue, so we'll always take from the front
        //*which should be a sorted list of lowest to highest total estimated path cost
        currX = aStarFrontier.front()[0];
        currY = aStarFrontier.front()[1];
        actCost = aStarFrontier.front()[3];
        //Check order: < ^ > v
        int leftBound = currY - 1;
        int rightBound = currY + 1;
        int upBound = currX - 1;
        int downBound = currX + 1;

        //If goal state reached, set done = true to exit loop
        if (currX == goal.first && currY == goal.second)
        {
            aStarDone = true;
        }
        else
        {
            //Is it possible to move left?
            if (leftBound >= 0 && grid[currX][leftBound] != "##" && (grid[currX][leftBound] == "--" || grid[currX][leftBound] == "GG"))
            {
                stepRank++;
                //Possible move, calculate path and estimated costs
                estCost = EstManhattanDistance(currX, leftBound);
                int leftMove = actCost + horizontalCost;
                totalEstCost = leftMove + estCost;
                //Update lists
                aStarVisited.push_back({ currX, leftBound, totalEstCost, leftMove, estCost, stepRank });
                grid[currX][leftBound] = std::to_string(aStarVisited.size() - 1);
                draw();
                if (currX == aStarFrontier.front()[0] && currY == aStarFrontier.front()[1])
                {
                    aStarFrontier.erase(aStarFrontier.begin());
                }
                aStarFrontier.push_back({ currX, leftBound, totalEstCost, leftMove, estCost, stepRank });
                aStarFrontierUpdated = true;
            }
            //Is it possible to move up?
            if (upBound >= 0 && grid[upBound][currY] != "##" && (grid[upBound][currY] == "--" || grid[upBound][currY] == "GG"))
            {
                stepRank++;
                //Possible move, calculate path and estimated costs
                estCost = EstManhattanDistance(upBound, currY);
                int upMove = actCost + upCost;
                totalEstCost = upMove + estCost;
                //Update lists
                aStarVisited.push_back({ upBound, currY, totalEstCost, upMove, estCost, stepRank });
                grid[upBound][currY] = std::to_string(aStarVisited.size() - 1);
                draw();
                if (currX == aStarFrontier.front()[0] && currY == aStarFrontier.front()[1])
                {
                    aStarFrontier.erase(aStarFrontier.begin());
                }
                aStarFrontier.push_back({ upBound, currY, totalEstCost, upMove, estCost, stepRank });
                aStarFrontierUpdated = true;
            }
            //Is it possible to move right?
            if (rightBound < col && grid[currX][rightBound] != "##" && (grid[currX][rightBound] == "--" || grid[currX][rightBound] == "GG"))
            {
                stepRank++;
                //Possible move, calculate path and estimated costs
                estCost = EstManhattanDistance(currX, rightBound);
                int rightMove = actCost + horizontalCost;
                totalEstCost = rightMove + estCost;
                //Update lists
                aStarVisited.push_back({ currX, rightBound, totalEstCost, rightMove, estCost, stepRank });
                grid[currX][rightBound] = std::to_string(aStarVisited.size() - 1);
                draw();
                if (currX == aStarFrontier.front()[0] && currY == aStarFrontier.front()[1])
                {
                    aStarFrontier.erase(aStarFrontier.begin());
                }
                aStarFrontier.push_back({ currX, rightBound, totalEstCost, rightMove, estCost, stepRank });
                aStarFrontierUpdated = true;
            }
            //Is it possible to move down?
            if (downBound < row && grid[downBound][currY] != "##" && (grid[downBound][currY] == "--" || grid[downBound][currY] == "GG"))
            {
                stepRank++;
                //Possible move, calculate path and estimated costs
                estCost = EstManhattanDistance(downBound, currY);
                int downMove = actCost + downCost;
                totalEstCost = downMove + estCost;
                //Update lists
                aStarVisited.push_back({ downBound, currY, totalEstCost, downMove, estCost, stepRank });
                grid[downBound][currY] = std::to_string(aStarVisited.size() - 1);
                draw();
                if (currX == aStarFrontier.front()[0] && currY == aStarFrontier.front()[1])
                {
                    aStarFrontier.erase(aStarFrontier.begin());
                }
                aStarFrontier.push_back({ downBound, currY, totalEstCost, downMove, estCost, stepRank });
                aStarFrontierUpdated = true;
            }
            //No possible move from this position, remove it from queue
            else if (aStarFrontierUpdated == false)
            {
                aStarFrontier.erase(aStarFrontier.begin());
            }
            aStarFrontierUpdated = false;

            //Sort nodes by path cost for the next step
            //Sorting by the lowest total estimated cost, which is [2] in A* vector elements
            std::sort(aStarFrontier.begin(), aStarFrontier.end(), lowestTotalEstCost);
        }
    }
    std::cout << "===== A* End =====\n\n";
    return 0;
}