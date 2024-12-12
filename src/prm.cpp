#include "prm.h"
#include <QTime>
#include <cmath>
#include <set>
#include <queue>
#include <unordered_map>

PRM::PRM()
    :start_(-1, -1), goal_(-1, -1)
{
    kd_tree_ = nullptr;
    random_gen_ = new QRandomGenerator(QDateTime::currentMSecsSinceEpoch() % UINT_MAX);
}

PRM::~PRM()
{
    //release kd tree
    if(kd_tree_ != nullptr)
    {
        delete kd_tree_;
    }

    prm_graph_.destroyGraph();
}

void PRM::constructGraph(const int **mat, int row, int col)
{
    assert(start_.x() != -1 && goal_.x() != -1);

    graph_mat_ = mat;

    //clear graph
    prm_graph_.destroyGraph();

    //select points randomly
    graph_mat_row_ = row;
    graph_mat_col_ = col;

    int n = VERTEX_COEFFICIENT * row * col;
    int vertex_k = K_COEFFICIENT * row * col; // the scale of nearest beightbor

    QVector<QPoint> points;
    points.push_back(start_);   //add start point
    points.push_back(goal_);     //add end point

    int i=0, j=0;
    while(n)
    {
        i = random_gen_->bounded(row);
        j = random_gen_->bounded(col);

        if(mat[i][j] == 0 || mat[i][j] == 2)
        {
            points.push_back(QPoint(j, i));
            n--;
        }
    }

    prm_graph_.addVertex(points);   //set vertex table of graph

    this->generateArc(points, vertex_k);      //generate arc of graph
}

/**
@breif PRM::generateArc
@param points: the nodes of the graph
@param vertex_k: the scale of nearest beightbor
@desc generate the arc of the graph by connecting the K nearest neighbors
 */
void PRM::generateArc(const QVector<QPoint>& points, const int vertex_k)
{
    if(prm_graph_.getVertex().size() == 0)
    {
        return;
    }

    //construct Kd tree
    if(kd_tree_ != nullptr)
    {
        delete kd_tree_;
    }

    kd_tree_ = new KdTree(vertex_k);
    kd_tree_->initKdTree(points);

    QVector<int> neighbors;

    //search the K nearest neighbors
    QVector<QPoint>::const_iterator it = points.begin();
    int index = 0;
    float distance, energy;

    for(; it !=points.end(); it++)
    {
        neighbors.clear();
        neighbors = kd_tree_->getKNN(*it);

        for(int i=0; i<neighbors.size(); i++)
        {            
            if(checkPath(points[index], points[neighbors[i]]))
            {
                distance = calHeuristicDistance(points[index], points[neighbors[i]]);
                energy = calHeuristicEnergy(points[index], points[neighbors[i]]);

                prm_graph_.addArc(index, neighbors[i], distance, energy);     //add to arc table
                prm_graph_.addArc(neighbors[i], index, distance, energy);     //add to arc table
            }
        }
        index++;
    }
}

void PRM::setStartPoint(const QPoint &point)
{
    start_.setX(point.x());
    start_.setY(point.y());
}

void PRM::setEndPoint(const QPoint &point)
{
    goal_.setX(point.x());
    goal_.setY(point.y());
}

void PRM::setRandomSeed(int seed)
{
    if(seed == 0)
    {
        random_gen_->seed(QDateTime::currentMSecsSinceEpoch() % UINT_MAX);
    }
    else
    {
        random_gen_->seed(seed);
    }
}

void PRM::searchPath(Algorithm algorithm, Strategy strategy)
{
    if(algorithm == Alg_AStar)
    {
        AStar(strategy);
    }
    else if(algorithm == Alg_Dijkstra)
    {
        Dijkstra(strategy);
    }
    else
    {
        throw std::runtime_error("Invalid stragety");
    }
}

float PRM::calPathCost(const QVector<QPoint> &points, Strategy strategy)
{
    float cost = 0.0f;

    if(points.size() == 0)
    {
        return cost;
    }

    auto heuristic_function = selectHeuristic(strategy);

    for(int i=0; i<points.size()-1; i++)
    {
        cost += heuristic_function(points[i], points[i+1]);
    }

    return cost;
}

const QVector<QPoint> &PRM::getPath() const
{
    return path_;
}

const Graph &PRM::getGraph() const
{
    return prm_graph_;
}

/**
@brief PRM::Dijkstra
@param distance_first: true for distance first, false for energy first
@desc Dijkstra algorithm
*/
void PRM::Dijkstra(Strategy strategy)
{
    auto heuristic_function = selectHeuristic(strategy);

    const QVector<Graph::Vertex> vertex = prm_graph_.getVertex();

    int start = prm_graph_.getVertex(start_);
    int goal = prm_graph_.getVertex(goal_);

    std::priority_queue<PathCost, QVector<PathCost>, std::greater<PathCost>> queue;  //gobal distance
    std::unordered_map<int, int> parent;  //the parent node of path node
    std::unordered_map<int, float> open_list; //current actual distance
    std::set<int> close_list; // visited set
    QVector<int> neightbor;

    queue.push(PathCost(start, 0));  //push start point into queue
    parent[start] = -1;   // -1 stand for no parent node
    open_list[start] = 0;

    while(!queue.empty())
    {
        PathCost current_cost = queue.top();
        int current = current_cost.index;
        queue.pop();

        close_list.insert(current); //visited

        if(current == goal) //arrive goal
        {
            reconstructPath(vertex, start, goal, parent);    // reconstruct path
            return;
        }

        neightbor = prm_graph_.getNeightbor(current);
        for(int next: neightbor)
        {
            if(close_list.find(next) != close_list.end())   //visited
            {
                continue;
            }

            //claculate the new cost
            float new_cost = open_list.at(current) +
                    heuristic_function(vertex[current].pos, vertex[next].pos);
            
            if(open_list.find(next) == open_list.end()
                    || new_cost < open_list.at(next))
            {
                open_list[next] = new_cost;
                queue.push(PathCost(next, new_cost));
                parent[next] = current;
            }
        }
    }

    path_.clear();  //empty path
}

/** 
@brief PRM::AStar
@param distance_first: true for distance first, false for energy first
@desc A star algorithm, the difference between A star and Dijkstra is that A star has a heuristic weight
*/
void PRM::AStar(Strategy strategy)
{
    auto heuristic_function = selectHeuristic(strategy);

    const QVector<Graph::Vertex> vertex = prm_graph_.getVertex();

    int start = prm_graph_.getVertex(start_);
    int goal = prm_graph_.getVertex(goal_);

    std::priority_queue<PathCost, QVector<PathCost>, std::greater<PathCost>> queue;  //gobal distance
    std::unordered_map<int, int> parent;  //the parent node of path node
    std::unordered_map<int, float> open_list; //current actual distance
    std::set<int> close_list;
    QVector<int> neightbor;

    queue.push(PathCost(start, 0));  //push start point into queue
    parent[start] = -1;   // -1 stand for no parent node
    open_list[start] = 0;

    while(!queue.empty())
    {
        PathCost current_cost = queue.top();
        int current = current_cost.index;
        queue.pop();

        close_list.insert(current); //visited

        if(current == goal) //arrive goal
        {
            reconstructPath(vertex, start, goal, parent);    // reconstruct path
            return;
        }

        neightbor = prm_graph_.getNeightbor(current);

        for(int next: neightbor)
        {
            if(close_list.find(next) != close_list.end())   //visited
            {
                continue;
            }

            //claculate the new cost
            //When two points are adjacent, the heuristic weight is the actual weight
            float new_cost = open_list.at(current) +
                    heuristic_function(vertex[current].pos, vertex[next].pos);

            // update the cost of next node when it is not in open list or the new cost is smaller
            if(open_list.find(next) == open_list.end()
                    || new_cost < open_list.at(next))
            {
                open_list[next] = new_cost;
                //f(n) = g(n) + h(n)
                //g(n) is the actual distance from start to current
                //h(n) is the heuristic weight from current to goal
                new_cost += heuristic_function(vertex[next].pos, vertex[goal].pos);
                queue.push(PathCost(next, new_cost));
                parent[next] = current;
            }
        }
    }

    path_.clear();  //empty path
}

bool PRM::checkPath(const QPoint &point1, const QPoint &point2)
{
    QPoint point1_ = transposePoint(point1);
    QPoint point2_ = transposePoint(point2);

    float num = 10 * std::max(abs(point1_.x() - point2_.x()), abs(point1_.y() - point2_.y()));

    //the same path
    if(num == 0)
    {
        return false;
    }

    float x_step = ((float)(point2_.x() - point1_.x())/num);
    float y_step = ((float)(point2_.y() - point1_.y())/num);

    QVector<QPoint> vec;

    for(int i=0; i<=num; i++)
    {
        vec.push_back(QPoint(std::round(point1_.x() + i*x_step),
                             std::round(point1_.y() + i*y_step)));
    }

    for(QPoint point: vec)
    {
        if(graph_mat_[point.x()][point.y()] == 1)
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief PRM::calHeuristicDistance
 * @param point1
 * @param point2
 * @return the euclidean distance between two points
 */
float PRM::calHeuristicDistance(const QPoint &point1, const QPoint &point2)
{
    return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
}

/**
 * @brief PRM::calHeuristicEnergy
 * @param point1
 * @param point2
 * @return the energy cost between two points
 */
float PRM::calHeuristicEnergy(const QPoint &point1, const QPoint &point2)
{
    float energy = 0.0f;

    QPoint point1_ = transposePoint(point1);
    QPoint point2_ = transposePoint(point2);

    int num = 30 * std::max(abs(point1_.x() - point2_.x()), abs(point1_.y() - point2_.y()));

    if(num == 0)
    {
        return 0.0f;
    }

    float x_step = ((float)(point2_.x() - point1_.x())/num);
    float y_step = ((float)(point2_.y() - point1_.y())/num);

    QVector<QPoint> vec;

    for(int i=0; i<=num; i++)
    {
        vec.push_back(QPoint(std::round(point1_.x() + i*x_step),
                             std::round(point1_.y() + i*y_step)));
    }

    QVector<QPoint>::const_iterator it = vec.cbegin();
    for(; it != vec.cend(); it++)
    {
        if(graph_mat_[it->x()][it->y()] == 2)
        {
            energy += SAND_WEIGHT;
        }
        else
        {
            energy += ROAD_WEIGHT;
        }
    }

    return energy;
}

QPoint PRM::transposePoint(const QPoint &point)
{
    QPoint res;

    res.setX(point.y());
    res.setY(point.x());

    return res;
}

bool PRM::isCrash(const int **mat, int row, int col, const QPoint &point)
{
    for(int i= point.x() - 1; i <= point.x() + 1; i++)
    {
        for(int j= point.y() - 1; j <= point.y() + 1; j++)
        {
            if(i >= 0 && i < row && j >= 0 && j <col)
            {
              if(mat[i][j] == 1)
              {
                  return true;
              }
            }
        }
    }

    return false;
}

void PRM::reconstructPath(const QVector<Graph::Vertex>& vertex, int start, int goal,
                          const std::unordered_map<int, int> &close_list)
{
    path_.clear();

    path_.push_front(QPoint(vertex[goal].pos));
    int index = close_list.at(goal);

    while(index != start)
    {
        path_.push_front(QPoint(vertex[index].pos));
        index = close_list.at(index);
    }

    path_.push_front(QPoint(vertex[start].pos));
}
