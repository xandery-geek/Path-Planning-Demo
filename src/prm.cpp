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
    vertex_k_ = K_COEFFICIENT * row * col;

    QVector<QPoint> points;

    points.push_back(start_);   //add start point
    points.push_back(goal_);     //add end point


    int i=0, j=0;
    while(n)
    {
        i = random_gen_->bounded(row);
        j = random_gen_->bounded(col);

        if(mat[i][j] == 0)
        {
            points.push_back(QPoint(j, i));
            n--;
        }
    }

    prm_graph_.addVertex(points);   //set vertex table of graph

    this->generateArc(points);      //generate arc of graph
}

void PRM::generateArc(const QVector<QPoint>& points)
{
    if(prm_graph_.getVertex().size() == 0)
    {
        return;
    }

    //KNN

    //construct Kd tree
    if(kd_tree_ != nullptr)
    {
        delete kd_tree_;
    }

    kd_tree_ = new KdTree(vertex_k_);
    kd_tree_->initKdTree(points);

    QVector<int> neighbors;

    //search the K nearest neighbors
    QVector<QPoint>::const_iterator it = points.begin();
    int index = 0;
    float dis, oil;

    for(; it !=points.end(); it++)
    {
        neighbors.clear();
        neighbors = kd_tree_->getKNN(*it);

        for(int i=0; i<neighbors.size(); i++)
        {            
            if(checkPath(points[index], points[neighbors[i]]))
            {
                dis = getDistance(points[index], points[neighbors[i]]);
                oil = getOil(points[index], points[neighbors[i]]);

                prm_graph_.addArc(index, neighbors[i], dis, oil);     //add to arc table
                prm_graph_.addArc(neighbors[i], index, dis, oil);     //add to arc table
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

void PRM::searchPath(bool option)
{
    AStar(option);
}

const QVector<QPoint> &PRM::getPath() const
{
    return path_;
}

const Graph &PRM::getGraph() const
{
    return prm_graph_;
}

void PRM::AStar(bool option)
{
    //function pointer which point to the function of weight calculate
    float (PRM::*getWeight)(const QPoint& point1, const QPoint& point2);

    if(option)  //oil first
    {
        getWeight = &PRM::getOil;
    }
    else    //distance first
    {
        getWeight = &PRM::getDistance;
    }

    const QVector<Graph::Vertex> vertex = prm_graph_.getVertex();

    int start = prm_graph_.getVertex(start_);
    int goal = prm_graph_.getVertex(goal_);

    std::priority_queue<AStarCost, QVector<AStarCost>, std::greater<AStarCost>> queue;  //gobal distance
    std::unordered_map<int, int> parent;  //the parent node of path node
    std::unordered_map<int, float> open_list; //current actual distance
    std::set<int> close_list;
    QVector<int> neightbor;

    queue.push(AStarCost(start, 0));  //push start point into queue
    parent[start] = -1;   // -1 stand for no parent node
    open_list[start] = 0;

    while(!queue.empty())
    {
        AStarCost current_const = queue.top();
        int current = current_const.index;
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

            float new_const = open_list.at(current) +
                    (this->*getWeight)(vertex[current].pos, vertex[next].pos);   //G

            if(open_list.find(next) == open_list.end()
                    || new_const < open_list.at(next))
            {
                open_list[next] = new_const;
                new_const += (this->*getWeight)(vertex[next].pos, vertex[goal].pos); //F = G + H
                queue.push(AStarCost(next, new_const));
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

float PRM::getDistance(const QPoint &point1, const QPoint &point2)
{
    return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
}

float PRM::getOil(const QPoint &point1, const QPoint &point2)
{
    float oil = 0.0f;

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
            oil += SAND_WEIGHT;
        }
        else
        {
            oil += ROAD_WEIGHT;
        }
    }

    return oil;
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
