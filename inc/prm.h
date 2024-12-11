#ifndef PRM_H
#define PRM_H

#include <QVector>
#include <QPoint>
#include <QRandomGenerator>
#include <unordered_map>
#include "graph.h"
#include "kdtree.h"


class PathCost
{
public:
    int index;
    float cost;

    PathCost()
    {
        index = 0;
        cost = 0.0f;
    }

    PathCost(int index, float cost)
    {
        this->index = index;
        this->cost = cost;
    }

    bool operator <(const PathCost& obj) const
    {
        return this->cost < obj.cost;
    }

    bool operator >(const PathCost& obj) const
    {
        return this->cost > obj.cost;
    }
};

class PRM
{
private:
    const float VERTEX_COEFFICIENT = 0.4;
    const float K_COEFFICIENT = 0.02;
    const float ROAD_WEIGHT = 1;
    const float SAND_WEIGHT = 8;

    QPoint start_;
    QPoint goal_;
    QVector<QPoint> path_;

    Graph prm_graph_;
    KdTree *kd_tree_;
    int vertex_k_;  // the scale of nearest beightbor

    const int **graph_mat_; //the actual map data
    int graph_mat_row_;
    int graph_mat_col_;
    QString stragety_;

    QRandomGenerator *random_gen_;

public:
    PRM(const QString& stragety="AStar");
    ~PRM();

    void constructGraph(const int** mat, int row, int col);
    void generateArc(const QVector<QPoint>& points); //KNN
    void setStrategy(const QString& stragety);
    void setStartPoint(const QPoint& point);
    void setEndPoint(const QPoint& point);
    void setRandomSeed(int seed);
    void searchPath(bool distance_first);
    float calPathCost(const QVector<QPoint>& points, bool distance_first);

    const QVector<QPoint>& getPath() const;
    const Graph& getGraph() const;

private:
    void AStar(bool distance_first);
    void Dijkstra(bool distance_first);
    bool checkPath(const QPoint& point1, const QPoint& point2);
    float calHeuristicDistance(const QPoint& point1, const QPoint& point2);
    float calHeuristicEnergy(const QPoint& point1, const QPoint& point2);
    QPoint transposePoint(const QPoint& point);
    bool isCrash(const int** mat, int row, int col, const QPoint& point);
    void reconstructPath(const QVector<Graph::Vertex> &vertex, int start, int goal,
                         const std::unordered_map<int, int> &close_list);

    std::function<float(const QPoint&, const QPoint&)> selectHeuristic(bool distance_first) {
        if (distance_first) {
            return [this](const QPoint& p1, const QPoint& p2) { return calHeuristicDistance(p1, p2); };
        } else {
            return [this](const QPoint& p1, const QPoint& p2) { return calHeuristicEnergy(p1, p2); };
        }
    }
};

#endif // PRM_H
