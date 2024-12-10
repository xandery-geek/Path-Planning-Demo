#ifndef KDTREE_H
#define KDTREE_H

#include <QPoint>
#include <QVector>
#include <queue>

//division axis

class KdTree
{
private:

    enum Axis
    {
       X_Axis = 0,
       Y_Axis
    };

    class KdNode
    {
    public:
        QPoint pos;
        int index;  //index in vector
        int division_axis;

        KdNode *left;
        KdNode *right;
    };

    class Neightbor
    {
    public:
        int index;
        float distance;

        Neightbor()
        {
            this->index = 0;
            this->distance = 0.0f;
        }

        Neightbor(int index, float distance)
        {
            this->index = index;
            this->distance = distance;
        }

        bool operator < (const Neightbor& neightbor) const
        {
            return this->distance < neightbor.distance;
        }

        bool operator > (const Neightbor& neightbor) const
        {
            return this->distance > neightbor.distance;
        }
    };

    typedef std::priority_queue<KdTree::Neightbor> NeightborQueue;

    unsigned int k_;
    KdNode *root_;

public:
    KdTree();
    KdTree(unsigned int k);

    ~KdTree();

    void initKdTree(const QVector<QPoint>& points);
    void setKValue(int k);
    void addNode(const KdNode* parent, const KdNode* child);
    void addNode(const KdNode* parent, const QPoint& pos);
    void deleteNode(const KdNode* node);
    void deleteNode(const QPoint& pos);
    const KdNode* getNode(const KdNode* node) const;
    const KdNode* getNode(const QPoint& pos) const;

    QVector<int> getKNN(const QPoint& target);   //get K nearest neightbors

private:
    void constructKdTree(QVector<KdNode*> nodes, KdNode **root, int left, int right, int axis);
    void destroyTree(KdNode *node);
    void KNNSearch(KdNode *node, const QPoint& point, QVector<Neightbor>& vec, int k);
    float getDistance(const QPoint& point1, const QPoint& point2);
    float getDistance(const QPoint &point, const KdNode* node);
    void pushQueue(NeightborQueue& queue, Neightbor& neightbor);
    KdNode* findNextNode(KdNode* node, const QPoint &point);

    static bool XAxisCompare(const KdNode* node1, const KdNode* node2);
    static bool YAxisCompare(const KdNode* node1, const KdNode* node2);
};

#endif // KDTREE_H


