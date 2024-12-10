#include "kdtree.h"
#include <algorithm>
#include <qmath.h>
#include <stack>

KdTree::KdTree()
{
    KdTree(1);
}

KdTree::KdTree(unsigned int k)
{
    this->k_ = k;
    this->root_ = nullptr;
}

KdTree::~KdTree()
{
    destroyTree(root_);
}

void KdTree::initKdTree(const QVector<QPoint> &points)
{
    QVector<KdNode*> nodes;

    //initialize node vector
    QVector<QPoint>::const_iterator it = points.cbegin();
    int index = 0;
    for(; it!=points.cend(); it++)
    {
        KdNode *temp = new KdNode;
        temp->pos.setX(it->x());
        temp->pos.setY(it->y());
        temp->index = index;
        temp->left = nullptr;
        temp->right = nullptr;
        nodes.push_back(temp);

        index++;
    }

    //construct kd tree
    int axis = X_Axis;  //division axis
    constructKdTree(nodes, &root_, 0, nodes.size(), axis);
}

void KdTree::setKValue(int k)
{
    this->k_ = k;
}

void KdTree::addNode(const KdTree::KdNode *parent, const KdTree::KdNode *child)
{

}

void KdTree::addNode(const KdTree::KdNode *parent, const QPoint &pos)
{

}

void KdTree::deleteNode(const KdTree::KdNode *node)
{

}

void KdTree::deleteNode(const QPoint &pos)
{

}

const KdTree::KdNode *KdTree::getNode(const KdTree::KdNode *node) const
{

}

const KdTree::KdNode *KdTree::getNode(const QPoint &pos) const
{

}

QVector<int> KdTree::getKNN(const QPoint &target)
{
    if(root_ == nullptr)
    {
        return QVector<int>();
    }

    QVector<int> vec;
    KdTree::NeightborQueue queue;
    std::stack<KdNode*> stack;

    KdNode* temp;
    KdNode* node = root_;

    while(node)
    {
        stack.push(node);   //push stack
        node = findNextNode(node, target);  //find leaf node
    }

    while(!stack.empty())
    {
        node = stack.top();
        stack.pop();

        Neightbor neightbor(node->index, getDistance(node->pos, target));
        pushQueue(queue, neightbor);

        if(queue.size() < this->k_)
        {
            temp = node;
            // find another sub tree
            if(node->division_axis == X_Axis)
            {
                if(target.x() < node->pos.x())
                {
                    node = node->right;
                }
                else if(target.x() > node->pos.x())
                {
                    node = node->left;
                }
            }
            else
            {
                if(target.y() < node->pos.y())
                {
                    node = node->right;
                }
                else if(target.y() > node->pos.y())
                {
                    node = node->left;
                }
            }

            if(temp != node)
            {
                while(node)
                {
                    stack.push(node);   //push stack
                    node = findNextNode(node, target);  //find leaf node
                }
            }
        }
        else
        {
            // search another sub tree
            if(queue.top().distance > getDistance(target, node))
            {
                temp = node;
                // find another sub tree
                if(node->division_axis == X_Axis)
                {
                    if(target.x() < node->pos.x())
                    {
                        node = node->right;
                    }
                    else if(target.x() > node->pos.x())
                    {
                        node = node->left;
                    }
                }
                else
                {
                    if(target.y() < node->pos.y())
                    {
                        node = node->right;
                    }
                    else if(target.y() > node->pos.y())
                    {
                        node = node->left;
                    }
                }

                if(temp != node)
                {
                    while(node)
                    {
                        stack.push(node);   //push stack
                        node = findNextNode(node, target);  //find leaf node
                    }
                }
            }
        }
    }

    while (!queue.empty())
    {
        Neightbor neightbor =  queue.top();
        queue.pop();
        vec.push_back(neightbor.index);
    }

    return vec;
}

void KdTree::constructKdTree(QVector<KdTree::KdNode *> nodes,
                             KdNode** root, int left, int right, int axis)
{
    if(left >= right)
    {
        root = nullptr;
        return;
    }

    //sort, get the mid elem
    int mid = (left+right)>>1;

    if(axis == X_Axis)
    {
        std::sort(nodes.begin()+left, nodes.begin()+right,
                  [](const KdNode* node1, const KdNode* node2)->bool
        {return node1->pos.x() < node2->pos.x(); });
    }
    else
    {
        std::sort(nodes.begin()+left, nodes.begin()+right,
                  [](const KdNode* node1, const KdNode* node2)->bool
        {return node1->pos.y() < node2->pos.y(); } );
    }

    nodes[mid]->division_axis = axis;   //set division axis
    *root = nodes[mid];

    constructKdTree(nodes, &((*root)->left), left, mid, axis^1);
    constructKdTree(nodes, &((*root)->right), mid+1, right, axis^1);
}

void KdTree::destroyTree(KdTree::KdNode *node)
{
    if(node == nullptr)
    {
        return;
    }

    destroyTree(node->left);
    destroyTree(node->right);

    delete node;
    node = nullptr;
}

void KdTree::KNNSearch(KdTree::KdNode *node, const QPoint &point,
                       QVector<Neightbor> &vec, int k)
{
    if(node == nullptr)
    {
        return;
    }

    if(node->division_axis == X_Axis)
    {
        if(point.x() > node->pos.x())
        {
            KNNSearch(node->right, point, vec, k);
        }
        else
        {
            KNNSearch(node->left, point, vec, k);
        }
    }
    else
    {
        if(point.y() > node->pos.y())
        {
            KNNSearch(node->right, point, vec, k);
        }
        else
        {
            KNNSearch(node->left, point, vec, k);
        }
    }

    if(vec.size() < (int)this->k_)
    {
        vec.push_back(Neightbor(node->index, getDistance(point, node->pos)));
    }
    else
    {
        float distance = getDistance(point, node->pos);

        int index = 0;
        float max = 0.0f;

        for(int i=0; i<vec.size();i++)
        {
            if(vec[i].distance > max)
            {
                index = i;
                max = vec[i].distance;
            }
        }

        if(distance < max)
        {
            vec[index].index = node->index;
            vec[index].distance = distance;
        }
    }
}

float KdTree::getDistance(const QPoint &point1, const QPoint &point2)
{
    return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
}

float KdTree::getDistance(const QPoint &point, const KdTree::KdNode *node)
{
    if(node->division_axis == X_Axis)
    {
        return getDistance(point, QPoint(node->pos.x(), point.y()));
    }
    else
    {
        return getDistance(point, QPoint(point.x(), node->pos.y()));
    }
}

void KdTree::pushQueue(KdTree::NeightborQueue &queue, KdTree::Neightbor &neightbor)
{
    // the queue is not full
    if(queue.size() < this->k_)
    {
        queue.push(neightbor);
    }
    else
    {
        if(neightbor.distance < queue.top().distance)
        {
            queue.pop();
            queue.push(neightbor);
        }
    }
}

KdTree::KdNode *KdTree::findNextNode(KdTree::KdNode *node, const QPoint& point)
{
    if(node->division_axis == X_Axis)
    {
        if(point.x() > node->pos.x())
        {
            return node->right;
        }
        else
        {
            return node->left;
        }
    }
    else
    {
        if(point.y() > node->pos.y())
        {
            return node->right;
        }
        else
        {
            return node->left;
        }
    }
}

bool KdTree::XAxisCompare(const KdTree::KdNode *node1, const KdTree::KdNode *node2)
{
    return node1->pos.x() < node2->pos.x();
}

bool KdTree::YAxisCompare(const KdTree::KdNode *node1, const KdTree::KdNode *node2)
{
    return node1->pos.y() < node2->pos.y();
}


