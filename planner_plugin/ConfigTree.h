#ifndef CONFIG_TREE_H
#define CONFIG_TREE_H

#include <limits>

template <typename NodeType>
class ConfigTree
{
public:
    ConfigTree(NodeType){}

    void setRoot(std::shared_ptr<NodeType> root) {root_ = root;}

    std::shared_ptr<NodeType> getClosestNode(std::function<float(NodeType)> distance_to_target)
    {
        float min_dist = std::numeric_limits<float>::max();
        std::vector<std::shared_ptr<NodeType> > nodes_to_check;
        std::shared_ptr<NodeType> closest_node =  root_;
        nodes_to_check.push_back(closest_node);

        while(nodes_to_check.size())
        {
            float d = distance_to_target(*nodes_to_check.back());
            if(d < min_dist)
            {
                closest_node = nodes_to_check.back();
                min_dist = d;
            }
            for (auto c : nodes_to_check.back()->getChildren())
            {
                nodes_to_check.push_back(c);
            }
            nodes_to_check.pop_back();
        }
        return closest_node;
    }

private:
    std::shared_ptr<NodeType> root_;
};

#endif