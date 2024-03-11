#ifndef STELLA_VPS_FEATURE_ORB_EXTRACTOR_NODE_H
#define STELLA_VPS_FEATURE_ORB_EXTRACTOR_NODE_H

#include "Stella_VPS/utils.hpp"

class orb_extractor_node {
public:
    //! Constructor
    orb_extractor_node() = default;

    //! Divide node to four child nodes
    std::array<orb_extractor_node, 4> divide_node();

    //! Size of area
    unsigned int size() const {
        return (pt_end_.x - pt_begin_.x) * (pt_end_.y - pt_begin_.y);
    }

    //! Keypoints which distributed into this node
    std::vector<cv::KeyPoint> keypts_;

    //! Begin and end of the allocated area on the image
    cv::Point2i pt_begin_, pt_end_;

    //! A iterator pointing to self, used for removal on list
    std::list<orb_extractor_node>::iterator iter_;
};

#endif