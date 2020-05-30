#include <simple_layers/wall_follow_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(object_layer::WallFollowLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace object_layer
{

    WallFollowLayer::WallFollowLayer() {}

    void WallFollowLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_);
        current_ = true;

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
            &WallFollowLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        robotPoseSub = nh.subscribe("/amcl_pose", 100, &WallFollowLayer::poseCallback, this);
        sideSub = nh.subscribe("/side", 100, &WallFollowLayer::sideCallback, this);
    }


    void WallFollowLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
    }

    void WallFollowLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                            double* min_y, double* max_x, double* max_y)
    {
        if (!enabled_)
        {
            return;
        }
        mark_x_ = robot_x + cos(robot_yaw);
        mark_y_ = robot_y + sin(robot_yaw);

        *min_x = std::min(*min_x, mark_x_);
        *min_y = std::min(*min_y, mark_y_);
        *max_x = std::max(*max_x, mark_x_);
        *max_y = std::max(*max_y, mark_y_);
    }

    void WallFollowLayer::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose)
    {
        robotPose = pose.pose.pose;
    }

    void WallFollowLayer::sideCallback(const std_msgs::Bool& side)
    {
        this->side = side.data;
    }

    void WallFollowLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                            int max_j)
    {
        if (!enabled_)
        {
            return;
        }
        unsigned int mx;
        unsigned int my;
        if(master_grid.worldToMap(mark_x_, mark_y_, mx, my))
        {
            Wall left = {Point(1.0, 1.0), Point(0.0, 1.0)};
            Wall right = {Point(-1.0, -1.0), Point(0.0, -1.0)};
            
            if(side)
            {
                double distance = right.at(0).m_xInMeter - left.at(0).m_xInMeter;
                for(std::size_t at = round(right.at(0).xInMeter); at != round(distance*0.8); ++at)
                {
                }
                
            } else 
            {

            }
        }
    }

} // end namespace