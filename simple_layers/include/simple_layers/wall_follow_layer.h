#ifndef WALL_FOLLOW_LAYER_H
#define WALL_FOLLOW_LAYER_H
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <simple_layers/Sector.hpp>

namespace object_layer
{
    class WallFollowLayer : public costmap_2d::Layer
    {
    public:
        WallFollowLayer();

        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                                    double* max_y);
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

        void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
        void sideCallback(const std_msgs::Bool& side);

    private:
        void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

        double mark_x_, mark_y_;
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
        
        ros::Subscriber robotPoseSub;
        ros::Subscriber sideSub;
        geometry_msgs::Pose robotPose;
        Sector testSector;
        bool side;
    };
}
#endif