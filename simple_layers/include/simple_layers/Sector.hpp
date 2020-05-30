#ifndef SECTOR_HPP
#define SECTOR_HPP

#include <array>
#include <geometry_msgs/Pose.h>

struct Point
{
    Point() : 
        m_xInMeter(0), 
        m_yInMeter(0),
        m_zInMeter(0)
    {
    }
    /**
     * @brief Construct a new Point object
     * 
     * @param a_xInMeter x position in meters from origin
     * @param a_yInMeter y position in meters from origin
     */
    Point(double a_xInMeter, double a_yInMeter) : 
        m_xInMeter(a_xInMeter), 
        m_yInMeter(a_yInMeter),
        m_zInMeter(0)
    {
    }
    /**
     * @brief Construct a new Point object
     * 
     * @param a_xInMeter x position in meters from origin
     * @param a_yInMeter y position in meters from origin
     * @param a_zInMeter z position in meters from origin
     */
    Point(double a_xInMeter, double a_yInMeter, double a_zInMeter) :
        m_xInMeter(a_xInMeter), 
        m_yInMeter(a_yInMeter),
        m_zInMeter(a_zInMeter)
    {
    }

    ~Point()
    {
    }
    
    double m_xInMeter;
    double m_yInMeter;
    double m_zInMeter;
};

typedef std::array<Point, 2> Wall;

struct Sector
{
    Wall a; 
    Wall b; 

    Wall& getLeftFrom(const geometry_msgs::Pose& pose)
    {
        double dxa1 = a.at(0).m_xInMeter - pose.position.x;
        double dya1 = a.at(0).m_yInMeter - pose.position.y;
        double dxa2 = a.at(1).m_xInMeter - pose.position.x;
        double dya2 = a.at(1).m_yInMeter - pose.position.y;

        double dxb1 = b.at(0).m_xInMeter - pose.position.x;
        double dyb1 = b.at(0).m_yInMeter - pose.position.y;
        double dxb2 = b.at(1).m_xInMeter - pose.position.x;
        double dyb2 = b.at(1).m_yInMeter - pose.position.y;
        if((dxa1 < 0 && dxa2 < 0) || (dya1 < 0 && dya2 < 0))
        {
            return a;
        }
        return b;
    }

    Wall& getRightFrom(const geometry_msgs::Pose& pose)
    {
        double dxa1 = a.at(0).m_xInMeter - pose.position.x;
        double dya1 = a.at(0).m_yInMeter - pose.position.y;
        double dxa2 = a.at(1).m_xInMeter - pose.position.x;
        double dya2 = a.at(1).m_yInMeter - pose.position.y;

        double dxb1 = b.at(0).m_xInMeter - pose.position.x;
        double dyb1 = b.at(0).m_yInMeter - pose.position.y;
        double dxb2 = b.at(1).m_xInMeter - pose.position.x;
        double dyb2 = b.at(1).m_yInMeter - pose.position.y;
        if((dxa1 > 0 && dxa2 > 0) || (dya1 > 0 && dya2 > 0))
        {
            return a;
        }
        return b;
    }
};

#endif
