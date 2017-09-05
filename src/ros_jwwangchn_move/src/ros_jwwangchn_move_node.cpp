#include <cstdlib>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>

using namespace std;

int altitudeTakeoff = 5; //起飞高度
int altitudeLand = 5;    //降落高度

bool m_init_local_pose_check = true;
int m_waypoint_count;

bool m_verbal_flag;

// Waypoint Points
int m_num_waypoint;
vector<double> m_x_pos;
vector<double> m_y_pos;
vector<double> m_z_pos;

mavros_msgs::State m_current_state;
geometry_msgs::PoseStamped m_current_pose;
vector<geometry_msgs::PoseStamped> m_waypoint_pose;

ros::Publisher m_local_pos_pub;
// 获取watpoint数据
bool get_waypoint()
{
    if (ros::param::get("offb_node/num_waypoint", m_num_waypoint))
    {
        ROS_INFO("Find num_waypoint");
    }
    else
    {
        ROS_WARN("Didn't find num_waypoint");
    }
    if (ros::param::get("offb_node/x_pos", m_x_pos))
    {
        ROS_INFO("Find x_pos");
    }
    else
    {
        ROS_WARN("Didn't find x_pos");
    }
    if (ros::param::get("offb_node/y_pos", m_y_pos))
    {
        ROS_INFO("Find y_pos");
    }
    else
    {
        ROS_WARN("Didn't find y_pos");
    }
    if (ros::param::get("offb_node/z_pos", m_z_pos))
    {
        ROS_INFO("Find z_pos");
    }
    else
    {
        ROS_WARN("Didn't find z_pos");
    }

    //Safety check in case wrong waypoints are fed.
    if (m_x_pos.size() != m_num_waypoint)
    {
        ROS_WARN("Wrong x_pos values.");
        return 0;
    }
    if (m_y_pos.size() != m_num_waypoint)
    {
        ROS_WARN("Wrong y_pos values.");
        return 0;
    }
    if (m_z_pos.size() != m_num_waypoint)
    {
        ROS_WARN("Wrong z_pos values.");
        return 0;
    }
    return 1;
}

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    m_current_state = *msg;
}

void cur_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    m_current_pose = *msg;
}

void publish_waypoint()
{
    if (!m_init_local_pose_check)
    {
        m_local_pos_pub.publish(m_waypoint_pose[m_waypoint_count]);
        if (m_verbal_flag)
        {
            double dist = sqrt(
                (m_current_pose.pose.position.x - m_waypoint_pose[m_waypoint_count].pose.position.x) *
                    (m_current_pose.pose.position.x - m_waypoint_pose[m_waypoint_count].pose.position.x) +
                (m_current_pose.pose.position.y - m_waypoint_pose[m_waypoint_count].pose.position.y) *
                    (m_current_pose.pose.position.y - m_waypoint_pose[m_waypoint_count].pose.position.y) +
                (m_current_pose.pose.position.z - m_waypoint_pose[m_waypoint_count].pose.position.z) *
                    (m_current_pose.pose.position.z - m_waypoint_pose[m_waypoint_count].pose.position.z));
            ROS_INFO("distance: %.2f", dist);
        }

        if (abs(m_current_pose.pose.position.x - m_waypoint_pose[m_waypoint_count].pose.position.x) < 0.5 &&
            abs(m_current_pose.pose.position.y - m_waypoint_pose[m_waypoint_count].pose.position.y) < 0.5 &&
            abs(m_current_pose.pose.position.z - m_waypoint_pose[m_waypoint_count].pose.position.z) < 0.5)
        {

            m_waypoint_count += 1;

            if (m_waypoint_count >= m_num_waypoint)
            {
                m_waypoint_count = m_waypoint_count - 1;
            }

            // ROS_INFO("m_waypoint_count = %d, cur_pos = (%.2f, %.2f, %.2f), next_pos = (%.2f, %.2f, %.2f)", m_waypoint_count,
            //     m_current_pose.pose.position.x, m_current_pose.pose.position.y, m_current_pose.pose.position.z,
            //     m_waypoint_pose[m_waypoint_count].pose.position.x, m_waypoint_pose[m_waypoint_count].pose.position.y, m_waypoint_pose[m_waypoint_count].pose.position.z);
        }
    }
}

void init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (m_init_local_pose_check)
    {
        for (int i = 0; i < m_num_waypoint; i++)
        {
            geometry_msgs::PoseStamped temp_target_pose;

            temp_target_pose.pose.position.x = msg->pose.position.x + m_x_pos[i];
            temp_target_pose.pose.position.y = msg->pose.position.y + m_y_pos[i];
            temp_target_pose.pose.position.z = msg->pose.position.z + m_z_pos[i];

            m_waypoint_pose.push_back(temp_target_pose);
        }

        m_init_local_pose_check = false;
    }

    publish_waypoint();

    ros::Rate rate(20.0);
    rate.sleep();
}

int main(int argc, char **argv)
{
    int rateNum = 10;

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle n;

    // Subscriber

    ros::Subscriber m_state_sub = n.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber m_local_pos_sub = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, cur_pose_cb);
    ros::Subscriber m_init_local_pos_sub = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, init_pose_cb);

    // Publisher
    m_local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    n.getParam("verbal_flag", m_verbal_flag);

    //  设定速率
    ros::Rate rate(rateNum);

    ////////////////////////////////////////////
    /////////////////GUIDED MODE/////////////////////
    ////////////////////////////////////////////
    // 修改模式
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "ALT_HOLD";
    if (cl.call(srv_setMode))
    {
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.success);
    }
    else
    {
        ROS_ERROR("Failed SetMode");
        return -1;
    }
    sleep(1);
    ////////////////////////////////////////////
    ///////////////////ARM//////////////////////
    ////////////////////////////////////////////
    // 解锁
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if (arming_cl.call(srv))
    {
        ROS_INFO("ARM send ok %d", srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed arming or disarming");
    }

    ////////////////////////////////////////////
    /////////////////TAKEOFF////////////////////
    ////////////////////////////////////////////
    // 起飞
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = altitudeTakeoff;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if (takeoff_cl.call(srv_takeoff))
    {
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
    }
    else
    {
        ROS_ERROR("Failed Takeoff");
    }

    ////////////////////////////////////////////
    /////////////////DO STUFF///////////////////
    ////////////////////////////////////////////
    // 执行飞行任务
    ROS_INFO("Start mission");
    sleep(5);
    while (n.ok())
    {
        m_local_pos_pub.publish(m_waypoint_pose[m_waypoint_count]);
        if (m_verbal_flag)
        {
            double dist = sqrt(
                (m_current_pose.pose.position.x - m_waypoint_pose[m_waypoint_count].pose.position.x) *
                    (m_current_pose.pose.position.x - m_waypoint_pose[m_waypoint_count].pose.position.x) +
                (m_current_pose.pose.position.y - m_waypoint_pose[m_waypoint_count].pose.position.y) *
                    (m_current_pose.pose.position.y - m_waypoint_pose[m_waypoint_count].pose.position.y) +
                (m_current_pose.pose.position.z - m_waypoint_pose[m_waypoint_count].pose.position.z) *
                    (m_current_pose.pose.position.z - m_waypoint_pose[m_waypoint_count].pose.position.z));
            ROS_INFO("distance: %.2f", dist);
        }

        if (abs(m_current_pose.pose.position.x - m_waypoint_pose[m_waypoint_count].pose.position.x) < 0.5 &&
            abs(m_current_pose.pose.position.y - m_waypoint_pose[m_waypoint_count].pose.position.y) < 0.5 &&
            abs(m_current_pose.pose.position.z - m_waypoint_pose[m_waypoint_count].pose.position.z) < 0.5)
        {

            m_waypoint_count += 1;

            // if (m_waypoint_count >= m_num_waypoint)
            // {
            //     m_waypoint_count = m_waypoint_count - 1;
            // }

            // ROS_INFO("m_waypoint_count = %d, cur_pos = (%.2f, %.2f, %.2f), next_pos = (%.2f, %.2f, %.2f)", m_waypoint_count,
            //     m_current_pose.pose.position.x, m_current_pose.pose.position.y, m_current_pose.pose.position.z,
            //     m_waypoint_pose[m_waypoint_count].pose.position.x, m_waypoint_pose[m_waypoint_count].pose.position.y, m_waypoint_pose[m_waypoint_count].pose.position.z);
        }
        if(m_waypoint_count >= m_num_waypoint)
        {
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    ////////////////////////////////////////////
    ///////////////////LAND/////////////////////
    ////////////////////////////////////////////
    // 降落
    ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = altitudeLand;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if (land_cl.call(srv_land))
    {
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }
    else
    {
        ROS_ERROR("Failed Land");
    }


    return 0;
}
