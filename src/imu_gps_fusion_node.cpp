#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <thread>
#include <mutex>
#include "imu_gps_fusion.h"

using namespace std;

nav_msgs::Path path; //result path

vector<sensor_msgs::ImuConstPtr> imu_buffer;
vector<sensor_msgs::NavSatFixConstPtr> gps_buffer;
mutex msg_mtx;

Fusion::ImuGpsFusion imu_gps_fuser;
ros::Publisher traj_puber;
double last_gps_time = -1.0;
Fusion::ImuData<double> last_gps_imu; //last interpolated imu data at gps time

void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    unique_lock<mutex> lock(msg_mtx);
    imu_buffer.push_back(msg);
}

void gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    unique_lock<mutex> lock(msg_mtx);
    gps_buffer.push_back(msg);
}

void interpolateImuData(const sensor_msgs::ImuConstPtr &first_data, const sensor_msgs::ImuConstPtr &second_data, double cur_stamp, sensor_msgs::Imu &inter_data)
{
    double first_stamp = first_data->header.stamp.toSec();
    double second_stamp = second_data->header.stamp.toSec();
    double scale = (cur_stamp - first_stamp) / (second_stamp - first_stamp);
    inter_data = *first_data;
    inter_data.angular_velocity.x = scale * (second_data->angular_velocity.x - first_data->angular_velocity.x) + first_data->angular_velocity.x;
    inter_data.angular_velocity.y = scale * (second_data->angular_velocity.x - first_data->angular_velocity.x) + first_data->angular_velocity.x;
    inter_data.angular_velocity.z = scale * (second_data->angular_velocity.x - first_data->angular_velocity.x) + first_data->angular_velocity.x;
    inter_data.linear_acceleration.x = scale * (second_data->linear_acceleration.x - first_data->linear_acceleration.x) + first_data->linear_acceleration.x;
    inter_data.linear_acceleration.y = scale * (second_data->linear_acceleration.y - first_data->linear_acceleration.y) + first_data->linear_acceleration.y;
    inter_data.linear_acceleration.z = scale * (second_data->linear_acceleration.z - first_data->linear_acceleration.z) + first_data->linear_acceleration.z;
}

Fusion::ImuData<double> fromImuMsg(const sensor_msgs::Imu &msg)
{
    Fusion::ImuData<double> imu_data;
    imu_data.gyr[0] = msg.angular_velocity.x;
    imu_data.gyr[1] = msg.angular_velocity.y;
    imu_data.gyr[2] = msg.angular_velocity.z;
    imu_data.acc[0] = msg.linear_acceleration.x;
    imu_data.acc[1] = msg.linear_acceleration.y;
    imu_data.acc[2] = msg.linear_acceleration.z;

    return move(imu_data);
}

void processThread()
{
    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        unique_lock<mutex> lock(msg_mtx);

        if (!imu_buffer.size() || !gps_buffer.size())
        {
            ROS_INFO_THROTTLE(10, "wait for gps or imu msg ......");
            lock.unlock();
            loop_rate.sleep();
            continue;
        }

        // init correlative param
        if (last_gps_time < 0)
        {
            // set reference ll for convert ll to enu frame
            imu_gps_fuser.cfgRefGps(gps_buffer[0]->latitude, gps_buffer[0]->longitude, gps_buffer[0]->altitude);

            // interpolate first imu data at first gps time
            auto iter = imu_buffer.begin();
            for (; iter != imu_buffer.end(); iter++)
            {
                if ((*iter)->header.stamp > gps_buffer[0]->header.stamp)
                    break;
            }
            if (imu_buffer.begin() == iter || imu_buffer.end() == iter) //cant find imu data before or after gps data
            {
                if (imu_buffer.begin() == iter)
                    gps_buffer.erase(gps_buffer.begin()); //no imu data before first gps data, cant interpolate at gps stamp
                lock.unlock();
                loop_rate.sleep();
                continue;
            }
            sensor_msgs::Imu inter_imu;
            double cur_stamp = gps_buffer[0]->header.stamp.toSec();
            interpolateImuData(*(iter - 1), *iter, cur_stamp, inter_imu);

            //record last gps frame time and interpolated imu data
            last_gps_imu = fromImuMsg(inter_imu);
            last_gps_time = gps_buffer[0]->header.stamp.toSec();

            // delete old imu datas and gps data
            imu_buffer.erase(imu_buffer.begin(), iter);
            gps_buffer.erase(gps_buffer.begin());

            lock.unlock();
            loop_rate.sleep();
            continue;
        }

        // collect imu datas during two neighbor gps frames
        vector<pair<Fusion::ImuData<double>, double>> imu_datas(0);
        // step0: search first imu data after gps data
        auto iter = imu_buffer.begin();
        for (; iter != imu_buffer.end(); iter++)
        {
            if ((*iter)->header.stamp > gps_buffer[0]->header.stamp)
                break;
        }
        if (imu_buffer.end() == iter) // no imu data after first gps data, wait for new imu data
        {
            lock.unlock();
            loop_rate.sleep();
            continue;
        }
        // step1: interpolate imu data at gps time
        assert(imu_buffer.begin() != iter);
        sensor_msgs::Imu inter_imu;
        double cur_stamp = gps_buffer[0]->header.stamp.toSec();
        interpolateImuData(*(iter - 1), *iter, cur_stamp, inter_imu);
        // step2: add imu data during last gps and first imu
        pair<Fusion::ImuData<double>, double> first_data;
        first_data.first = last_gps_imu;
        first_data.second = imu_buffer[0]->header.stamp.toSec() - last_gps_time;
        imu_datas.push_back(move(first_data));
        // step3: add imu data during first imu and last imu before gps data
        if (distance(imu_buffer.begin(), iter) >= 2)
        {
            for (auto tmp_iter = imu_buffer.begin(); tmp_iter != iter - 1; tmp_iter++)
            {
                auto next_iter = tmp_iter + 1;
                pair<Fusion::ImuData<double>, double> tmp_data;
                tmp_data.first = fromImuMsg(*(*tmp_iter));
                tmp_data.second = ((*next_iter)->header.stamp - (*tmp_iter)->header.stamp).toSec();
                imu_datas.push_back(move(tmp_data));
            }
        }
        // step4: add imu data during gps and last imu before gps data
        pair<Fusion::ImuData<double>, double> last_data;
        last_data.first = fromImuMsg(*(*(iter - 1)));
        last_data.second = cur_stamp - (*(iter - 1))->header.stamp.toSec();
        imu_datas.push_back(move(last_data));

        // generate gps data
        Fusion::GpsData<double> gps_data;
        gps_data.data[0] = gps_buffer[0]->latitude;
        gps_data.data[1] = gps_buffer[0]->longitude;
        gps_data.data[2] = gps_buffer[0]->altitude;
        gps_data.cov.setIdentity();
        gps_data.cov(0, 0) = gps_buffer[0]->position_covariance[0];
        gps_data.cov(1, 1) = gps_buffer[0]->position_covariance[4];
        gps_data.cov(2, 2) = gps_buffer[0]->position_covariance[8];

        // update state (core)
        imu_gps_fuser.gpsUpdate(gps_data, imu_datas);

        // update last data
        sensor_msgs::Imu gps_imu_msg;
        interpolateImuData(*(iter - 1), *iter, cur_stamp, gps_imu_msg);
        last_gps_imu = fromImuMsg(gps_imu_msg);
        last_gps_time = cur_stamp;

        // get result pose, and publish
        if (traj_puber.getNumSubscribers() != 0)
        {
            Fusion::State<double> result = imu_gps_fuser.getState();
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.header.stamp = gps_buffer[0]->header.stamp;
            pose.pose.position.x = result.p[0];
            pose.pose.position.y = result.p[1];
            pose.pose.position.z = result.p[2];
            pose.pose.orientation.w = result.q.w();
            pose.pose.orientation.x = result.q.x();
            pose.pose.orientation.y = result.q.y();
            pose.pose.orientation.z = result.q.z();
            path.poses.push_back(pose);
            path.header = pose.header;
            traj_puber.publish(path);
        }

        // delete old data
        gps_buffer.erase(gps_buffer.begin());
        imu_buffer.erase(imu_buffer.begin(), iter);

        lock.unlock();
        loop_rate.sleep();
        continue;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_gps_fusion_node");
    ros::NodeHandle nh, ph("~");

    // load imu param and config fuser
    double sigma_an, sigma_wn, sigma_aw, sigma_ww;
    if (!ph.getParam("sigma_an", sigma_an) || !ph.getParam("sigma_wn", sigma_wn) || !ph.getParam("sigma_aw", sigma_aw) || !ph.getParam("sigma_ww", sigma_ww))
    {
        cout << "please config imu param !!!" << endl;
        return 0;
    }
    imu_gps_fuser.cfgImuVar(sigma_an, sigma_wn, sigma_aw, sigma_ww);

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1000, imuCallback);
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1000, gpsCallback);
    traj_puber = nh.advertise<nav_msgs::Path>("traj", 1);

    // start process
    thread process_thread(processThread);

    ros::spin();

    process_thread.join();

    return 0;
}