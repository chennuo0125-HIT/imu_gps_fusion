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

int imu_freq = 100; //imu frequency is location frequency

nav_msgs::Path path; //result path

vector<sensor_msgs::ImuConstPtr> imu_buffer;
vector<sensor_msgs::NavSatFixConstPtr> gps_buffer;
mutex imu_mtx, gps_mtx;

Fusion::ImuGpsFusion imu_gps_fuser; // fuser object
ros::Publisher traj_puber;
bool initialized = false;
Fusion::ImuData<double> last_gps_imu;     //last interpolated imu data at gps time
Fusion::ImuData<double> last_imu;         //last imu data for predict
Fusion::State<double> last_updated_state; //last updated state by gps

void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    unique_lock<mutex> lock(imu_mtx);
    imu_buffer.push_back(msg);
}

void gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    unique_lock<mutex> lock(gps_mtx);
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
    imu_data.stamp = msg.header.stamp.toSec();
    imu_data.gyr[0] = msg.angular_velocity.x;
    imu_data.gyr[1] = msg.angular_velocity.y;
    imu_data.gyr[2] = msg.angular_velocity.z;
    imu_data.acc[0] = msg.linear_acceleration.x;
    imu_data.acc[1] = msg.linear_acceleration.y;
    imu_data.acc[2] = msg.linear_acceleration.z;

    return move(imu_data);
}

Fusion::GpsData<double> fromGpsMsg(const sensor_msgs::NavSatFix &msg)
{
    Fusion::GpsData<double> gps_data;
    gps_data.data[0] = msg.latitude;
    gps_data.data[1] = msg.longitude;
    gps_data.data[2] = msg.altitude;
    gps_data.cov.setIdentity();
    gps_data.cov(0, 0) = msg.position_covariance[0];
    gps_data.cov(1, 1) = msg.position_covariance[4];
    gps_data.cov(2, 2) = msg.position_covariance[8];
    return move(gps_data);
}

void pubResult()
{
    if (traj_puber.getNumSubscribers() != 0)
    {
        Fusion::State<double> result = imu_gps_fuser.getNominalState();
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
}

void processThread()
{
    ros::Rate loop_rate(imu_freq);
    while (ros::ok())
    {
        unique_lock<mutex> imu_lock(imu_mtx);
        unique_lock<mutex> gps_lock(gps_mtx);

        // if no data, no need update state
        if (!imu_buffer.size() && !gps_buffer.size())
        {
            ROS_INFO_THROTTLE(10, "wait for gps or imu msg ......");
            imu_lock.unlock();
            gps_lock.unlock();
            loop_rate.sleep();
            continue;
        }

        // init correlative param
        if (!initialized)
        {
            // wait for enough sensor data to init
            if (!imu_buffer.size() || !gps_buffer.size())
            {
                ROS_INFO_THROTTLE(10, "wait for gps or imu msg ......");
                imu_lock.unlock();
                gps_lock.unlock();
                loop_rate.sleep();
                continue;
            }

            // use imu datas at start to initial imu pose
            vector<Fusion::ImuData<double>> imu_datas;
            for (auto &imu_msg : imu_buffer)
            {
                imu_datas.push_back(fromImuMsg(*imu_msg));
            }
            imu_gps_fuser.imuInit(imu_datas);

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
                imu_lock.unlock();
                gps_lock.unlock();
                loop_rate.sleep();
                continue;
            }
            sensor_msgs::Imu inter_imu;
            double cur_stamp = gps_buffer[0]->header.stamp.toSec();
            interpolateImuData(*(iter - 1), *iter, cur_stamp, inter_imu);

            //record last gps frame time and interpolated imu data
            last_gps_imu = fromImuMsg(inter_imu);
            last_gps_imu.stamp = cur_stamp;
            last_imu = last_gps_imu;
            last_updated_state = imu_gps_fuser.getState();

            // delete old imu datas and gps data
            imu_buffer.erase(imu_buffer.begin(), iter);
            gps_buffer.erase(gps_buffer.begin());

            imu_lock.unlock();
            gps_lock.unlock();
            loop_rate.sleep();

            initialized = true;

            continue;
        }

        // use imu predict location for increase locate frequency
        // actual state no change
        for (auto &imu_msg : imu_buffer)
        {
            Fusion::ImuData<double> cur_imu = fromImuMsg(*imu_msg);
            if (cur_imu.stamp > last_imu.stamp)
            {
                imu_gps_fuser.updateNominalState(last_imu, cur_imu);
                last_imu = cur_imu;
            }
        }

        // use gps data to update state
        if (gps_buffer.size() != 0)
        {
            if (gps_buffer.front()->status.status != 2)
            {
                cout << "gps data is bad !!!" << endl;
                gps_buffer.erase(gps_buffer.begin());
                imu_lock.unlock();
                gps_lock.unlock();
                loop_rate.sleep();
                continue;
            }

            // recover to last updated state for imu predict again
            imu_gps_fuser.recoverState(last_updated_state);

            // collect imu datas during two neighbor gps frames
            vector<Fusion::ImuData<double>> imu_datas(0);
            // search first imu data after gps data
            auto iter = imu_buffer.begin();
            for (; iter != imu_buffer.end(); iter++)
            {
                if ((*iter)->header.stamp > gps_buffer[0]->header.stamp)
                    break;
            }
            if (imu_buffer.end() == iter) // no imu data after first gps data, wait for new imu data
            {
                imu_lock.unlock();
                gps_lock.unlock();
                loop_rate.sleep();
                continue;
            }
            assert(imu_buffer.begin() != iter);
            // add last gps_imu data (interpolated data at gps time)
            imu_datas.push_back(last_gps_imu);
            // add imu data between last gps_imu data and current gps_imu data
            for (auto tmp_iter = imu_buffer.begin(); tmp_iter != iter; tmp_iter++)
                imu_datas.push_back(fromImuMsg(*(*tmp_iter)));
            // add current gps_imu data
            sensor_msgs::Imu inter_imu;
            double cur_stamp = gps_buffer[0]->header.stamp.toSec();
            interpolateImuData(*(iter - 1), *iter, cur_stamp, inter_imu);
            Fusion::ImuData<double> cur_gps_imu = fromImuMsg(inter_imu);
            cur_gps_imu.stamp = cur_stamp;
            imu_datas.push_back(cur_gps_imu);

            // generate gps data
            Fusion::GpsData<double> gps_data = fromGpsMsg(*gps_buffer[0]);

            // update state (core)
            imu_gps_fuser.gpsUpdate(gps_data, imu_datas);

            // update last data
            last_gps_imu = cur_gps_imu;
            last_imu = last_gps_imu;

            // delete old data
            gps_buffer.erase(gps_buffer.begin());
            imu_buffer.erase(imu_buffer.begin(), iter);

            // update last state
            last_updated_state = imu_gps_fuser.getState();
        }

        // publish result
        pubResult();

        imu_lock.unlock();
        gps_lock.unlock();
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

    // load imu freqency param
    if (!ph.getParam("imu_freq", imu_freq))
    {
        cout << "no config imu_freq param, use default 100 !" << endl;
    }

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1000, imuCallback);
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1000, gpsCallback);
    traj_puber = nh.advertise<nav_msgs::Path>("traj", 1);

    // start process
    thread process_thread(processThread);

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    process_thread.join();

    return 0;
}