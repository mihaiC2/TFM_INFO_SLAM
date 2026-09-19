/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include"../../../include/System.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
	ros::Publisher pose_pub;
	ros::Publisher map_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings use_viewer(0|1)" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, (strcmp(argv[3], "1") == 0));

	ImageGrabber igb(&SLAM);
	
	ros::NodeHandle nodeHandler;
    igb.pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/orb_slam3/pose", 1);
	igb.map_pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/orb_slam3/map_points", 1);
	
	ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

	// Publicar solo si la posición es válida
    if(!Tcw.translation().hasNaN())
    {
        #pragma region Publicar pose
        // Pose de la cámara respecto al mundo
        Sophus::SE3f Twc = Tcw.inverse();

        // Mensaje para ROS
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = cv_ptr->header.stamp;
        pose_msg.header.frame_id = "map";

        pose_msg.pose.position.x = Twc.translation().x();
        pose_msg.pose.position.y = Twc.translation().y();
        pose_msg.pose.position.z = Twc.translation().z();

        // Rotación en formato Cuaternión (Obligatorio en ROS)
        pose_msg.pose.orientation.x = Twc.unit_quaternion().x();
        pose_msg.pose.orientation.y = Twc.unit_quaternion().y();
        pose_msg.pose.orientation.z = Twc.unit_quaternion().z();
        pose_msg.pose.orientation.w = Twc.unit_quaternion().w();

        pose_pub.publish(pose_msg);
        #pragma endregion
    }

    #pragma region Publicar nube de puntos de TODO el mapa
    // Temporizador: 1 mapa/s
    static double last_cloud_pub_time = 0.0;
    double current_time = cv_ptr->header.stamp.toSec();

    if(current_time - last_cloud_pub_time >= 1.0) 
    {
        // Extraemos TODOS los puntos del mapa (Requiere GetAllMapPoints())
        vector<ORB_SLAM3::MapPoint*> vpMPs = mpSLAM->GetAllMapPoints();

        if(!vpMPs.empty())
        {
            sensor_msgs::PointCloud2 cloud;
            cloud.header.stamp = cv_ptr->header.stamp;
            cloud.header.frame_id = "map";

            cloud.height = 1;
            cloud.is_dense = false;
            cloud.is_bigendian = false;

            sensor_msgs::PointCloud2Modifier modifier(cloud);
            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(vpMPs.size()); // Reservamos el máximo temporalmente

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

            int puntos_validos = 0;

            for(size_t i = 0; i < vpMPs.size(); ++i)
            {
                ORB_SLAM3::MapPoint* pMP = vpMPs[i];
                
                // Si el punto es malo, saltamos sin sumar los iteradores
                if(!pMP || pMP->isBad()) 
                    continue;

                Eigen::Vector3f pos = pMP->GetWorldPos();

                *iter_x = pos(0);
                *iter_y = pos(1);
                *iter_z = pos(2);

                // Solo avanzamos la memoria si hemos escrito un punto válido
                ++iter_x; 
                ++iter_y; 
                ++iter_z;
                puntos_validos++;
            }

            // Recortamos el tamaño del mensaje a los puntos reales
            cloud.width = puntos_validos;
            cloud.row_step = cloud.width * cloud.point_step;
            cloud.data.resize(cloud.row_step);

            map_pub.publish(cloud);
        }
        
        // Actualizamos el tiempo de la última publicación
        last_cloud_pub_time = current_time;
    }
    #pragma endregion
}