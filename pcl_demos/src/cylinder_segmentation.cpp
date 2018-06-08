#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher pubz;
ros::Publisher pubm;

tf2_ros::Buffer tf2_buffer;

typedef pcl::PointXYZRGB PointT;

ros::Publisher marker_array;

visualization_msgs::MarkerArray markers;

int marker_cnt;

typedef struct {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
} rgb;

typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} hsv;

static hsv   rgb2hsv(rgb in);

hsv rgb2hsv(rgb in)
{
    hsv         out;
    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
    if( in.g >= max )
        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}


void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
    // voxel filtering

    pcl::PCLPointCloud2::Ptr cloud_filtered_voxel (new pcl::PCLPointCloud2 ());

    //std::cerr << "Pointcloud_blob before filtering: " << cloud_blob->width * cloud_blob->height 
    //    << " data points (" << pcl::getFieldsList (*cloud_blob) << ")."<< std::endl;;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_blob);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered_voxel);

    //std::cerr << "Pointcloud_blob after filtering: " << cloud_filtered_voxel->width * cloud_filtered_voxel->height 
    //   << " data points (" << pcl::getFieldsList (*cloud_filtered_voxel) << ")."<< std::endl;;

    //-------------------------------------------------------------------------------------------------------

    // All the objects needed
    pubx.publish(cloud_filtered_voxel);
    ros::Time time_rec, time_test;
    time_rec = ros::Time::now();

    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    Eigen::Vector4f centroid;

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered_depth (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    // Read in the cloud data
    pcl::fromPCLPointCloud2 (*cloud_filtered_voxel, *cloud);
    //std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.3, 1.5);
    pass.filter (*cloud_filtered_depth);
    //std::cerr << "PointCloud after depth filtering has: " << cloud_filtered_depth->points.size () << " data points." << std::endl;

    pass.setInputCloud (cloud_filtered_depth);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-0.25, 0.25);
    pass.filter (*cloud_filtered);
    //std::cerr << "PointCloud after height filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

    //pubz.publish(cloud_filtered);

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Write the planar inliers to disk
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_plane);
    //std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    pcl::PCLPointCloud2 outcloud_plane;
    pcl::toPCLPointCloud2 (*cloud_plane, outcloud_plane);
    pubx.publish (outcloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    //std::cerr << "callback" << std::endl;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (500);
    seg.setDistanceThreshold (0.005);
    seg.setRadiusLimits (0.10, 0.14);
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    //std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.empty ()) {
    std::cerr << "Can't find the cylindrical component." << std::endl;
    } else {
	  //std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
    
    if (cloud_cylinder->points.size() < 400) { 
        return; 
    }
    
    pcl::compute3DCentroid (*cloud_cylinder, centroid);
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
    //std::cerr << "centroid of the cylindrical component: " << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << std::endl;
    
    // barve -------------------------------------------------------------------------------------------------------------------------------------------
    // unpack rgb into r/g/b
    double r_sum  = 0;
    double g_sum  = 0;
    double b_sum  = 0;

    for(int i = 0; i < cloud_cylinder->points.size(); i++){
        uint32_t point_rgb = *reinterpret_cast<int*>(&cloud_cylinder->points[i].rgb);

        r_sum  += (double)((int)(point_rgb >> 16) & 0x0000ff) / 255;
        g_sum  += (double)((int) (point_rgb >> 8)  & 0x0000ff) / 255;
        b_sum  += (double)((int) (point_rgb)       & 0x0000ff) / 255;


    }

    r_sum /= cloud_cylinder->points.size();
    g_sum /= cloud_cylinder->points.size();
    b_sum /= cloud_cylinder->points.size();

    rgb rgb_avg = {r_sum, g_sum, b_sum};

    hsv hsv_avg = rgb2hsv(rgb_avg);

    double hue = hsv_avg.h / 2;
    double saturation = hsv_avg.s * 255;
    double value = hsv_avg.v * 255;

    std::cerr << "RGB komponenta: " << r_sum << " " << g_sum << " " <<  b_sum << std::endl;

    std::cerr << "HSV komponenta: " << hue << " " << saturation << " " <<  value << std::endl;

    double marker_barva[3] = {1.0f, 0.0f, 1.0f};

    if(hue > 150 || hue < 15) {
        std::cerr << "red"<< std::endl; // 174
        marker_barva[0] = 1.0f;
        marker_barva[1] = 0.0f;
        marker_barva[2] = 0.0f;
    }
    else if(hue > 40 && hue < 70){
        std::cerr << "green"<< std::endl; // 48
        marker_barva[0] = 0.0f;
        marker_barva[1] = 1.0f;
        marker_barva[2] = 0.0f;
    }
    else if(hue > 95 && hue < 140) { // 110
        std::cerr << "blue"<< std::endl;
        marker_barva[0] = 0.0f;
        marker_barva[1] = 0.0f;
        marker_barva[2] = 1.0f;
    }
    else if (hue > 15 && hue < 35) { // 23
        std::cerr << "yellow"<< std::endl;
        marker_barva[0] = 1.0f;
        marker_barva[1] = 1.0f;
        marker_barva[2] = 0.0f;
    }

    
    //std::cerr << "RGB komponenta: " << r << " " << g << " " <<  b << std::endl;

    //rgb to hsv
    
    
    // barve -------------------------------------------------------------------------------------------------------------------------------------------

    //Create a point in the "camera_rgb_optical_frame"
    geometry_msgs::PointStamped point_camera;
    geometry_msgs::PointStamped point_map;
    visualization_msgs::Marker marker;
    geometry_msgs::TransformStamped tss;
    
    point_camera.header.frame_id = "camera_rgb_optical_frame";
    point_camera.header.stamp = ros::Time::now();

    point_map.header.frame_id = "map";
    point_map.header.stamp = ros::Time::now();

    point_camera.point.x = centroid[0];
    point_camera.point.y = centroid[1];
    point_camera.point.z = centroid[2];

    try{
        time_test = ros::Time::now();

        std::cerr << time_rec << std::endl;
        std::cerr << time_test << std::endl;
        tss = tf2_buffer.lookupTransform("map","camera_rgb_optical_frame", time_rec);
        //tf2_buffer.transform(point_camera, point_map, "map", ros::Duration(2));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Transform warning: %s\n", ex.what());
    }

    //std::cerr << tss ;

    tf2::doTransform(point_camera, point_map, tss);

    //std::cerr << "point_camera: " << point_camera.point.x << " " <<  point_camera.point.y << " " <<  point_camera.point.z << std::endl;

    //std::cerr << "point_map: " << point_map.point.x << " " <<  point_map.point.y << " " <<  point_map.point.z << std::endl;

    marker.header.frame_id = "map";
    marker.header.stamp = time_rec;

    marker.ns = "cylinder";
    marker.id = marker_cnt++;

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = point_map.point.x;
    marker.pose.position.y = point_map.point.y;
    marker.pose.position.z = point_map.point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = marker_barva[0];
    marker.color.g = marker_barva[1];
    marker.color.b = marker_barva[2];
    marker.color.a = 1.0f;

    //marker.lifetime = ros::Duration();
    //marker_cnt++;
    //markers.markers.resize(marker_cnt);

    markers.markers.push_back(marker);
        
    //marker_array.publish(markers);
    pubm.publish (marker);

    pcl::PCLPointCloud2 outcloud_cylinder;
    pcl::toPCLPointCloud2 (*cloud_cylinder, outcloud_cylinder);
    std::cerr << "\ndodan marker\n" << std::endl;
    puby.publish (outcloud_cylinder);
    std::cerr << "\n publishan marker\n" << std::endl;
  }
  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cylinder_segment");
  ros::NodeHandle nh;
  // For transforming between coordinate frames
  tf2_ros::TransformListener tf2_listener(tf2_buffer);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pubx = nh.advertise<pcl::PCLPointCloud2> ("planes", 1);
  puby = nh.advertise<pcl::PCLPointCloud2> ("cylinder", 1);
  //pubz = nh.advertise<pcl::PointCloud<PointT> > ("point_cloud", 1);

  pubm = nh.advertise<visualization_msgs::Marker>("detected_cylinder",1);
  //marker_array = nh.advertise<visualization_msgs::MarkerArray>("markers", 100);
  markers.markers.resize(100);
  marker_cnt = 0;
  // Spin
  ros::spin ();
}
