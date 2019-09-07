/***********************************************************
 * Author:  Daniel Tran
 *          Liam Gogley
 * 
 * Purpose: The following .cpp file will utilize the Intel
 *          realsense camera to stream and capture frame
 *          data of the environment. Color is then applied
 *          and a point cloud is generated and saved to
 *          a point cloud data format (.pcd).
 * 
 * Version 0.09 - Last Editted 11/07/18
 * 
 * Rev:     Implementation of RGB Texture function to map
 *          color to point cloud data.
 * 
 ***********************************************************/

#include <iostream>
#include <algorithm> 
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>

// Intel Realsense Headers
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;

//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values. 
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
  
    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//=================================================== 
pcl::PointCloud<PointT>::Ptr PCL_Conversion(const rs2::points& points, const rs2::video_frame& color){

    // Object Declaration (Point Cloud)
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud->width  = static_cast<uint32_t>( sp.width()  );   
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>

    }
    
   return cloud; // PCL RGB Point Cloud generated
}

int main() {
    //======================
    // Variable Declaration
    //======================
    bool captureLoop = true; // Loop control for generating point clouds
   
    //====================
    // Object Declaration
    //====================
    pcl::PointCloud<PointT>::Ptr newCloud (new pcl::PointCloud<PointT>);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> openCloud;

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    
    //======================
    // Stream configuration
    //======================
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    
    rs2::pipeline_profile selection = pipe.start(cfg); 

    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    
    // Wait for frames from the camera to settle
    for (int i = 0; i < 30; i++) {
    	auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
    }

	// Capture a single frame and obtain depth + RGB values from it    
	auto frames = pipe.wait_for_frames();
	auto depth = frames.get_depth_frame();
	auto RGB = frames.get_color_frame();

	// Map Color texture to each point
    pc.map_to(RGB);

	// Generate Point Cloud
	auto points = pc.calculate(depth);

	// Convert generated Point Cloud to PCL Formatting
	pcl::PointCloud<PointT>::Ptr cloud = PCL_Conversion(points, RGB);
    
    //PCL objects
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
  	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	pcl::ExtractIndices<PointT> extract;
	pcl::PLYWriter writer;

	//PCL Datasets
  	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
  	pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices); 
  	      
	//========================================
	// Filter PointCloud (PassThrough Method)
	//========================================
	pcl::PassThrough<PointT> pass; // Create the filtering object
	pass.setInputCloud (cloud);           // Input generated cloud to filter
	pass.setFilterFieldName ("z");        // Set field name to Z-coordinate
    pass.setFilterLimits (0.0, 1.5);      // Set accepted interval values
    pass.filter (*newCloud);              // Filtered Cloud Outputted    
    writer.write ("Captured_Frame.ply", *newCloud, false);
                
	//HSV領域でのポイントの削除
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_HSVfiltered (new pcl::PointCloud<pcl::PointXYZHSV>);
  	pcl::PointIndices::Ptr inliers_color(new pcl::PointIndices()); 
  	pcl::ExtractIndices<PointT> extract_color;
	pcl::PointCloudXYZRGBtoXYZHSV (*newCloud, *cloud_HSVfiltered);
 	for (int i = 0; i < cloud_HSVfiltered->points.size (); i++) {  
		if(cloud_HSVfiltered->points[i].h>=350||cloud_HSVfiltered->points[i].h<10){
			inliers_color->indices.push_back(i);

		}
  	}

  	pcl::PointCloud<PointT>::Ptr fruit_cloud (new pcl::PointCloud<PointT>);
  	extract_color.setInputCloud(newCloud); 
  	extract_color.setIndices(inliers_color); 
  	extract_color.setNegative(false); 
  	extract_color.filter(*fruit_cloud);//赤色抽出	
    
	writer.write("fruit.ply", *fruit_cloud, false);

	int maxFruitNum=6,fruitNum=6;
	//果実検出
	double Fmodel[maxFruitNum][4];
	printf("Fruit detection\n");
	for(int num=0;num<maxFruitNum;num++){
		std::ostringstream oss;
		oss<<num;
		// Estimate point normals
		//ポイントの法線を推定
		printf("Estimate point normals\n");
		ne.setSearchMethod (tree);
		ne.setInputCloud (fruit_cloud);
		ne.setKSearch (50);
		ne.compute (*cloud_normals);
		// Create the segmentation object for sphere segmentation and set all the parameters
		//球面モデルのパラメータを設定
		printf("Parameter setting\n");
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_SPHERE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setNormalDistanceWeight (0.1);//点法線と平面法線の間の角度距離（0〜π/ 2）に与える相対的な重み
		seg.setMaxIterations (10000);
		seg.setDistanceThreshold (0.002);//0.06);//容認する誤差範囲
		seg.setRadiusLimits (0.02, 0.04);//0.3);//果実半径
		seg.setInputCloud (fruit_cloud);
		seg.setInputNormals (cloud_normals);
		// Obtain the sphere inliers and coefficients
		//球面モデルの係数を取得
		printf("Obatain coefficients\n");
		seg.segment (*inliers_sphere, *coefficients_sphere);
		std::cerr << "sphere coefficients: " << *coefficients_sphere << std::endl;
		// Write the sphere inliers to disk
		extract.setInputCloud (fruit_cloud);
		extract.setIndices (inliers_sphere);
		extract.setNegative (false);
		pcl::PointCloud<PointT>::Ptr cloud_sphere (new pcl::PointCloud<PointT> ());
		extract.filter (*cloud_sphere);
		if (cloud_sphere->points.empty ()){
			std::cerr << "Can't find the sphere component." << std::endl;
			fruitNum=num;
			break;
		}	
		else
		{
			//std::cerr << "PointCloud representing the sphere component: " << cloud_sphere->points.size () << " data points." << std::endl;
			Fmodel[num][0]=coefficients_sphere->values[0];
			Fmodel[num][1]=coefficients_sphere->values[1];
			Fmodel[num][2]=coefficients_sphere->values[2];
			Fmodel[num][3]=coefficients_sphere->values[3];
			printf("%d xyz:%f %f %f r:%f\n\n",num,Fmodel[num][0],Fmodel[num][1],Fmodel[num][2],Fmodel[num][3]);
			writer.write ("sphere"+oss.str()+".ply", *cloud_sphere, false);
			extract.setNegative (true);
			extract.filter (*fruit_cloud);
		}
	}

    cout << "Exiting Program... " << endl; 
    return EXIT_SUCCESS;
}
