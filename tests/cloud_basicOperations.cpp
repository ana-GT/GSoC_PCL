/**
 * @file cloud_basicOperations
 */

#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <stdlib.h>

void printHelp();
void show( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud );
void getBoundingBox( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
		     Eigen::Vector3f &_centroid,
		     Eigen::Matrix3f &_rot,
		     Eigen::Vector3f &_dim,
		     pcl::PointCloud<pcl::PointXYZ>::Ptr &_centeredCloud );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  int v;
  std::string output_name = "output_name.pcd";
  std::string input_name;

  bool scaleCloud_flag = false;
  bool centerCloud_flag = false;

  double scale = 1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr mid_cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud( new pcl::PointCloud<pcl::PointXYZ>() );

  while( (v=getopt(argc, argv, "p:hs:o:c") ) != -1 ) {
    switch(v) {
      
      /** Input pointcloud */
    case 'p' : {
      
      input_name = std::string( optarg );
      if( pcl::io::loadPCDFile<pcl::PointXYZ>( input_name,
					       *input_cloud ) == -1 ) {
	std::cout << "\t [ERROR] Could not read pointcloud "<<
	  input_name<< std::endl;
	return 1;
      }

    } break;

      /**Help */
    case 'h' : {
      printHelp();
    } break;

      /** Scale pointcloud */
    case 's' : {
      scaleCloud_flag = true;
      scale = atof( optarg );
    } break;

      /** Store pointcloud in filename  */      
    case 'o' : {
      output_name = std::string( optarg );
    } break;

      /** Center and normalize pointcloud */
    case 'c' : {
      centerCloud_flag = true;
    } break;
      
    } // switch
  }
  
  // Get bounding box information
  Eigen::Vector3f centroid, dim; 
  Eigen::Matrix3f rot;
  std::cout << "Get bounding box"<<std::endl;
  getBoundingBox( input_cloud,
		  centroid,
		  rot,
		  dim,
		  mid_cloud );
  std::cout << "NEXT"<<std::endl;
  Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f tf_center = Eigen::Matrix4f::Identity();
  tf_center.block(0,3,3,1) = centroid;
  tf_center.block(0,0,3,3) = rot;  

  // If scale
  if( scaleCloud_flag ) {

    Eigen::Matrix4f tf_sc = Eigen::Matrix4f::Identity();
    for( int i = 0; i < 3; ++i ) { tf_sc(i,i) = scale; }
    
    if( centerCloud_flag ) {
      transf = tf_sc*tf_center.inverse();
    }
    else {
      transf = tf_center*tf_sc*tf_center.inverse();      
    }

  } else {
    if( centerCloud_flag ) {
      transf = tf_center.inverse();
    }
  }


  // Show
  if( centerCloud_flag || scaleCloud_flag ) {
    std::cout << "\t Show centered or scale cloud"<<std::endl;
    pcl::transformPointCloud( *input_cloud, *output_cloud, transf );
    // Save
    pcl::io::savePCDFileASCII( output_name, *output_cloud );        
    std::cout <<"\t [*] Saved "<< output_name  << std::endl;
    show( output_cloud );

  } else {
    std::cout << "\t Show just input cloud"<<std::endl;
    show( input_cloud );
  }

  return 0;
}

/**
 * @function printHelp
 */
void printHelp() {
  std::cout<<"Syntax: ./cloud_basicOperations [i | s | o] "<<std::endl;
  std::cout<<"\t -p P: Input pointcloud"<<std::endl;
  std::cout<<"\t -i: Display bounding box information "<<std::endl;
  std::cout<<"\t -s S: Scale pointcloud"<<std::endl;
  std::cout<<"\t -o O: Output pcd filename"<<std::endl;
}

/**
 * @function getBoundingBox
 * @brief Get bounding box information
 */
void getBoundingBox( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
		     Eigen::Vector3f &_centroid,
		     Eigen::Matrix3f &_rot,
		     Eigen::Vector3f &_dim,
		     pcl::PointCloud<pcl::PointXYZ>::Ptr &_centeredCloud ) {

    // [DEBUG] Compute the bounding box center
    Eigen::Vector4d centroid;
    pcl::compute3DCentroid( *_cloud, centroid );

    _centroid << (float) centroid(0), (float) centroid(1), (float) centroid(2);
    
    // 2. Compute main axis orientations
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud( _cloud );
    Eigen::Vector3f eigVal = pca.getEigenValues();
    Eigen::Matrix3f eigVec = pca.getEigenVectors();
    // Make sure 3 vectors are normal w.r.t. each other
    Eigen::Vector3f v3 = (eigVec.col(0)).cross( eigVec.col(1) );
    eigVec.col(2) = v3;
 
    _rot = eigVec;

    // Transform _cloud
    Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
    transf.block(0,3,3,1) << (float)centroid(0), (float)centroid(1), (float)centroid(2);
    transf.block(0,0,3,3) = eigVec;
    Eigen::Matrix4f tinv; tinv = transf.inverse();

    pcl::transformPointCloud( *_cloud, *_centeredCloud, tinv );

    // Get maximum and minimum
    pcl::PointXYZ minPt; pcl::PointXYZ maxPt;
    pcl::getMinMax3D( *_centeredCloud, minPt, maxPt );

    // Get dimensions
    _dim(0) = (float)( maxPt.x - minPt.x )*0.5;
    _dim(1) = (float)( maxPt.y - minPt.y )*0.5;
    _dim(2) = (float)( maxPt.z - minPt.z )*0.5;
}

/**
 * @function showInfo
 * @brief Show pointcloud in viewer and bounding box information
 */
void show( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud ) {

    // [DEBUG] Visualize 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("showInfo viewer") );
    //viewer->addCoordinateSystem(1.0, 0);
     //viewer->setBackgroundColor(1,1,1);
    
    // [DEBUG] Visualize input pointcloud (GREEN)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col(_cloud, 0,255,0);
    viewer->addPointCloud( _cloud, col, "input cloud"  );

    
    // [DEBUG] Visualize bounding box
    Eigen::Vector3f centroid, dim; Eigen::Matrix3f rot;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2( new pcl::PointCloud<pcl::PointXYZ>() );
    getBoundingBox( _cloud, centroid, rot, dim, cloud2 ); 
	/*	    
    viewer->addCube( centroid,
		     Eigen::Quaternionf(rot),
		     dim(0)*2, dim(1)*2, dim(2)*2, 
		     "OBB" );
*/
    // [PRINT INFO]
    std::cout << "\t [INFO] Bounding box dimensions: "<< dim.transpose() << std::endl;
    std::cout << "\t [INFO] Centroid: "<< centroid.transpose() << std::endl;   
    std::cout << "\t [INFO] Rotation: "<< std::endl;
    std::cout << rot << std::endl;
 
    while( !viewer->wasStopped() ) {
	viewer->spinOnce(100);
	boost::this_thread::sleep( boost::posix_time::microseconds(100000) );
    }

}

