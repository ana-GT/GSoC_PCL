/**
 * @function SQ_utils_SE_test.cpp
 */
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <SQ_utils.h>


int main( int argc, char* argv[] ) {

  double a1 = 0.5;
  double a2 = 0.8;
  double e = 0.2;
  int N = 1000;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  cloud = sampleSE_FranklinBarr( a1, a2, e, N );
  
  std::cout << "Cloud size: "<< cloud->points.size() << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("SQ_utils_SE_test Viewer") );
  viewer->addCoordinateSystem(1.0, 0);
  
  // 2. Visualize super ellipse
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col(cloud, 0,255,0);
  viewer->addPointCloud( cloud, col, "Superellipse" );

  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000) );
  }
  
  return 0;
}
