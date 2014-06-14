/**
 * @file ceresSolver.cpp
 */
#include <pcl/io/pcd_io.h>
#include <ceres/ceres.h>
#include <string>
#include <unistd.h>

#include <SQ_sampler.h>

struct SQBaseEquation {

  SQBaseEquation( double _x, double _y, double _z ) : 
    x_(_x), y_(_y), z_(_z) {}

  template<typename T> bool operator() ( const T* const dim,
					 const T* const e,
					 T* residual ) const {
    

    // F(x) : Base SQ equation
    T F; T fxy; T fz;
    fxy = pow( (T(x_)/dim[0])*(T(x_)/dim[0]), T(1.0)/e[1] ) + pow( (T(y_)/dim[1])*(T(y_)/dim[1]), T(1.0)/e[1] );
    fz =  pow( (z_/dim[2])*(z_/dim[2]), T(1.0)/e[0] );
    F = pow(fxy, T(e[1]/e[0]) ) + fz;    
    residual[0] = pow(F, e[1]) -T(1.0);
    return true;
    
  }

private:
  double x_;
  double y_;
  double z_;
};


/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Parameters
  double* dim = new double[3];
  double* e = new double[2];

  // Read pointcloud
  int v;
  std::string pcd_filename;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );


  SQ_params par;
  SQ_sampler sqs;
  cloud = sqs.sampleSQ_naive( par );

  while( (v=getopt( argc, argv, "p")) != -1 ) {

    switch(v) {
    case 'p': {
      pcd_filename = std::string( argv[1] );
      
      if( pcl::io::loadPCDFile<pcl::PointXYZ>( pcd_filename,
					       * cloud ) == -1 ) {
	std::cout << "\t [ERROR] Could not read pointcloud"<< std::endl;
	return 1;
      }

    } break;
    } // end switch

  }

  // Approximate bounding box
  


  // Create problem and fill it with residual blocks
  ceres::Problem problem;
  pcl::PointCloud<pcl::PointXYZ>::iterator it;
  for( it = cloud->begin(); it != cloud->end(); ++it ) {
    problem.AddResidualBlock( new ceres::AutoDiffCostFunction<SQBaseEquation, 1, 3, 2>( new SQBaseEquation( (*it).x, (*it).y, (*it).z ) ),
			      new ceres::CauchyLoss(0.5),
			      dim, e );
  }

  // Add options
  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;

  ceres::Solver::Summary summary;

  // Solve
  ceres::Solve( options, &problem, &summary );

  // Visualize final results
  std::cout << summary.BriefReport() << std::endl;

  return 0; 
}

// Local Variables:
// mode: c++
// tab-width: 4
// End:
