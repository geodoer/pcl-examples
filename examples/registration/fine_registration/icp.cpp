/*
 *\references: 
 1. 点云配准——（1）几种点云配准算法比较_小修勾的博客-CSDN博客_点云配准算法
	https://blog.csdn.net/weixin_43236944/article/details/88188532
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include<pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>//去除NAN点的头文件

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


//点云可视化

void visualize_pcd(PointCloud::Ptr pcd_src, PointCloud::Ptr pcd_tgt, PointCloud::Ptr pcd_final)
{
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	//原始点云绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
	//目标点云红色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
	//匹配好的点云蓝色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 0, 255);

	viewer.setBackgroundColor(255,255,255);
	viewer.addPointCloud(pcd_src, src_h, "source cloud");
	viewer.addPointCloud(pcd_tgt, tgt_h, "target cloud");
	viewer.addPointCloud(pcd_final, final_h, "result cloud");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	

}
int main(int argc, char** argv)
{

	//创建点云指针
	PointCloud::Ptr cloud_source(new PointCloud);
	PointCloud::Ptr cloud_target(new PointCloud);
	
	
	// 加载点云文件
	pcl::io::loadPCDFile("E:/vs13/pcldata/bun/rabbit.pcd", *cloud_source);
	std::cout << "source loaded!" << std::endl;
	pcl::io::loadPCDFile("E:/vs13/pcldata/bun/rabbit_t.pcd", *cloud_target);
	std::cout << "target loaded!" << std::endl;

	clock_t start = clock();
	//去除NAN点

	std::vector<int> indices_src; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*cloud_source, *cloud_source, indices_src);
	std::cout << "remove *cloud_source nan" << endl;

	std::vector<int> indices_tgt; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*cloud_target, *cloud_target, indices_tgt);
	std::cout << "remove *cloud_tgt nan" << endl;
	//下采样滤波
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(0.012, 0.012, 0.012);
	voxel_grid.setInputCloud(cloud_source);
	PointCloud::Ptr cloud_src(new PointCloud);
	voxel_grid.filter(*cloud_src);
	std::cout << "down size *cloud_src_o from " << cloud_source->size() << "to" << cloud_src->size() << endl;

	//
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
	voxel_grid.setLeafSize(0.012, 0.012, 0.012);
	voxel_grid.setInputCloud(cloud_target);
	PointCloud::Ptr cloud_tgt(new PointCloud);
	voxel_grid.filter(*cloud_tgt);
	std::cout << "down size *cloud_target from " << cloud_target->size() << "to" << cloud_tgt->size() << endl;
	
	//gicp time
	
	// ICP配准
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	PointCloud::Ptr cloud_source_registration(new PointCloud);
	//kdTree 加速搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(cloud_src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(cloud_tgt);
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);

	//设置参数
	icp.setInputSource(cloud_src);
	icp.setInputTarget(cloud_tgt);
	icp.setMaxCorrespondenceDistance(2);//当两个点云相距较远时候，距离值要变大，所以一开始需要粗配准。
	icp.setTransformationEpsilon(1e-6);//svd奇异值分解，对icp时间影响不大
	icp.setEuclideanFitnessEpsilon(0.01);//前后两次误差大小，当误差值小于这个值停止迭代
	icp.setMaximumIterations(60);//最大迭代次数
	icp.align(*cloud_source_registration);

	clock_t end = clock();
	cout << "icp time" << (double)(end - start) / CLOCKS_PER_SEC << endl;

	Eigen::Matrix4f transformation = icp.getFinalTransformation();
	std::cout << transformation << std::endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source_registration, transformation);

	

	//可视化
	visualize_pcd(cloud_source, cloud_target, cloud_source_registration);
	return 0;
}
