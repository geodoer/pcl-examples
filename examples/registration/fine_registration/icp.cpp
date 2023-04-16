/*
 *\references: 
 1. ������׼������1�����ֵ�����׼�㷨�Ƚ�_С�޹��Ĳ���-CSDN����_������׼�㷨
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
#include <pcl/features/normal_3d.h>//ȥ��NAN���ͷ�ļ�

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


//���ƿ��ӻ�

void visualize_pcd(PointCloud::Ptr pcd_src, PointCloud::Ptr pcd_tgt, PointCloud::Ptr pcd_final)
{
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	//ԭʼ������ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
	//Ŀ����ƺ�ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
	//ƥ��õĵ�����ɫ
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

	//��������ָ��
	PointCloud::Ptr cloud_source(new PointCloud);
	PointCloud::Ptr cloud_target(new PointCloud);
	
	
	// ���ص����ļ�
	pcl::io::loadPCDFile("E:/vs13/pcldata/bun/rabbit.pcd", *cloud_source);
	std::cout << "source loaded!" << std::endl;
	pcl::io::loadPCDFile("E:/vs13/pcldata/bun/rabbit_t.pcd", *cloud_target);
	std::cout << "target loaded!" << std::endl;

	clock_t start = clock();
	//ȥ��NAN��

	std::vector<int> indices_src; //����ȥ���ĵ������
	pcl::removeNaNFromPointCloud(*cloud_source, *cloud_source, indices_src);
	std::cout << "remove *cloud_source nan" << endl;

	std::vector<int> indices_tgt; //����ȥ���ĵ������
	pcl::removeNaNFromPointCloud(*cloud_target, *cloud_target, indices_tgt);
	std::cout << "remove *cloud_tgt nan" << endl;
	//�²����˲�
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
	
	// ICP��׼
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	PointCloud::Ptr cloud_source_registration(new PointCloud);
	//kdTree ��������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(cloud_src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(cloud_tgt);
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);

	//���ò���
	icp.setInputSource(cloud_src);
	icp.setInputTarget(cloud_tgt);
	icp.setMaxCorrespondenceDistance(2);//��������������Զʱ�򣬾���ֵҪ�������һ��ʼ��Ҫ����׼��
	icp.setTransformationEpsilon(1e-6);//svd����ֵ�ֽ⣬��icpʱ��Ӱ�첻��
	icp.setEuclideanFitnessEpsilon(0.01);//ǰ����������С�������ֵС�����ֵֹͣ����
	icp.setMaximumIterations(60);//����������
	icp.align(*cloud_source_registration);

	clock_t end = clock();
	cout << "icp time" << (double)(end - start) / CLOCKS_PER_SEC << endl;

	Eigen::Matrix4f transformation = icp.getFinalTransformation();
	std::cout << transformation << std::endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source_registration, transformation);

	

	//���ӻ�
	visualize_pcd(cloud_source, cloud_target, cloud_source_registration);
	return 0;
}
