/*
 *\references: 
 1. ������׼������1�����ֵ�����׼�㷨�Ƚ�_С�޹��Ĳ���-CSDN����_������׼�㷨
	https://blog.csdn.net/weixin_43236944/article/details/88188532
 */
#include <pcl/registration/ia_ransac.h>//����һ����
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>//
#include <pcl/filters/filter.h>//
#include <pcl/registration/icp.h>//icp��׼
#include <pcl/visualization/pcl_visualizer.h>//���ӻ�
#include <time.h>//ʱ��

using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//���ƿ��ӻ�
void visualize_pcd(PointCloud::Ptr pcd_src, PointCloud::Ptr pcd_tgt, PointCloud::Ptr pcd_final)
{

	//������ʼ��Ŀ��
	pcl::visualization::PCLVisualizer viewer("registration Viewer");


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 0, 255);
	viewer.setBackgroundColor(255,255,255);
	viewer.addPointCloud(pcd_src, src_h, "source cloud");
	viewer.addPointCloud(pcd_tgt, tgt_h, "tgt cloud");
	viewer.addPointCloud(pcd_final, final_h, "final cloud");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

//����תƽ�ƾ��������ת�Ƕ�
void matrix2angle(Eigen::Matrix4f &result_trans, Eigen::Vector3f &result_angle)
{
	double ax, ay, az;
	if (result_trans(2, 0) == 1 || result_trans(2, 0) == -1)
	{
		az = 0;
		double dlta;
		dlta = atan2(result_trans(0, 1), result_trans(0, 2));
		if (result_trans(2, 0) == -1)
		{
			ay = M_PI / 2;
			ax = az + dlta;
		}
		else
		{
			ay = -M_PI / 2;
			ax = -az + dlta;
		}
	}
	else
	{
		ay = -asin(result_trans(2, 0));
		ax = atan2(result_trans(2, 1) / cos(ay), result_trans(2, 2) / cos(ay));
		az = atan2(result_trans(1, 0) / cos(ay), result_trans(0, 0) / cos(ay));
	}
	result_angle << ax, ay, az;

	cout << "x����ת�Ƕȣ�" << ax << endl;
	cout << "y����ת�Ƕȣ�" << ay << endl;
	cout << "z����ת�Ƕȣ�" << az << endl;
}
int main(int argc, char** argv)
{
	//���ص����ļ�
	PointCloud::Ptr cloud_src_o(new PointCloud);//ԭ���ƣ�����׼
	pcl::io::loadPCDFile("E:/vs13/pcldata/bun/rabbit.pcd", *cloud_src_o);
	PointCloud::Ptr cloud_tgt_o(new PointCloud);//Ŀ�����
	pcl::io::loadPCDFile("E:/vs13/pcldata/bun/rabbit_1.pcd", *cloud_tgt_o);

	clock_t start = clock();

	//ȥ��NAN��
	std::vector<int> indices_src; //����ȥ���ĵ������
	pcl::removeNaNFromPointCloud(*cloud_src_o, *cloud_src_o, indices_src);
	std::cout << "remove *cloud_src_o nan" << endl;

	std::vector<int> indices_tgt;
	pcl::removeNaNFromPointCloud(*cloud_tgt_o, *cloud_tgt_o, indices_tgt);
	std::cout << "remove *cloud_tgt_o nan" << endl;

	//�²����˲�
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(0.012, 0.012, 0.012);
	voxel_grid.setInputCloud(cloud_src_o);
	PointCloud::Ptr cloud_src(new PointCloud);
	voxel_grid.filter(*cloud_src);
	std::cout << "down size *cloud_src_o from " << cloud_src_o->size() << "to" << cloud_src->size() << endl;
	//::io::savePCDFileASCII("rabbit_src_down.pcd", *cloud_src);


	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
	voxel_grid_2.setLeafSize(0.012, 0.012, 0.012);
	voxel_grid_2.setInputCloud(cloud_tgt_o);
	PointCloud::Ptr cloud_tgt(new PointCloud);
	voxel_grid_2.filter(*cloud_tgt);
	std::cout << "down size *cloud_tgt_o.pcd from " << cloud_tgt_o->size() << "to" << cloud_tgt->size() << endl;


	//������淨��
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
	ne_src.setInputCloud(cloud_src);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_src.setSearchMethod(tree_src);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
	ne_src.setRadiusSearch(0.02);
	ne_src.compute(*cloud_src_normals);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
	ne_tgt.setInputCloud(cloud_tgt);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_tgt.setSearchMethod(tree_tgt);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
	//ne_tgt.setKSearch(20);
	ne_tgt.setRadiusSearch(0.02);
	ne_tgt.compute(*cloud_tgt_normals);

	//����PFH
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_src;
	pfh_src.setInputCloud(cloud_src);
	pfh_src.setInputNormals(cloud_src_normals);
	pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
	pfh_src.setSearchMethod(tree_src_fpfh);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_src(new pcl::PointCloud<pcl::PFHSignature125>());
	pfh_src.setRadiusSearch(0.05);
	pfh_src.compute(*pfhs_src);
	std::cout << "compute *cloud_src pfh" << endl;

	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_tgt;
	pfh_src.setInputCloud(cloud_tgt);
	pfh_src.setInputNormals(cloud_tgt_normals);
	pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
	pfh_src.setSearchMethod(tree_tgt_fpfh);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs_tgt(new pcl::PointCloud<pcl::PFHSignature125>());
	pfh_src.setRadiusSearch(0.05);
	pfh_src.compute(*pfhs_tgt);
	std::cout << "compute *cloud_tgt pfh" << endl;


	//SAC��׼
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::PFHSignature125> scia;
	scia.setInputSource(cloud_src);
	scia.setInputTarget(cloud_tgt);
	scia.setSourceFeatures(pfhs_src);
	scia.setTargetFeatures(pfhs_tgt);
	//scia.setMinSampleDistance(1);
	//scia.setNumberOfSamples(2);
	//scia.setCorrespondenceRandomness(20);
	PointCloud::Ptr sac_result(new PointCloud);
	scia.align(*sac_result);
	std::cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	Eigen::Matrix4f sac_trans;
	sac_trans = scia.getFinalTransformation();
	std::cout << sac_trans << endl;

	clock_t sac_time = clock();


	cout << "sac time: " << (double)(sac_time - start) / (double)CLOCKS_PER_SEC << " s" << endl;


	pcl::transformPointCloud(*cloud_src_o, *sac_result, sac_trans);
	
	//�������
	Eigen::Vector3f ANGLE_origin;
	Eigen::Vector3f TRANS_origin;
	ANGLE_origin << 0, 0, M_PI / 4;
	TRANS_origin << 0, 0.3, 0.2;
	double a_error_x, a_error_y, a_error_z;
	double t_error_x, t_error_y, t_error_z;
	Eigen::Vector3f ANGLE_result;
	matrix2angle(sac_trans, ANGLE_result);
	a_error_x = fabs(ANGLE_result(0)) - fabs(ANGLE_origin(0));
	a_error_y = fabs(ANGLE_result(1)) - fabs(ANGLE_origin(1));
	a_error_z = fabs(ANGLE_result(2)) - fabs(ANGLE_origin(2));
	cout << "����ʵ����ת�Ƕ�:\n" << ANGLE_origin << endl;
	cout << "x����ת��� : " << a_error_x << "  y����ת��� : " << a_error_y << "  z����ת��� : " << a_error_z << endl;

	cout << "����ʵ��ƽ�ƾ���:\n" << TRANS_origin << endl;
	t_error_x = fabs(sac_trans(0, 3)) - fabs(TRANS_origin(0));
	t_error_y = fabs(sac_trans(1, 3)) - fabs(TRANS_origin(1));
	t_error_z = fabs(sac_trans(2, 3)) - fabs(TRANS_origin(2));
	cout << "����õ���ƽ�ƾ���" << endl << "x��ƽ��" << sac_trans(0, 3) << endl << "y��ƽ��" << sac_trans(1, 3) << endl << "z��ƽ��" << sac_trans(2, 3) << endl;
	cout << "x��ƽ����� : " << t_error_x << "  y��ƽ����� : " << t_error_y << "  z��ƽ����� : " << t_error_z << endl;

	//���ӻ�
	visualize_pcd(cloud_src_o, cloud_tgt_o, sac_result);
	return (0);
}
