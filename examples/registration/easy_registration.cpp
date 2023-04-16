/*
 *\brief 简单的匹配算法
 *\desc
 *	应用场景
 *		1. 点位置很精确：建模人员ctrl+c、ctrl+v出来的模型，带有平移、旋转、缩放等变换
 *		2. 点云数量很少：可能只是一个长方体，只有8个顶点
 */

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/ply_io.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>

using namespace std;
using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

pcl::PointCloud<pcl::Normal>::Ptr computeNormal(const PointCloud::Ptr& cloud)
{
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud< pcl::Normal>);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
	ne_src.setInputCloud(cloud);
	ne_src.setRadiusSearch(0.02);

#if 0
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_src.setSearchMethod(tree_src);
#endif

	ne_src.compute(*cloud_normals);
	return cloud_normals;
}

class ComputeFeatureUtils
{
public:
	//形状上下文
	struct Compute3dsc
	{
		using SACType = pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::ShapeContext1980>;

		pcl::PointCloud<pcl::ShapeContext1980>::Ptr operator()(const PointCloud::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr normal = nullptr)
		{
			pcl::PointCloud<pcl::ShapeContext1980>::Ptr cloud_sc(new pcl::PointCloud<pcl::ShapeContext1980>());

			if (!normal)
			{
				normal = computeNormal(cloud);
			}

			pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980> sp_tgt;
			sp_tgt.setInputCloud(cloud);
			sp_tgt.setInputNormals(normal);
			sp_tgt.setRadiusSearch(0.5);

#if 0
			//kdTree加速
			pcl::search::KdTree<PointT>::Ptr tree_tgt_sp(new pcl::search::KdTree<PointT>);
			sp_tgt.setSearchMethod(tree_tgt_sp);
#endif

			sp_tgt.compute(*cloud_sc);
			return cloud_sc;
		}
	};
	//
	struct ComputeFpfh
	{
		using SACType = pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33>;

		pcl::PointCloud<pcl::FPFHSignature33>::Ptr operator()(const PointCloud::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr normal = nullptr)
		{
			if (!normal)
			{
				normal = computeNormal(cloud);
			}

			pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;

			fpfh_src.setInputCloud(cloud);
			fpfh_src.setInputNormals(normal);

#if 0
			pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
			fpfh_src.setSearchMethod(tree_src_fpfh);
#endif

			pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
			fpfh_src.setRadiusSearch(0.05);
			fpfh_src.compute(*fpfhs_src);

			return fpfhs_src;
		}
	};
	//
	struct ComputePfh
	{
		using SACType = pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::PFHSignature125>;

		pcl::PointCloud<pcl::PFHSignature125>::Ptr operator()(const PointCloud::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr normal = nullptr)
		{
			if (normal)
			{
				normal = computeNormal(cloud);
			}

			pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;

			pfh.setInputCloud(cloud);
			pfh.setInputNormals(normal);
			pfh.setRadiusSearch(0.05);

#if 0
			pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
			pfh.setSearchMethod(tree_tgt_fpfh);
#endif

			pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());
			pfh.compute(*pfhs);
			return pfhs;
		}
	};
};

template<typename CompuateFeature, typename SACType>
struct SAC
{
	void operator()(const PointCloud::Ptr& source, const PointCloud::Ptr& target, PointCloud::Ptr result)
	{
		SACType scia;
		
		scia.setInputSource(source);
		scia.setInputTarget(target);

		scia.setSourceFeatures(CompuateFeature()(source));
		scia.setTargetFeatures(CompuateFeature()(target));
		//scia.setMinSampleDistance(1);
		//scia.setNumberOfSamples(2);
		//scia.setCorrespondenceRandomness(20);

		PointCloud::Ptr sac_result(new PointCloud);
		scia.align(*sac_result);

		std::cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;

		auto sac_trans = scia.getFinalTransformation();

		pcl::transformPointCloud(*source, *sac_result, sac_trans);
	}
};
using SAC_3dsc	= SAC<ComputeFeatureUtils::Compute3dsc, ComputeFeatureUtils::Compute3dsc::SACType>;
using SAC_fpfh	= SAC<ComputeFeatureUtils::ComputeFpfh, ComputeFeatureUtils::ComputeFpfh::SACType>;
using SAC_pfh	= SAC<ComputeFeatureUtils::ComputePfh,	ComputeFeatureUtils::ComputePfh::SACType>;

struct NDT
{
	void operator()(const PointCloud::Ptr& source, const PointCloud::Ptr& target, PointCloud::Ptr result)
	{
		pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

		ndt.setInputSource(source);
		ndt.setInputTarget(target);

		ndt.setTransformationEpsilon(0.01);
		ndt.setStepSize(0.05);
		ndt.setResolution(3);//网格分辨率
		ndt.setMaximumIterations(100);

		ndt.align(*result);

		auto transform = ndt.getFinalTransformation();
		std::cout << transform << std::endl;
		pcl::transformPointCloud(*source, *result, transform);
	}
};

struct ICP
{
	void operator()(const PointCloud::Ptr& source, const PointCloud::Ptr& target, PointCloud::Ptr result)
	{
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

		//kdTree加速搜索
		//这里只是简单的模型，无需加速
#if 0
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
		tree1->setInputCloud(source);
		icp.setSearchMethodSource(tree1);
		
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
		tree2->setInputCloud(target);
		icp.setSearchMethodTarget(tree2);
#endif

		//设置参数
		icp.setInputSource(source);
		icp.setInputTarget(target);

		icp.setMaxCorrespondenceDistance(0.5);//当两个点云相距较远时候，距离值要变大，所以一开始需要粗配准。
		icp.setTransformationEpsilon(1e-10);//svd奇异值分解，对icp时间影响不大
		icp.setEuclideanFitnessEpsilon(0.01);//前后两次误差大小，当误差值小于这个值停止迭代
		icp.setMaximumIterations(100);//最大迭代次数
		icp.align(*result);

		auto transform = icp.getFinalTransformation();
		std::cout << transform << std::endl;

		pcl::transformPointCloud(*source, *result, transform);
	}
};

template<typename Op>
struct OneRegistration
{
	void operator()(const std::string& source_path,
		const std::string& target_path,
		const std::string& result_path)
	{
		PointCloud::Ptr source_cloud{ new PointCloud };
		PointCloud::Ptr target_cloud{ new PointCloud };
		PointCloud::Ptr result_cloud{ new PointCloud };

		{
			pcl::io::loadPLYFile(source_path, *source_cloud);
			pcl::io::loadPLYFile(target_path, *target_cloud);
		}
		{
			pcl::io::savePLYFileASCII(source_path + "pcl.ply", *source_cloud);
			pcl::io::savePLYFileASCII(target_path + "pcl.ply", *target_cloud);
		}

		Op()(source_cloud, target_cloud, result_cloud);

		pcl::io::savePLYFileASCII(result_path, *result_cloud);
	}
};

template<typename Op1, typename Op2>
struct TowRegistration
{
	void operator()(const std::string& source_path,
					const std::string& target_path,
					const std::string& middle_path,
					const std::string& result_path)
	{
		PointCloud::Ptr source_cloud{ new PointCloud };
		PointCloud::Ptr target_cloud{ new PointCloud };
		PointCloud::Ptr middle_cloud{ new PointCloud }; //粗配准结果
		PointCloud::Ptr result_cloud{ new PointCloud }; //精配准结果

		if (pcl::io::loadPLYFile(source_path, *source_cloud) == -1)
		{
			std::cout << "read error! " << source_path << std::endl;
			return;
		}

		if (pcl::io::loadPLYFile(target_path, *target_cloud) == -1)
		{
			std::cout << "read error! " << target_path << std::endl;
			return;
		}

		{
			pcl::io::savePLYFileASCII(source_path + "pcl.ply", *source_cloud);
			pcl::io::savePLYFileASCII(target_path + "pcl.ply", *target_cloud);
		}

		Op1()(source_cloud, target_cloud, middle_cloud);
		pcl::io::savePLYFileASCII(middle_path, *middle_cloud);

		Op2()(middle_cloud, target_cloud, result_cloud);
		pcl::io::savePLYFileASCII(result_path, *result_cloud);
	}
};

int main()
{
#if 0
	OneRegistration<ICP>()(
		DATA_PATH "/small_close/5.ply",
		DATA_PATH "/small_close/6.ply",
		DATA_PATH "/small_close/result.ply"
		);
#endif

#if 0
	TowRegistration<NDT, ICP>()(
		DATA_PATH "/small_close/5.ply",
		DATA_PATH "/small_close/6.ply",
		DATA_PATH "/small_close/middle.ply",
		DATA_PATH "/small_close/result.ply"
	);
#endif

#if 1
	std::string dir = DATA_PATH "502cfx/";

	//OneRegistration<ICP>()(
	//	dir + "modelA.ply",
	//	dir + "modelB.ply",
	//	dir + "icp_result.ply"
	//	);

	TowRegistration<SAC_3dsc, ICP>()(
		dir + "modelA.ply",
		dir + "modelB.ply",
		dir + "middle.ply",
		dir + "result.ply"
	);
#endif

	return 0;
}
