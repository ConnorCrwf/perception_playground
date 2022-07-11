#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <perception_playground/util/pcl_util.h>

typedef struct{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster;
    float centroid_z;
    float centroid_y;
    float width;
    float height;
    int num_points;
} Pcl_Cluster;

bool compareClustersZ(const Pcl_Cluster &i1, const Pcl_Cluster &i2)
{
    return (i1.centroid_z < i2.centroid_z);
}

bool compareClustersY(const Pcl_Cluster &i1, const Pcl_Cluster &i2)
{
    return (i1.centroid_y < i2.centroid_y);
}


int findMaxDiffIndex(std::vector<float> z_values)
{
    float max_diff_sum = 0.0;
    int max_diff_index;
    for (int i = 0; i < z_values.size(); ++i)
    {
      float diff_sum = 0.0;
      for (int j = 0; j < z_values.size(); ++j) {
        float diff = std::abs(z_values[i] - z_values[j]);
        diff_sum = diff_sum + diff;
      }
      std::cout << "Diff Sum at index " << i << " is " << diff_sum << std::endl;
      if (diff_sum > max_diff_sum) {
      max_diff_sum = diff_sum; 
      max_diff_index = i;
      }
    }
    std::cout << "Max Diff Sum is " << max_diff_sum << " at index " << max_diff_index << std::endl;
    return max_diff_index;
}
 
 int main (int argc, char** argv)
 {
   std::string pcd_name = std::string(argv[1]);
   float distanceThreshold = atof(argv[2]);
   float resolution = atof(argv[3]);
   int minCluster = atoi(argv[4]);
   int maxCluster = atoi(argv[5]);

   // Read in the cloud data
   pcl::PCDReader reader;
   
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  //  reader.read ("table_scene_lms400.pcd", *cloud);
   reader.read (pcd_name, *cloud);    
   std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*
 
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
   pcl::VoxelGrid<pcl::PointXYZ> vg;
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
   vg.setInputCloud (cloud);
   vg.setLeafSize (0.01f, 0.01f, 0.01f);
   vg.filter (*cloud_filtered);
   std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

   // Create the segmentation object for the planar model and set all the parameters
   pcl::SACSegmentation<pcl::PointXYZ> seg;
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
   pcl::PCDWriter writer;


   std::stringstream ss1;
   ss1 << "cloud_cluster_after_leaf_filter.pcd";
   writer.write<pcl::PointXYZ> (ss1.str (), *cloud_filtered, false); //*
 
   /*
   seg.setOptimizeCoefficients (true);
   seg.setModelType (pcl::SACMODEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setMaxIterations (100);
   seg.setDistanceThreshold (distanceThreshold);
 
   int nr_points = (int) cloud_filtered->size ();
   while (cloud_filtered->size () > 0.3 * nr_points)
   {
     // Segment the largest planar component from the remaining cloud
     seg.setInputCloud (cloud_filtered);
     seg.segment (*inliers, *coefficients);
     if (inliers->indices.size () == 0)
     {
       std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
       break;
     }
 
     // Extract the planar inliers from the input cloud
     pcl::ExtractIndices<pcl::PointXYZ> extract;
     extract.setInputCloud (cloud_filtered);
     extract.setIndices (inliers);
     extract.setNegative (false);
 
     // Get the points associated with the planar surface
     extract.filter (*cloud_plane);
     std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
 
     // Remove the planar inliers, extract the rest
     extract.setNegative (true);
     extract.filter (*cloud_f);
     *cloud_filtered = *cloud_f;
   }
   */

  //  std::stringstream ss2;
  //  ss2 << "cloud_cluster_21.pcd";
  //  writer.write<pcl::PointXYZ> (ss2.str (), *cloud_filtered, false); //*

   // Creating the KdTree object for the search method of the extraction
   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
   tree->setInputCloud (cloud_filtered);
 
   std::vector<pcl::PointIndices> cluster_indices;
   pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
   ec.setClusterTolerance (resolution); // 2cm
   ec.setMinClusterSize (minCluster); //50 works
   std::cout << "minCluster" << pcd_name << std::endl;
   ec.setMaxClusterSize (maxCluster);
   ec.setSearchMethod (tree);
   ec.setInputCloud (cloud_filtered);
   ec.extract (cluster_indices);

   std::vector<Pcl_Cluster> info_clusters;
   int j = 0;
   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
   {
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
     CloudPtr_XYZ cloud_cluster (new Cloud_XYZ);
     for (const auto& idx : it->indices)
       cloud_cluster->push_back ((*cloud_filtered)[idx]); //*
     cloud_cluster->width = cloud_cluster->size ();
     cloud_cluster->height = 1;
     cloud_cluster->is_dense = true;
 
     std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
     std::stringstream ss;
     ss << "cloud_cluster_" << j << ".pcd";
     writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
     j++;

     //Add information about each cluster
     Pcl_Cluster single_cluster;
     single_cluster.cluster = cloud_cluster;
     Eigen::Vector4f centroid;
     pcl::compute3DCentroid(*cloud_cluster, centroid);
     single_cluster.centroid_y = centroid[1];
     single_cluster.centroid_z = centroid[2];
     pcl::PointXYZ minPt, maxPt;
     pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);
     single_cluster.width = maxPt.x - minPt.x;
     single_cluster.height = maxPt.y - minPt.y;
     single_cluster.num_points = cloud_cluster->width;

     info_clusters.push_back(single_cluster);
   }

   // sort by X position
    // std::sort(info_clusters.begin(), info_clusters.end(), compareClusters);
    std::sort(info_clusters.begin(), info_clusters.end(), compareClustersY);
    std::stringstream gap;
    gap << "gap_cluster.pcd";
    writer.write<pcl::PointXYZ> (gap.str (), *info_clusters.back().cluster, false); //*
    CloudPtr_XYZ gap_cluster = info_clusters.back().cluster;
    pcl::PointXYZ gapMinPt, gapMaxPt;
    pcl::getMinMax3D (*gap_cluster, gapMinPt, gapMaxPt);
    // just need to return one thing here and that's the maxY and then we transform it
    float height = gapMaxPt.y - gapMinPt.y;
    std::cout << "Gap Size in Inches is " << height*39.3701 << std::endl;

    //Not Necessary
    /*

    float minY = info_clusters.front().centroid_y;
    float maxY = info_clusters.back().centroid_y;
    std::cout << "Min Y is " << minY << std::endl;
    std::cout << "Max Y is " << maxY << std::endl;

    std::sort(info_clusters.begin(), info_clusters.end(), compareClustersZ);
    std::cout << "Info Cluster Size is " << info_clusters.size() << std::endl;
    std::vector<float> compare(info_clusters.size());
    
    for (int i = 0; i < info_clusters.size(); ++i)
    {
      compare[i] = info_clusters[i].centroid_z;
      std::stringstream sorted;
      sorted << "cloud_cluster_sorted_" << i << ".pcd";
      std::cout << "Centroid Z is " << info_clusters[i].centroid_z << std::endl;
      writer.write<pcl::PointXYZ> (sorted.str (), *info_clusters[i].cluster, false); //*
    }

    int max_index = findMaxDiffIndex(compare);
    std::stringstream final;
    final << "gap_cluster_attempt.pcd";
    writer.write<pcl::PointXYZ> (final.str (), *info_clusters[max_index].cluster, false); //*
    */

   

  return (0);
}