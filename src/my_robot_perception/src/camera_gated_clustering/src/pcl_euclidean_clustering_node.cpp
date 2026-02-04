#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "fusion_msgs/msg/enriched_cluster.hpp"
#include "fusion_msgs/msg/enriched_cluster_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl_conversions/pcl_conversions.h"

#include <queue>

class PclEuclideanClusteringNode : public rclcpp::Node {
public:
  PclEuclideanClusteringNode()
  : Node("pcl_euclidean_clustering_node"),
    cluster_id_counter_(0) {
  declare_parameter<std::string>("topics.lidar_points", "/ouster/points");
  declare_parameter<std::string>("clustering.external_topic", "/fusion/lidar_clusters");
  declare_parameter<std::string>("clustering.method", "euclidean");
  declare_parameter<double>("clustering.cluster_tolerance_m", 0.45);
  declare_parameter<int>("clustering.min_cluster_size", 40);
  declare_parameter<int>("clustering.max_cluster_size", 20000);
  declare_parameter<double>("clustering.voxel_leaf_size", 0.3);
  declare_parameter<double>("clustering.dbscan_eps_m", 0.3);
  declare_parameter<int>("clustering.dbscan_min_samples", 10);
  declare_parameter<std::string>("clustering.marker_topic", "/fusion/lidar_markers");
  declare_parameter<double>("clustering.marker_lifetime_sec", 0.2);

  input_topic_ = get_parameter("topics.lidar_points").as_string();
  output_topic_ = get_parameter("clustering.external_topic").as_string();

  method_ = get_parameter("clustering.method").as_string();
  cluster_tolerance_ = get_parameter("clustering.cluster_tolerance_m").as_double();
  min_cluster_size_ = get_parameter("clustering.min_cluster_size").as_int();
  max_cluster_size_ = get_parameter("clustering.max_cluster_size").as_int();
  voxel_leaf_size_ = get_parameter("clustering.voxel_leaf_size").as_double();
  dbscan_eps_ = get_parameter("clustering.dbscan_eps_m").as_double();
  dbscan_min_samples_ = get_parameter("clustering.dbscan_min_samples").as_int();
  marker_topic_ = get_parameter("clustering.marker_topic").as_string();
  marker_lifetime_sec_ = get_parameter("clustering.marker_lifetime_sec").as_double();

    cluster_pub_ = create_publisher<fusion_msgs::msg::EnrichedClusterArray>(
      output_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, 10);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PclEuclideanClusteringNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "PCL Euclidean clustering node started. input=%s output=%s",
                input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (voxel_leaf_size_ > 0.0) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(cloud);
      voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      voxel.filter(*filtered);
    } else {
      filtered = cloud;
    }

    if (filtered->empty()) {
      return;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(filtered);

    std::vector<pcl::PointIndices> cluster_indices;

    if (method_ == "dbscan") {
      const size_t n_points = filtered->size();
      std::vector<int> labels(n_points, -2);  // -2 unassigned, -1 noise
      std::vector<bool> visited(n_points, false);
      int cluster_id = 0;

      std::vector<int> nn_indices;
      std::vector<float> nn_dists;

      for (size_t i = 0; i < n_points; ++i) {
        if (visited[i]) {
          continue;
        }
        visited[i] = true;
        nn_indices.clear();
        nn_dists.clear();
  tree->radiusSearch((*filtered)[i], dbscan_eps_, nn_indices, nn_dists);

        if (static_cast<int>(nn_indices.size()) < dbscan_min_samples_) {
          labels[i] = -1;
          continue;
        }

        labels[i] = cluster_id;
        std::queue<int> q;
        for (int idx : nn_indices) {
          q.push(idx);
        }

        while (!q.empty()) {
          int j = q.front();
          q.pop();

          if (!visited[j]) {
            visited[j] = true;
            std::vector<int> nn_j;
            std::vector<float> nn_j_d;
            tree->radiusSearch((*filtered)[j], dbscan_eps_, nn_j, nn_j_d);
            if (static_cast<int>(nn_j.size()) >= dbscan_min_samples_) {
              for (int k : nn_j) {
                q.push(k);
              }
            }
          }

          if (labels[j] == -2 || labels[j] == -1) {
            labels[j] = cluster_id;
          }
        }

        cluster_id++;
      }

      cluster_indices.resize(cluster_id);
      for (size_t i = 0; i < n_points; ++i) {
        int label = labels[i];
        if (label >= 0 && label < cluster_id) {
          cluster_indices[label].indices.push_back(static_cast<int>(i));
        }
      }
    } else {
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(cluster_tolerance_);
      ec.setMinClusterSize(min_cluster_size_);
      ec.setMaxClusterSize(max_cluster_size_);
      ec.setSearchMethod(tree);
  ec.setInputCloud(filtered);
      ec.extract(cluster_indices);
    }

    fusion_msgs::msg::EnrichedClusterArray out_msg;
    out_msg.header = msg->header;
    out_msg.total_clusters = static_cast<int32_t>(cluster_indices.size());
    out_msg.camera_validated_count = 0;
    out_msg.lidar_only_count = out_msg.total_clusters;
    out_msg.average_fusion_confidence = 0.0f;

  std::vector<float> confidences;
  visualization_msgs::msg::MarkerArray marker_array;

    for (const auto &indices : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      cluster->reserve(indices.indices.size());
      for (int idx : indices.indices) {
        cluster->push_back((*filtered)[idx]);
      }

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster, centroid);

      pcl::PointXYZ min_pt, max_pt;
      pcl::getMinMax3D(*cluster, min_pt, max_pt);

      fusion_msgs::msg::EnrichedCluster cluster_msg;
      cluster_msg.header = msg->header;
      cluster_msg.cluster_id = cluster_id_counter_++;

      cluster_msg.centroid.x = centroid[0];
      cluster_msg.centroid.y = centroid[1];
      cluster_msg.centroid.z = centroid[2];

      cluster_msg.dimensions.x = max_pt.x - min_pt.x;
      cluster_msg.dimensions.y = max_pt.y - min_pt.y;
      cluster_msg.dimensions.z = max_pt.z - min_pt.z;

      cluster_msg.orientation.w = 1.0;
      cluster_msg.orientation.x = 0.0;
      cluster_msg.orientation.y = 0.0;
      cluster_msg.orientation.z = 0.0;

      cluster_msg.object_class = "lidar_cluster";
      cluster_msg.camera_confidence = 0.0f;
      cluster_msg.camera_validated = false;

      cluster_msg.point_count = static_cast<int32_t>(cluster->size());
      float distance = std::sqrt(
        centroid[0] * centroid[0] + centroid[1] * centroid[1] + centroid[2] * centroid[2]);
      cluster_msg.average_distance = distance;

      float lidar_confidence = std::min(1.0f, static_cast<float>(cluster->size()) / 1000.0f);
      cluster_msg.lidar_confidence = lidar_confidence;
      cluster_msg.fusion_confidence = lidar_confidence;
      cluster_msg.association_score = 0.0f;

      cluster_msg.bbox_2d_xmin = 0.0f;
      cluster_msg.bbox_2d_ymin = 0.0f;
      cluster_msg.bbox_2d_xmax = 0.0f;
      cluster_msg.bbox_2d_ymax = 0.0f;

      out_msg.clusters.push_back(cluster_msg);
      confidences.push_back(cluster_msg.fusion_confidence);

  visualization_msgs::msg::Marker marker;
  marker.header = msg->header;
  marker.ns = "lidar_clusters";
  marker.id = cluster_msg.cluster_id;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = cluster_msg.centroid.x;
  marker.pose.position.y = cluster_msg.centroid.y;
      marker.pose.position.z = cluster_msg.centroid.z;
  marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.03;
  marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.3f;
      marker.color.a = 0.35f;
  marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);

      const float l = cluster_msg.dimensions.x * 0.5f;
      const float w = cluster_msg.dimensions.y * 0.5f;
      const float h = cluster_msg.dimensions.z * 0.5f;

      geometry_msgs::msg::Point p0, p1, p2, p3, p4, p5, p6, p7;
      p0.x = cluster_msg.centroid.x - l; p0.y = cluster_msg.centroid.y - w; p0.z = cluster_msg.centroid.z - h;
      p1.x = cluster_msg.centroid.x + l; p1.y = cluster_msg.centroid.y - w; p1.z = cluster_msg.centroid.z - h;
      p2.x = cluster_msg.centroid.x + l; p2.y = cluster_msg.centroid.y + w; p2.z = cluster_msg.centroid.z - h;
      p3.x = cluster_msg.centroid.x - l; p3.y = cluster_msg.centroid.y + w; p3.z = cluster_msg.centroid.z - h;
      p4.x = cluster_msg.centroid.x - l; p4.y = cluster_msg.centroid.y - w; p4.z = cluster_msg.centroid.z + h;
      p5.x = cluster_msg.centroid.x + l; p5.y = cluster_msg.centroid.y - w; p5.z = cluster_msg.centroid.z + h;
      p6.x = cluster_msg.centroid.x + l; p6.y = cluster_msg.centroid.y + w; p6.z = cluster_msg.centroid.z + h;
      p7.x = cluster_msg.centroid.x - l; p7.y = cluster_msg.centroid.y + w; p7.z = cluster_msg.centroid.z + h;

      marker.points = {
        p0, p1, p1, p2, p2, p3, p3, p0,
        p4, p5, p5, p6, p6, p7, p7, p4,
        p0, p4, p1, p5, p2, p6, p3, p7
      };
  marker_array.markers.push_back(marker);
    }

    if (!confidences.empty()) {
      float sum = 0.0f;
      for (float v : confidences) {
        sum += v;
      }
      out_msg.average_fusion_confidence = sum / static_cast<float>(confidences.size());
    }

  cluster_pub_->publish(out_msg);
  marker_pub_->publish(marker_array);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string method_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  double voxel_leaf_size_;
  double dbscan_eps_;
  int dbscan_min_samples_;
  std::string marker_topic_;
  double marker_lifetime_sec_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<fusion_msgs::msg::EnrichedClusterArray>::SharedPtr cluster_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  int32_t cluster_id_counter_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PclEuclideanClusteringNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
