#pragma once

#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>

namespace hologram {

// Custom PCL point type that includes confidence
struct PointXYZRGBNC {
    PCL_ADD_POINT4D;     // Adds x,y,z,padding
    PCL_ADD_RGB;         // Adds rgb
    PCL_ADD_NORMAL4D;    // Adds normal[3]
    float confidence;    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Ensures proper alignment
} EIGEN_ALIGN16;         // Enforces SSE padding

// Register the custom point type
POINT_CLOUD_REGISTER_POINT_STRUCT(hologram::PointXYZRGBNC,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, rgb, rgb)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, confidence, confidence)
)

class PointCloud {
public:
    using Ptr = std::shared_ptr<PointCloud>;
    using ConstPtr = std::shared_ptr<const PointCloud>;
    using CloudType = pcl::PointCloud<PointXYZRGBNC>;
    
    PointCloud();
    ~PointCloud() = default;
    
    // Basic operations
    void addPoint(const PointXYZRGBNC& point);
    void addPoints(const std::vector<PointXYZRGBNC>& points);
    void clear();
    size_t size() const;
    bool empty() const;
    
    // Access
    CloudType::Ptr getCloud() { return cloud; }
    CloudType::ConstPtr getCloud() const { return cloud; }
    pcl::KdTreeFLANN<PointXYZRGBNC>::Ptr getKdTree();
    pcl::octree::OctreePointCloudSearch<PointXYZRGBNC>::Ptr getOctree(float resolution = 0.01f);
    
    // Transformations
    void transform(const Eigen::Matrix4f& matrix);
    void translate(const Eigen::Vector3f& translation);
    void rotate(const Eigen::Quaternionf& rotation);
    void scale(const Eigen::Vector3f& scale);
    
    // File operations
    bool loadFromPLY(const std::string& filename);
    bool saveToPLY(const std::string& filename) const;
    
    // Spatial queries
    std::vector<PointXYZRGBNC> getPointsInRadius(const Eigen::Vector3f& center, float radius) const;
    std::vector<PointXYZRGBNC> getPointsInBox(const Eigen::Vector3f& min_pt, const Eigen::Vector3f& max_pt) const;
    
    // Statistics
    float getAverageSpacing() const;
    Eigen::Vector3f getCentroid() const;
    float getDensity() const;
    
    // Modification
    void subsample(float leaf_size);
    void removeOutliers(float radius, int min_neighbors);
    void estimateNormals(float radius);
    
    // Conversion utilities
    static PointXYZRGBNC glmToPoint(const glm::vec3& position, const glm::vec3& color = glm::vec3(1.0f),
                                   const glm::vec3& normal = glm::vec3(0.0f, 0.0f, 1.0f), float confidence = 1.0f);
    static glm::vec3 pointToGlm(const PointXYZRGBNC& point);
    
private:
    CloudType::Ptr cloud;
    pcl::KdTreeFLANN<PointXYZRGBNC>::Ptr kdtree;
    pcl::octree::OctreePointCloudSearch<PointXYZRGBNC>::Ptr octree;
    float octreeResolution;
    
    void buildKdTree();
    void buildOctree(float resolution);
};

} // namespace hologram 