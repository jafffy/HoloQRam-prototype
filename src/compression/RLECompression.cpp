#include "hologram/compression/RLECompression.hpp"
#include <cstring>
#include <cmath>

namespace hologram {

void RLECompression::compress(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<char>& compressedData) {
    std::vector<Point> points;
    points.reserve(cloud->points.size());
    
    // Convert PCL points to our internal format
    for (const auto& p : cloud->points) {
        points.push_back({
            p.x, p.y, p.z,
            static_cast<uint8_t>(p.r),
            static_cast<uint8_t>(p.g),
            static_cast<uint8_t>(p.b)
        });
    }
    
    // Perform RLE compression
    encodeRLE(points, compressedData);
}

void RLECompression::decompress(const std::vector<char>& compressedData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    std::vector<Point> points;
    decodeRLE(compressedData, points);
    
    // Create a new cloud if needed
    if (!cloud) {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }
    
    // Convert back to PCL format
    cloud->points.clear();
    cloud->points.reserve(points.size());
    
    for (const auto& p : points) {
        pcl::PointXYZRGB point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.r = p.r;
        point.g = p.g;
        point.b = p.b;
        cloud->points.push_back(point);
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
}

void RLECompression::encodeRLE(const std::vector<Point>& points, std::vector<char>& compressed) {
    if (points.empty()) {
        compressed.clear();
        return;
    }
    
    // Reserve space for the total number of points
    compressed.resize(sizeof(size_t));
    size_t numPoints = points.size();
    memcpy(compressed.data(), &numPoints, sizeof(size_t));
    
    Point lastPoint = points[0];
    uint32_t runLength = 1;
    
    for (size_t i = 1; i < points.size(); ++i) {
        const Point& currentPoint = points[i];
        
        // Check if points are equal (within epsilon for floating point values)
        bool pointsEqual = 
            std::abs(currentPoint.x - lastPoint.x) < 0.0001f &&
            std::abs(currentPoint.y - lastPoint.y) < 0.0001f &&
            std::abs(currentPoint.z - lastPoint.z) < 0.0001f &&
            currentPoint.r == lastPoint.r &&
            currentPoint.g == lastPoint.g &&
            currentPoint.b == lastPoint.b;
        
        if (pointsEqual && runLength < UINT32_MAX) {
            runLength++;
        } else {
            // Store the run length and point
            size_t offset = compressed.size();
            compressed.resize(offset + sizeof(uint32_t) + sizeof(Point));
            memcpy(&compressed[offset], &runLength, sizeof(uint32_t));
            memcpy(&compressed[offset + sizeof(uint32_t)], &lastPoint, sizeof(Point));
            
            lastPoint = currentPoint;
            runLength = 1;
        }
    }
    
    // Store the last run
    size_t offset = compressed.size();
    compressed.resize(offset + sizeof(uint32_t) + sizeof(Point));
    memcpy(&compressed[offset], &runLength, sizeof(uint32_t));
    memcpy(&compressed[offset + sizeof(uint32_t)], &lastPoint, sizeof(Point));
}

void RLECompression::decodeRLE(const std::vector<char>& compressed, std::vector<Point>& points) {
    if (compressed.size() < sizeof(size_t)) {
        points.clear();
        return;
    }
    
    // Read total number of points
    size_t numPoints;
    memcpy(&numPoints, compressed.data(), sizeof(size_t));
    points.reserve(numPoints);
    
    size_t pos = sizeof(size_t);
    while (pos < compressed.size()) {
        uint32_t runLength;
        Point point;
        
        memcpy(&runLength, &compressed[pos], sizeof(uint32_t));
        pos += sizeof(uint32_t);
        
        memcpy(&point, &compressed[pos], sizeof(Point));
        pos += sizeof(Point);
        
        // Expand the run
        for (uint32_t i = 0; i < runLength; ++i) {
            points.push_back(point);
        }
    }
}

} // namespace hologram 