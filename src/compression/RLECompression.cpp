#include "hologram/compression/RLECompression.hpp"
#include <cstring>

void RLECompression::compress(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<char>& compressedData) {
    std::vector<Point> points;
    points.reserve(cloud->points.size());
    
    // Convert PCL points to our internal format
    for (const auto& p : cloud->points) {
        points.push_back({p.x, p.y, p.z, static_cast<uint8_t>(p.r), static_cast<uint8_t>(p.g), static_cast<uint8_t>(p.b)});
    }
    
    encodeRLE(points, compressedData);
}

void RLECompression::decompress(const std::vector<char>& compressedData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    std::vector<Point> points;
    decodeRLE(compressedData, points);
    
    cloud->points.clear();
    cloud->points.reserve(points.size());
    
    // Convert back to PCL points
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
    cloud->is_dense = true;
}

void RLECompression::encodeRLE(const std::vector<Point>& points, std::vector<char>& compressed) {
    if (points.empty()) {
        compressed.clear();
        return;
    }
    
    // First store the total number of points
    size_t numPoints = points.size();
    compressed.resize(sizeof(size_t));
    memcpy(compressed.data(), &numPoints, sizeof(size_t));
    
    Point lastPoint = points[0];
    size_t count = 1;
    
    for (size_t i = 1; i < points.size(); ++i) {
        const Point& currentPoint = points[i];
        
        // If points are similar (within a small epsilon) or we've reached max count
        if (count < 255 &&
            std::abs(currentPoint.x - lastPoint.x) < 0.0001f &&
            std::abs(currentPoint.y - lastPoint.y) < 0.0001f &&
            std::abs(currentPoint.z - lastPoint.z) < 0.0001f &&
            currentPoint.r == lastPoint.r &&
            currentPoint.g == lastPoint.g &&
            currentPoint.b == lastPoint.b) {
            count++;
        } else {
            // Store the run
            compressed.push_back(static_cast<char>(count));
            compressed.resize(compressed.size() + sizeof(Point));
            memcpy(&compressed[compressed.size() - sizeof(Point)], &lastPoint, sizeof(Point));
            
            lastPoint = currentPoint;
            count = 1;
        }
    }
    
    // Store the last run
    compressed.push_back(static_cast<char>(count));
    compressed.resize(compressed.size() + sizeof(Point));
    memcpy(&compressed[compressed.size() - sizeof(Point)], &lastPoint, sizeof(Point));
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
        // Read count and point
        uint8_t count = static_cast<uint8_t>(compressed[pos++]);
        Point point;
        memcpy(&point, &compressed[pos], sizeof(Point));
        pos += sizeof(Point);
        
        // Replicate the point count times
        for (uint8_t i = 0; i < count; ++i) {
            points.push_back(point);
        }
    }
} 