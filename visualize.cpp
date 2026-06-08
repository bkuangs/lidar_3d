#include <iostream>
#include <memory>
#include <open3d/Open3D.h>
#include <vector>

int main(int argc, char **argv)
{
    const std::string path = argc > 1 ? argv[1] : "lidar_scan.ply";

    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    if (!open3d::io::ReadPointCloud(path, *pcd))
    {
        std::cerr << "Failed to read point cloud: " << path << '\n';
        return 1;
    }

    if (pcd->IsEmpty())
    {
        std::cerr << "Point cloud is empty: " << path << '\n';
        return 1;
    }

    auto frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(1.0);
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;
    geometries.push_back(pcd);
    geometries.push_back(frame);

    open3d::visualization::DrawGeometries(geometries, "LiDAR Scan");

    return 0;
}
