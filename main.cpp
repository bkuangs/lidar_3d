#include "geometry.hpp"
#include "lidar.hpp"
#include "scene.hpp"
#include <cmath>
#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <algorithm>
#include <array>
#include <iostream>
#include <map>

auto pcd = std::make_shared<open3d::geometry::PointCloud>();
constexpr double pi = 3.14159265358979323846;
constexpr double deg = pi / 180.0;

/////////////////////////////////////////////////////
//// init scene and lidar
/////////////////////////////////////////////////////
Scene makeScene()
{
    Scene scene;

    scene.objects.push_back(std::make_shared<PlaneGeometry>(
        Vec3(0, 0, 1), Vec3(0, 0, 0), 0));

    scene.objects.push_back(std::make_shared<SphereGeometry>(
        Vec3(5.0, 0.0, 1.2), 0.75, 1));

    scene.objects.push_back(std::make_shared<SphereGeometry>(
        Vec3(0.0, 5.0, 1.8), 0.8, 2));

    scene.objects.push_back(std::make_shared<SphereGeometry>(
        Vec3(-5.0, 0.0, 0.8), 0.7, 3));

    scene.objects.push_back(std::make_shared<SphereGeometry>(
        Vec3(0.0, -5.0, 1.4), 0.6, 4));

    scene.objects.push_back(std::make_shared<SphereGeometry>(
        Vec3(3.5, -3.5, 2.5), 0.5, 5));

    return scene;
}

Lidar makeLidar()
{
    Lidar lidar;

    lidar.pose = Eigen::Isometry3d::Identity();
    lidar.pose.translation() = Vec3(0.0, 0.0, 1.2); // lidar origin

    lidar.minRange = 0.1;
    lidar.maxRange = 50.0;

    lidar.minAzimuth = 0.0;
    lidar.maxAzimuth = 2.0 * pi;
    lidar.azimuthSamples = 720;

    lidar.minElevation = -30.0 * deg;
    lidar.maxElevation = 30.0 * deg;
    lidar.elevationSamples = 128;

    return lidar;
}

/////////////////////////////////////////////////////
//// convert lidar scan angles to world coordinates
/////////////////////////////////////////////////////
Vec3 scanDir(double azimuth, double elevation)
{
    double x = std::cos(elevation) * std::cos(azimuth);
    double y = std::cos(elevation) * std::sin(azimuth);
    double z = std::sin(elevation);

    return Vec3(x, y, z).normalized();
}

Vec3 colorForObject(int objId, double intensity)
{
    static const std::array<Vec3, 6> palette{
        Vec3(0.55, 0.55, 0.55),
        Vec3(0.95, 0.22, 0.18),
        Vec3(0.18, 0.55, 0.95),
        Vec3(0.18, 0.72, 0.35),
        Vec3(0.95, 0.72, 0.16),
        Vec3(0.72, 0.28, 0.95),
    };

    Vec3 base = palette[std::clamp(objId, 0, static_cast<int>(palette.size()) - 1)];
    return (0.25 + 0.75 * intensity) * base;
}

int main()
{
    auto scene = makeScene();
    auto lidar = makeLidar();
    std::map<int, int> hitCounts;
    int totalRays = 0;

    for (int v = 0; v < lidar.elevationSamples; ++v)
    {
        // progress through scans (fraction)
        double vf = static_cast<double>(v) / (lidar.elevationSamples - 1);

        // elevation angle
        double elevation =
            lidar.minElevation +
            vf * (lidar.maxElevation - lidar.minElevation);

        for (int h = 0; h < lidar.azimuthSamples; ++h)
        {
            ++totalRays;
            double hf = static_cast<double>(h) / lidar.azimuthSamples;

            // horizontal angle
            double azimuth =
                lidar.minAzimuth +
                hf * (lidar.maxAzimuth - lidar.minAzimuth);

            Vec3 localDir = scanDir(azimuth, elevation);

            Ray ray;

            // ray starts at lidar pose
            ray.ori = lidar.pose.translation();

            // convert ray direction to world frame
            ray.dir = lidar.pose.rotation() * localDir;

            Hit hit = scene.intersect(ray);

            if (hit.hit && hit.t >= lidar.minRange && hit.t <= lidar.maxRange)
            {
                Vec3 point = ray.ori + hit.t * ray.dir;

                // add point to point cloud
                pcd->points_.push_back(Eigen::Vector3d(
                    point.x(),
                    point.y(),
                    point.z()));

                double intensity = std::clamp(hit.n.dot(-ray.dir), 0.0, 1.0);
                pcd->colors_.push_back(colorForObject(hit.objId, intensity));
                ++hitCounts[hit.objId];
            }
        }
    }

    open3d::io::WritePointCloud("lidar_scan.ply", *pcd);
    std::cout << "Wrote " << pcd->points_.size() << " points from "
              << totalRays << " rays to lidar_scan.ply\n";
    for (const auto &[objId, count] : hitCounts)
        std::cout << "  object " << objId << ": " << count << " hits\n";

    return 0;
}
