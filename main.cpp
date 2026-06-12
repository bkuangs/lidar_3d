#include "geometry.hpp"
#include "lidar.hpp"
#include "scene.hpp"
#include "scan.hpp"
#include "hallway.hpp"
#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <thread>
#include <chrono>
#include <algorithm>

void updatePointCloud(const ScanResult &scan, open3d::geometry::PointCloud &pcd)
{
    pcd.points_.clear();
    pcd.colors_.clear();

    pcd.points_.reserve(scan.points.size());
    pcd.colors_.reserve(scan.points.size());

    for (const ScanPoint &point : scan.points)
    {
        double c = 0.25 + 0.75 * point.intensity;

        pcd.points_.push_back(point.point_world);
        pcd.colors_.push_back(Vec3(c, c, c));
    }
}

void updateFirstPersonCamera(
    open3d::visualization::Visualizer &vis,
    const Lidar &lidar)
{
    constexpr double lookahead = 8.0;

    const Vec3 eye = lidar.pose.translation();
    const Vec3 forward = lidar.pose.rotation() * Vec3(1.0, 0.0, 0.0);
    const Vec3 up = lidar.pose.rotation() * Vec3(0.0, 0.0, 1.0);

    auto &view = vis.GetViewControl();
    view.SetLookat(eye + lookahead * forward);
    view.SetFront(forward);
    view.SetUp(up);
}

void runSimulation()
{
    HallwayWorld world = makeHallwayWorld();
    Lidar lidar;
    lidar.pose.translation() = Vec3(0.0, 0.0, 1.2);
    updateObstacleWindow(world, lidar.pose.translation().x());
    auto scene = makeHallwayScene(world);

    double speed = 1.0; // m/s
    double dt = 1.0 / 30.0;

    bool exit_viewer = false;

    open3d::visualization::VisualizerWithKeyCallback vis;
    vis.CreateVisualizerWindow("LiDAR hallway", 1280, 720);

    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    ScanResult scan = scanScene(scene, lidar);
    updatePointCloud(scan, *pcd);
    vis.AddGeometry(pcd);

    bool fpv_camera = true;

    vis.RegisterKeyCallback(
        GLFW_KEY_ESCAPE,
        [&exit_viewer](open3d::visualization::Visualizer *) {
            exit_viewer = true;
            return false;
        });

    vis.RegisterKeyCallback(
        GLFW_KEY_F,
        [&fpv_camera](open3d::visualization::Visualizer *) {
            fpv_camera = !fpv_camera;
            return false;
        });

    while (!exit_viewer)
    {
        if (!vis.PollEvents())
            break; // window close button was clicked

        lidar.pose.translation().x() += speed * dt;
        updateObstacleWindow(world, lidar.pose.translation().x());
        scene = makeHallwayScene(world);

        scan = scanScene(scene, lidar);
        updatePointCloud(scan, *pcd);

        if (fpv_camera)
            updateFirstPersonCamera(vis, lidar);

        vis.UpdateGeometry(pcd);
        vis.UpdateRender();

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
}

int main()
{
    runSimulation();

    return 0;
}
