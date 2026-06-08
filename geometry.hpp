#pragma once
#include <Eigen/Dense>
#include <memory>

using Vec3 = Eigen::Vector3d;
namespace geom {
    inline constexpr double Epsilon = 1e-9;
}

struct Ray
{
    Vec3 ori;
    Vec3 dir;
};

struct Hit
{
    bool hit = false;
    double t = -1.0;
    Vec3 p = Vec3::Zero();
    Vec3 n = Vec3::Zero();
    double intensity = 0.0;
    int objId = -1;
};

/////////////////////////////////////////////////////
//// BASE GEOMETRY CLASS
//// explicitly define public + virtual destructor whenever a base class has virtual functions
//// if not, derived objects will not be deleted when the base class destructor is called
//// virtual ensures deletion looks up correct destructor in vtable
/////////////////////////////////////////////////////
class Geometry
{
public:
    virtual ~Geometry() = default;
    virtual Hit intersect(const Ray &ray) const = 0; // "= 0" -> pure virtual function, derived classes MUST override 
};

/////////////////////////////////////////////////////
//// DERIVED GEOMETRY CLASSES
/////////////////////////////////////////////////////

class SphereGeometry : public Geometry
{
public:
    SphereGeometry(Vec3 center, double radius, int objId)
        : center_(center), radius_(radius), objId_(objId) {}

    Hit intersect(const Ray &ray) const override
    {
        Hit hit;

        Vec3 v = ray.ori - center_;

        double a = ray.dir.dot(ray.dir);
        double b = 2.0 * v.dot(ray.dir);
        double c = v.dot(v) - (radius_ * radius_);

        double delta = (b * b) - 4.0 * a * c;
        if (delta < 0.0)
            return Hit{};

        double t = ((-1.0 * b) - sqrt(delta)) / (2.0 * a);
        if (t <= geom::Epsilon)
            return Hit{};

        Vec3 p = ray.ori + t * ray.dir;
        Vec3 n = (p - center_).normalized();

        hit = Hit{true, t, p, n, 1.0, objId_};

        return hit;
    }

private:
    Vec3 center_;
    double radius_;
    int objId_;
};

class PlaneGeometry : public Geometry
{
public:
    PlaneGeometry(Vec3 normal, Vec3 point, int objId)
        : normal_(normal), point_(point), objId_(objId) {};

    Hit intersect(const Ray &ray) const override
    {
        Hit hit;

        double denominator = ray.dir.dot(normal_);
        if (std::abs(denominator) < geom::Epsilon)
            return Hit{};

        double t = normal_.dot(point_ - ray.ori) / denominator;
        if (t <= geom::Epsilon)
            return Hit{};

        Vec3 hitP = ray.ori + t * ray.dir;

        hit = Hit{true, t, hitP, normal_, 1.0, objId_};

        return hit;
    }

private:
    Vec3 normal_;
    Vec3 point_;
    int objId_;
};

/*
class TriangleMeshGeometry : public Geometry
{
public:
    Hit intersect(const Ray &ray) const override
    {
        // test ray against triangles
    }

private:
    std::vector<Vec3> vertices_;
    std::vector<Eigen::Vector3i> triangles_;
};
*/