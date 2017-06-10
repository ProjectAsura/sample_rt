//-------------------------------------------------------------------------------------------------
// File : main.cpp
// Desc : Main Entry Point.
// Copyright(c) Project Asura. All right reserved.
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Inlcudes
//-------------------------------------------------------------------------------------------------
#include <r3d_math.h>
#include <r3d_camera.h>
#include <r3d_shape.h>
#include <r3d_image.h>
#include <vector>
#include <algorithm>


namespace {

Sphere g_spheres[] = {
    Sphere(1e5,     Vector3( 1e5 + 1.0,    40.8,          81.6), Vector3(0.75,  0.25,  0.25), ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(-1e5 + 99.0,   40.8,          81.6), Vector3(0.25,  0.25,  0.75), ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(50.0,          40.8,           1e5), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(50.0,          40.8,  -1e5 + 170.0), Vector3(),                   ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(50.0,           1e5,          81.6), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(50.0,   -1e5 + 81.6,          81.6), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse),
    Sphere(16.5,    Vector3(27.0,          16.5,          47.0), Vector3(0.99,  0.99,  0.99), ReflectionType::PerfecctSpecular),
    Sphere(16.5,    Vector3(73.0,          16.5,          78.0), Vector3(0.99,  0.99,  0.99), ReflectionType::Refraction)
};

const Vector3 g_light_pos(50.0, 75.0, 81.6);
const Vector3 g_light_color(256.0, 256.0, 256.0);

inline bool intersect_scene(const Ray& ray, double* t, int* id)
{
    auto n = static_cast<int>(sizeof(g_spheres) / sizeof(g_spheres[0]));

    *t  = D_MAX;
    *id = -1;

    for (auto i = 0; i < n; ++i)
    {
        auto d = g_spheres[i].intersect(ray);
        if (d > D_HIT_MIN && d < *t)
        {
            *t  = d;
            *id = i;
        }
    }

    return *t < D_HIT_MAX;
}

} // namespace

Vector3 radiance(const Ray& ray, int depth)
{
    double t;
    int   id;

    if (!intersect_scene(ray, &t, &id))
    { return Vector3(0.0, 0.0, 0.0); }

    const auto& obj = g_spheres[id];
    const auto hit_pos = ray.pos + ray.dir * t;
    const auto normal  = normalize(hit_pos - obj.pos);
    const auto orienting_normal = (dot(normal, ray.dir) < 0.0) ? normal : -normal;

    if(depth > 5)
    { return Vector3(0.0, 0.0, 0.0); }

    switch (obj.type)
    {
    case ReflectionType::Diffuse:
        {
            double t_;
            int id_;
            const auto light_dir = g_light_pos - hit_pos;
            const auto mag = length(light_dir);

            intersect_scene(Ray(hit_pos, light_dir / mag), &t_, &id_);
            if (t_ >= mag)
            {
                auto nl = std::max(dot(orienting_normal, light_dir / mag), 0.0);
                auto diffuse = (obj.color / (mag * mag)) * nl;
                return g_light_color * diffuse;
            }
            else
            {
                return Vector3(0.0, 0.0, 0.0);
            }
        }
        break;

    case ReflectionType::PerfecctSpecular:
        {
            return obj.color * radiance(Ray(hit_pos, ray.dir - normal * 2.0 * dot(normal, ray.dir)), depth + 1);
        }
        break;

    case ReflectionType::Refraction:
        {
            auto reflect = Ray(hit_pos, ray.dir - normal * 2.0 * dot(normal, ray.dir));
            auto into = dot(normal, orienting_normal) > 0.0;

            const auto nc = 1.0;
            const auto nt = 1.5;
            const auto nnt = (into) ? nc / nt : nt / nc;
            const auto ddn = dot(ray.dir, orienting_normal);
            const auto cos2t = 1.0 - nnt * nnt * (1.0 - ddn * ddn);

            if (cos2t < 0.0)
            {
                return obj.color * radiance(reflect, depth + 1);
            }

            auto refract = normalize(ray.dir * nnt - normal * ((into) ? 1.0 : -1.0) * (ddn * nnt + sqrt(cos2t)) );

            const auto a = nt - nc;
            const auto b = nt + nc;
            const auto R0 = (a * a) / (b * b);
            const auto c = 1.0 - (into ? -ddn : dot(refract, normal));
            const auto Re = R0 + (1.0 - R0) * pow(c, 5.0);
            const auto Tr = 1.0 - Re;
            const auto p = 0.25 + 0.5 * Re;

            return obj.color * (radiance(reflect, depth + 1) * Re + radiance(Ray(hit_pos, refract), depth + 1) * Tr);
        }
        break;

    case ReflectionType::Specular:
        {
            double t_;
            int id_;
            const auto light_dir = g_light_pos - hit_pos;
            const auto mag = length(light_dir);

            intersect_scene(Ray(hit_pos, light_dir / mag), &t_, &id_);
            if (t_ >= mag)
            {
                auto nl = std::max(dot(orienting_normal, light_dir / mag), 0.0);
                auto diffuse = (obj.color / (mag * mag)) * nl;
                return g_light_color * diffuse;
            }
            else
            {
                return Vector3(0.0, 0.0, 0.0);
            }
        }
        break;
    }

    return Vector3(0.0, 0.0, 0.0);
}

int main(int argc, char** argv)
{
    int width  = 640;
    int height = 480;

    Camera camera2(
        Vector3(50.0, 52.0, 295.6),
        normalize(Vector3(0.0, -0.042612, -1.0)),
        Vector3(0.0, 1.0, 0.0),
        0.5135,
        double(width) / double(height),
        130.0
    );

    std::vector<Vector3> image;
    image.resize(width * height);

    for (size_t i = 0; i < image.size(); ++i)
    { image[i] = Vector3(0.0, 0.0, 0.0); }

    for (auto y = 0; y < height; ++y)
    {
        for (auto x = 0; x < width; ++x)
        {
            auto idx = y * width + x;
            auto fx = double(x) / double(width)  - 0.5;
            auto fy = double(y) / double(height) - 0.5;
            image[idx] += radiance(camera2.emit(fx, fy), 0);
        }
    }

    save_bmp("image.bmp", width, height, &image.data()->x);
    image.clear();

    return 0;
}