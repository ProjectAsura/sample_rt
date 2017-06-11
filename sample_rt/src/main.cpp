﻿//-------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------
// Global Varaibles.
//-------------------------------------------------------------------------------------------------
const int     g_max_depth = 5;
const Vector3 g_light_pos   (50.0,  75.0,   81.6);
const Vector3 g_light_color (256.0, 256.0, 256.0);
const Vector3 g_back_ground (0.0,   0.0,    0.0);
const Vector3 g_shadow_color(0.0,   0.0,    0.0);
const Sphere  g_spheres[] = {
    Sphere(1e5,     Vector3( 1e5 + 1.0,    40.8,          81.6), Vector3(0.25,  0.75,  0.25), ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(-1e5 + 99.0,   40.8,          81.6), Vector3(0.25,  0.25,  0.75), ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(50.0,          40.8,           1e5), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(50.0,          40.8,  -1e5 + 170.0), Vector3(),                   ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(50.0,           1e5,          81.6), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse),
    Sphere(1e5,     Vector3(50.0,   -1e5 + 81.6,          81.6), Vector3(0.75,  0.75,  0.75), ReflectionType::Diffuse),
    Sphere(16.5,    Vector3(27.0,          16.5,          47.0), Vector3(0.75,  0.25,  0.25), ReflectionType::Specular),
    Sphere(16.5,    Vector3(73.0,          16.5,          78.0), Vector3(0.99,  0.99,  0.99), ReflectionType::Refraction)
};


//-------------------------------------------------------------------------------------------------
//      シーンとの交差判定を行います.
//-------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------
//      放射輝度を求めます.
//-------------------------------------------------------------------------------------------------
Vector3 radiance(const Ray& ray, int depth)
{
    double t;
    int   id;

    // シーンとの交差判定.
    if (!intersect_scene(ray, &t, &id))
    { return g_back_ground; }

    const auto& obj = g_spheres[id];
    const auto hit_pos = ray.pos + ray.dir * t;
    const auto normal  = normalize(hit_pos - obj.pos);
    const auto orienting_normal = (dot(normal, ray.dir) < 0.0) ? normal : -normal;

    if(depth > g_max_depth)
    { return g_back_ground; }

    switch (obj.type)
    {
    case ReflectionType::Diffuse:
        {
            double t_;
            int    id_;

            // ライトベクトル.
            auto light_dir  = g_light_pos - hit_pos;

            // ライトまでの距離.
            auto light_dist = length(light_dir);

            // ライトベクトルを正規化.
            light_dir /= light_dist;

            // ライトとの間に遮蔽物がないことを確認.
            intersect_scene(Ray(hit_pos, light_dir), &t_, &id_);

            // 遮蔽物がない場合.
            if (t_ >= light_dist)
            {
                auto diffuse = obj.color * std::max(dot(orienting_normal, light_dir), 0.0);
                return g_light_color * diffuse / (light_dist * light_dist);
            }
            else
            {
                return g_shadow_color;
            }
        }
        break;

    case ReflectionType::PerfecctSpecular:
        {
            return obj.color * radiance(Ray(hit_pos, reflect(ray.dir, normal)), depth + 1);
        }
        break;

    case ReflectionType::Refraction:
        {
            // 反射ベクトル.
            auto reflect_ray = Ray(hit_pos, reflect(ray.dir, normal));

            // 内部侵入するか?
            auto into = dot(normal, orienting_normal) > 0.0;

            // 空気の屈折率
            const auto nc = 1.0;

            // 物体の屈折率
            const auto nt = 1.5;

            const auto nnt = (into) ? (nc / nt) : (nt / nc);
            const auto vn = dot(ray.dir, orienting_normal);
            const auto cos2t = 1.0 - nnt * nnt * (1.0 - vn * vn);

            // 全反射かどうかチェック.
            if (cos2t < 0.0)
            { return obj.color * radiance(reflect_ray, depth + 1); }

            // 屈折ベクトル.
            auto refract = normalize(ray.dir * nnt - normal * ((into) ? 1.0 : -1.0) * (vn * nnt + sqrt(cos2t)) );

            // Schlickによる Fresnel の反射係数の近似.
            const auto a = nt - nc;
            const auto b = nt + nc;
            const auto R0 = (a * a) / (b * b);

            const auto c = 1.0 - ((into) ? -vn : dot(refract, normal));
            const auto Re = R0 + (1.0 - R0) * pow(c, 5.0);

            const auto nnt2 = pow((into) ? (nc / nt) : (nt /nc), 2.0);
            const auto Tr = (1.0 - Re) * nnt2;
            const auto p = 0.25 + 0.5 * Re;

            const auto reflect_result = radiance(reflect_ray, depth + 1) * Re;
            const auto refract_result = radiance(Ray(hit_pos, refract), depth +1) * Tr;

            return obj.color * (reflect_result + refract_result);
        }
        break;

    case ReflectionType::Specular:
        {
            double t_;
            int    id_;

            // ライトベクトル.
            auto light_dir  = g_light_pos - hit_pos;

            // ライトまでの距離.
            auto light_dist = length(light_dir);

            // ライトベクトルを正規化.
            light_dir /= light_dist;

            // ライトまでの間に遮蔽物がないかどうかチェック.
            intersect_scene(Ray(hit_pos, light_dir), &t_, &id_);
            if (t_ >= light_dist)
            {
                auto shininess  = 500.0;
                auto diffuse    = (obj.color / (light_dist * light_dist)) * std::max(dot(orienting_normal, light_dir), 0.0);
                auto specular   = (obj.color) * pow(std::max(dot(light_dir, reflect(ray.dir, orienting_normal)), 0.0), shininess);
                return g_light_color * (diffuse + specular);
            }
            else
            {
                return g_shadow_color;
            }
        }
        break;
    }

    return g_back_ground;
}

} // namespace


//-------------------------------------------------------------------------------------------------
//      メインエントリーポイントです.
//-------------------------------------------------------------------------------------------------
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
    { image[i] = g_back_ground; }

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