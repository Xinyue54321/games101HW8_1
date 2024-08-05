//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { DIFFUSE,MICROFACET};

class Material{
private:
    
    
    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {//计算反射方向
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {//折射方向
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {//计算反射和折射的能量比例
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    //发光
    float ior;
    float roughness;
    //粗糙程度
    Vector3f Kd, Ks;
    float specularExponent;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0), float a =0);
    inline MaterialType getType();//材质类型
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);//没写
    inline Vector3f getEmission();//返回光强
    inline bool hasEmission();//发光吗？

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);//半球采样
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);//半球均匀采样的pdf
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

    inline float DistributionGGX(const Vector3f N, const Vector3f H, const float a);
    inline float GeometrySchlickGGX(const float NdotV, const float k);
    inline float GeometrySmith(const Vector3f N, const Vector3f V, const Vector3f L, const float roughness);
};

Material::Material(MaterialType t, Vector3f e, float a) {
    m_type = t;
    //m_color = c;
    m_emission = e;
    roughness = a;
    ior = 1.6585;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {//是否发光
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);//(-1,1)生成一个随机的cos(theta)
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;//计算sin(theta)，随机生成一个角度phi，theta和phi是三维极坐标系的两个角度
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);//生成随机的方向
            return toWorld(localRay, N);//转换为世界坐标系
            
            break;
        }
        case MICROFACET:
        {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);//(-1,1)生成一个随机的cos(theta)
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;//计算sin(theta)，随机生成一个角度phi，theta和phi是三维极坐标系的两个角度
            Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);//生成随机的方向
            return toWorld(localRay, N);//转换为世界坐标系

            break;
        }
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){//半球均匀采样的pdf
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
        case MICROFACET:
        {
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
    }
}

float Material::DistributionGGX(const Vector3f N, const Vector3f H, const float a) {
    float a2 = a * a;
    float NdotH = std::max(dotProduct(N, H), 0.0f);
    float NdotH2 = NdotH * NdotH;

    float nom = a2;
    float denom = (NdotH * (a2 - 1.0f) + 0.1f);
    denom = M_PI * denom * denom;
    
    return nom / denom;
}

float Material::GeometrySchlickGGX(const float NdotV, const float k)
{
    float nom = NdotV;
    float denom = NdotV * (1.0f - k) + k;

    return nom / denom;
}

float Material::GeometrySmith(const Vector3f N, const Vector3f V, const Vector3f L, const float roughness)
{
    float r = roughness + 1;
    float k = r * r / 8.0f;
    float NdotV = std::max(dotProduct(N, V), 0.0f);
    float NdotL = std::max(dotProduct(N, L), 0.0f);
    float ggx1 = GeometrySchlickGGX(NdotV, k);
    float ggx2 = GeometrySchlickGGX(NdotL, k);

    return ggx1 * ggx2;
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){//计算BRDF
    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse   model
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / M_PI;//？
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
        case MICROFACET:
        {
            if (dotProduct(wo, N) > 0.0f) {
                Vector3f h = (wo + wi).normalized();
                float D = DistributionGGX(N, h, roughness);
                float G = GeometrySmith(N, wo, wi, roughness);

                Vector3f F0(0.45, 0.45, 0.45);
                //fresnel(wi, N, ior, F);
                Vector3f F = F0 + (Vector3f(1) - F0) * pow((1 - dotProduct(wi, h)), 5);
                
                
                float fenmu= std::max(0.01f,(4 * dotProduct(N, wo) * dotProduct(N, wi)));
                Vector3f specular = D * G * F / fenmu;

                float _ks;
                fresnel(wi, N, ior,_ks);
                float _kd = 1.0f - _ks;
                Vector3f diffuse = 1.0f / M_PI;
                Vector3f BRDF = _ks * specular + Kd * _kd * diffuse;
                return BRDF;
            }
            else
                return 0.0f;
            break;
        }
    }
}

#endif //RAYTRACING_MATERIAL_H
