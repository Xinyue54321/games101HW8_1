//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{//scene中的bvh判断相交
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{//
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

Vector3f Scene::shade(Intersection p, const Ray &ray) const{ 
    Intersection x;
    float pdf_light;
    Vector3f L_dir(0, 0, 0), L_indir(0, 0, 0);
    sampleLight(x, pdf_light);
    Vector3f ws_dir = normalize(x.coords - p.coords);
    Ray ws(p.coords, ws_dir);
    
    if ((x.coords - p.coords).norm() - this->intersect(ws).distance < 0.001) {
        Vector3f BRDF = p.m->eval(ws.direction, -ray.direction, p.normal);
        float cosw1 = std::max(0.f, dotProduct(ws.direction, p.normal));
        float cosw2 = std::max(0.f, dotProduct(-ws.direction, x.normal));
        float distance2 = dotProduct((p.coords - x.coords), (p.coords - x.coords));
        L_dir = x.emit * BRDF * cosw1 * cosw2 / distance2 / pdf_light;
    }

    float P_PR = get_random_float();
    if (P_PR < RussianRoulette) {
        Vector3f wi_dir = p.m->sample(ray.direction, p.normal);
        Ray wi(p.coords, wi_dir);
        Intersection q = this->intersect(wi);
        if (q.happened && !q.m->hasEmission()) {
            Vector3f BRDF = p.m->eval( wi.direction, -ray.direction, p.normal);
            float pdf_dif = p.m->pdf(ray.direction, wi.direction, p.normal);
            float cosw = dotProduct(wi.direction, p.normal);
            L_indir = shade(q, wi) * BRDF * cosw / pdf_dif / RussianRoulette;
        }

    }
    
    return L_dir +L_indir + p.emit;
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection p = this->intersect(ray);
    Vector3f hitColor(0, 0, 0);
    if (p.happened) {
        hitColor += shade(p, ray);
    }
    return hitColor;
}