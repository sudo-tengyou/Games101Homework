//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection pInter = intersect(ray);
    if(!pInter.happened) {
        return Vector3f();
    }
    if (pInter.m->hasEmission()) {
        return pInter.m->getEmission();
    }

    // 计算直接光照
    Vector3f pNormal = pInter.normal.normalized();
    Vector3f& p = pInter.coords;

    Intersection lightInter;
    float lightPdf = 0.0f;
    sampleLight(lightInter, lightPdf);
    Vector3f& lightPos = lightInter.coords;
    Vector3f lightDir = (lightPos - p).normalized();
    Vector3f lightNormal = lightInter.normal.normalized();
    Vector3f& lightEmit = lightInter.emit;

    Intersection lightDirBlock = intersect(Ray(p, lightDir));
    Vector3f L_dir;
    //if(lightDirBlock.distance - (p-lightPos).norm() > -10 * EPSILON) {
    if(lightInter.obj == lightDirBlock.obj) {
        L_dir = lightEmit * pInter.m->eval(ray.direction, lightDir, pNormal) *
            dotProduct(lightDir, pNormal) * dotProduct(-lightDir, lightNormal) /
            pow((p-lightPos).norm(), 2) / lightPdf;
    }

    if(get_random_float() > RussianRoulette) {
        return L_dir;
    }

    // 计算间接光照
    Vector3f inDir = pInter.m->sample(ray.direction, pNormal).normalized();
    Ray inRay(p, inDir);
    Intersection inInter = intersect(inRay);
    Vector3f L_indir;
    if(inInter.happened && (!inInter.m->hasEmission())) {
        L_indir = castRay(inRay, depth+1) * pInter.m->eval(ray.direction, inDir, pNormal) *
            dotProduct(inDir, pNormal) / pInter.m->pdf(ray.direction, inDir, pNormal) /
            RussianRoulette;  // 除以该概率是为了保持能量守恒
    }

    Vector3f hitColor = L_dir + L_indir;
    hitColor.x = clamp(0, 1, hitColor.x);
    hitColor.y = clamp(0, 1, hitColor.y);
    hitColor.z = clamp(0, 1, hitColor.z);

    return hitColor;
}