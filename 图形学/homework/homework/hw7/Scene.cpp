//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Material.hpp"

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
    Intersection inter = intersect(ray);
    if(!inter.happened)
        return {};
    
    Vector3f L_dir(0,0,0);
    if(inter.m->hasEmission())  return inter.m->getEmission();
    //direct light
    Intersection inter2;
    float pdf = 0;
    sampleLight(inter2,pdf);

    Vector3f wi = ray.direction;
    Vector3f wo = (inter2.coords - inter.coords).normalized();
    Ray ray2(inter.coords,wo);
    Intersection inter3 = intersect(ray2);
    // if no block between light and inter
    if(inter3.distance - inter2.distance > -EPSILON)
        L_dir += inter2.emit * inter.m->eval(wi,wo,inter.normal) * dotProduct(wo,inter.normal) * dotProduct(-wo,inter2.normal) / pdf / pow((inter2.coords-inter.coords).norm(),2);
    Vector3f L_indir(0,0,0);
    if(get_random_float()<=RussianRoulette)
    {
        Vector3f dir2 = inter.m->sample(wi,inter.normal).normalized();
        Ray ray3(inter.coords,dir2);
        Intersection inter4 = intersect(ray3);
        if(inter4.happened&&!inter4.m->hasEmission())
        {
            L_indir = castRay(ray3,depth+1) * inter.m->eval(wi,dir2,inter.normal) * dotProduct(dir2,inter.normal) / inter.m->pdf(wi,dir2,inter.normal) / RussianRoulette;
        }
    }
    return L_dir + L_indir;
}