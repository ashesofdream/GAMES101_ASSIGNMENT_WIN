//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <random>
#include <type_traits>


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
    //this function will use the luminous square
    //to generate random num to sample a light
    //

    float emit_area_sum = 0;
    //calculate all no-light object 
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
// Actually this should be called eye ray
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    static std::random_device r_dev;
    static std::mt19937 gen(r_dev());
    static std::uniform_real_distribution<> real(0,1);
    // TO DO Implement Path Tracing Algorithm here
    Vector3f return_color;
    //auto Russian_Roulette = real(gen);
    
    //TODO: 
    auto obj_inter = intersect(ray);
    if(!obj_inter.happened) return return_color;
    if(depth>0&&obj_inter.m->hasEmission()) return return_color;
    // direct ray
    float pdf = 0.f;
    Intersection light_inter;
    sampleLight(light_inter,pdf);
    
    auto wi = (obj_inter.coords - light_inter.coords).normalized() ;
    Ray sample_ray(light_inter.coords,wi);
    Vector3f wo = -ray.direction;
    if(intersect(sample_ray).obj == obj_inter.obj){ 
        auto fr = obj_inter.m->eval(wi, wo, obj_inter.normal);
        auto cos_theta = dotProduct(obj_inter.normal,-wi);
        auto cos_pine_theta = dotProduct(light_inter.normal, wi);
        return_color += (light_inter.emit * fr * cos_theta * cos_pine_theta/
        (light_inter.coords - obj_inter.coords).norm_square()
        /pdf);
    }
    //global illumination
    if(obj_inter.obj&&real(gen)<RussianRoulette){
        //TODO: detect global illumination
        Ray indirect_ray(obj_inter.coords,obj_inter.m->sample(wo, obj_inter.normal));
        auto indirect_emi = castRay(indirect_ray, depth+1);
        if(indirect_emi.norm_square()<EPSILON) return return_color; 
        return_color += indirect_emi*
        obj_inter.m->eval(-indirect_ray.direction,wo,obj_inter.normal)*
        dotProduct(indirect_ray.direction, obj_inter.normal)/
        obj_inter.m->pdf(-indirect_ray.direction, wo, obj_inter.normal)/
        RussianRoulette;
    }
    if(obj_inter.m->hasEmission()) return_color += obj_inter.m->m_emission;
    return return_color;
}
