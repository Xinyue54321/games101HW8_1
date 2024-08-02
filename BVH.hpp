//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

struct BVHBuildNode;
// BVHAccel Forward Declarations
struct BVHPrimitiveInfo;

// BVHAccel Declarations
inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
class BVHAccel {

public:
    // BVHAccel Public Types
    enum class SplitMethod { NAIVE, SAH };

    // BVHAccel Public Methods
    BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);//建立BVH树
    Bounds3 WorldBound() const;//没写
    ~BVHAccel();

    Intersection Intersect(const Ray &ray) const;//找ray和物体交点
    Intersection getIntersection(BVHBuildNode* node, const Ray& ray)const;//找ray和物体交点，Intersect调用了这个函数，用的时候直接调Intersect就行
    bool IntersectP(const Ray &ray) const;//没写
    BVHBuildNode* root;

    // BVHAccel Private Methods
    BVHBuildNode* recursiveBuild(std::vector<Object*>objects);//建立bvh树的过程，是BVHAccel内调用的，用的时候直接用BVHAccel就可以了，其实声明的时候就已经调用了

    // BVHAccel Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Object*> primitives;

    void getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf);//好像是算pdf的
    void Sample(Intersection &pos, float &pdf);//好像是算pdf的,调用这个，不要调用getSample
};

struct BVHBuildNode {
    Bounds3 bounds;
    BVHBuildNode *left;
    BVHBuildNode *right;
    Object* object;
    float area;

public:
    int splitAxis=0, firstPrimOffset=0, nPrimitives=0;
    // BVHBuildNode Public Methods
    BVHBuildNode(){
        bounds = Bounds3();
        left = nullptr;right = nullptr;
        object = nullptr;
    }
};




#endif //RAYTRACING_BVH_H
