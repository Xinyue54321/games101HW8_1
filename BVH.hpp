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
    BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);//����BVH��
    Bounds3 WorldBound() const;//ûд
    ~BVHAccel();

    Intersection Intersect(const Ray &ray) const;//��ray�����彻��
    Intersection getIntersection(BVHBuildNode* node, const Ray& ray)const;//��ray�����彻�㣬Intersect����������������õ�ʱ��ֱ�ӵ�Intersect����
    bool IntersectP(const Ray &ray) const;//ûд
    BVHBuildNode* root;

    // BVHAccel Private Methods
    BVHBuildNode* recursiveBuild(std::vector<Object*>objects);//����bvh���Ĺ��̣���BVHAccel�ڵ��õģ��õ�ʱ��ֱ����BVHAccel�Ϳ����ˣ���ʵ������ʱ����Ѿ�������

    // BVHAccel Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Object*> primitives;

    void getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf);//��������pdf��
    void Sample(Intersection &pos, float &pdf);//��������pdf��,�����������Ҫ����getSample
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
