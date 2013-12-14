#ifndef NIFBULLET_NIFBULLETLOADER_HPP
#define NIFBULLET_NIFBULLETLOADER_HPP

#include <string>

#include <BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h>


namespace Nif
{
    class Node;
    class NiTriShape;
}

namespace NifBullet
{

class BulletShape;


class TriangleMesh : public btTriangleIndexVertexArray
{
    btAlignedObjectArray<btVector3> mVertices;
    btAlignedObjectArray<unsigned short> mIndices;

public:
    TriangleMesh();

    virtual void preallocateVertices(int) { }
    virtual void preallocateIndices(int) { }

    void addVertex(const btVector3 &vertex);

    void addTriangleIndices(unsigned short idx1, unsigned short idx2, unsigned short idx3);

    void finalise();

    TriangleMesh *clone(float scale) const;
};


class BulletShapeLoader
{
    const Nif::Node *findRootCollisionNode(const Nif::Node *node);

    void loadTriShape(const Nif::NiTriShape *trishape, BulletShape *shape);
    void buildFromRootCollision(const Nif::Node *node, BulletShape *shape);
    void buildFromModel(const Nif::Node *node, BulletShape *shape);

    bool getBoundingBox(const Nif::Node *node, BulletShape *shape);

public:
    void load(const std::string &name, BulletShape *shape);
};

}

#endif /* NIFBULLET_NIFBULLETLOADER_HPP */
