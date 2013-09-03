#ifndef NIFBULLET_NIFBULLETLOADER_HPP
#define NIFBULLET_NIFBULLETLOADER_HPP

#include <string>


namespace Nif
{
    class Node;
    class NiTriShape;
}

namespace NifBullet
{

class BulletShape;

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
