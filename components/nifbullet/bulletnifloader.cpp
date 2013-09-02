#include "bulletnifloader.hpp"

#include <btBulletCollisionCommon.h>

#include "../nif/niffile.hpp"
#include "../nif/node.hpp"

#include "bulletshape.hpp"


namespace NifBullet
{

void BulletShapeLoader::load(const std::string &name, BulletShape *shape)
{
    Nif::NIFFile::ptr nif = Nif::NIFFile::create(name);
    assert(nif->numRoots() > 0);

    const Nif::Record *r = nif->getRoot(0);
    assert(r != NULL);

    const Nif::Node *node = dynamic_cast<const Nif::Node*>(r);
    assert(node != NULL);

    shape->setCollisionShape(new btBoxShape(btVector3(10,10,10)));
}

}
