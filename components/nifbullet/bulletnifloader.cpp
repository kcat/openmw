#include "bulletnifloader.hpp"

#include <btBulletCollisionCommon.h>

#include "../nif/niffile.hpp"
#include "../nif/node.hpp"

#include "bulletshape.hpp"


namespace
{

struct TriangleMeshShape : public btBvhTriangleMeshShape
{
    TriangleMeshShape(btStridingMeshInterface *meshInterface, bool useQuantizedAabbCompression)
      : btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression)
    {
    }

    virtual ~TriangleMeshShape()
    {
        delete getTriangleInfoMap();
        delete m_meshInterface;
    }
};

}

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

    const Nif::Node *colroot = findRootCollisionNode(node);
    shape->mHasRootCollision = (colroot != NULL);

    if(colroot)
        buildFromRootCollision(colroot, shape);
    else
        buildFromModel(node, shape);

    if(!shape->getCollisionShape())
        shape->setCollisionShape(new btBoxShape(btVector3(10,10,10)));
}


const Nif::Node* BulletShapeLoader::findRootCollisionNode(const Nif::Node *node)
{
    if(node->recType == Nif::RC_RootCollisionNode)
        return node;

    const Nif::NiNode *ninode = dynamic_cast<const Nif::NiNode*>(node);
    if(ninode != NULL)
    {
        for(size_t i = 0;i < ninode->children.length();i++)
        {
            node = ninode->children[i].getPtr();
            if(node && (node=findRootCollisionNode(node)))
                return node;
        }
    }

    return NULL;
}

void BulletShapeLoader::buildFromRootCollision(const Nif::Node *node, BulletShape *shape)
{
    // NOTE: We may want to load AvoidNode shapes in as ghost objects, which can then be used for
    // AI purposes (to avoid contact).
    if(node->recType == Nif::RC_AvoidNode)
        return;

    if(node->recType == Nif::RC_NiTriShape)
    {
        const Nif::NiTriShape *trishape = static_cast<const Nif::NiTriShape*>(node);
        loadTriShape(trishape, shape);
    }

    const Nif::NiNode *ninode = dynamic_cast<const Nif::NiNode*>(node);
    if(ninode != NULL)
    {
        for(size_t i = 0;i < ninode->children.length();i++)
        {
            node = ninode->children[i].getPtr();
            if(node) buildFromRootCollision(node, shape);
        }
    }
}

void BulletShapeLoader::buildFromModel(const Nif::Node* node, BulletShape* shape)
{
    Nif::ExtraPtr extra = node->extra;
    while(!extra.empty())
    {
        if(extra->recType == Nif::RC_NiStringExtraData)
        {
            // No collisions beyond this node if one of these strings are set.
            const Nif::NiStringExtraData *strdata = static_cast<const Nif::NiStringExtraData*>(extra.getPtr());
            if(strdata->string == "NCO" || strdata->string == "NCC" || strdata->string == "MRK")
                return;
        }
        extra = extra->extra;
    }

    if(node->recType == Nif::RC_NiTriShape)
    {
        const Nif::NiTriShape *trishape = static_cast<const Nif::NiTriShape*>(node);
        loadTriShape(trishape, shape);
    }

    const Nif::NiNode *ninode = dynamic_cast<const Nif::NiNode*>(node);
    if(ninode != NULL)
    {
        for(size_t i = 0;i < ninode->children.length();i++)
        {
            node = ninode->children[i].getPtr();
            if(node) buildFromModel(node, shape);
        }
    }
}


void BulletShapeLoader::loadTriShape(const Nif::NiTriShape *trishape, BulletShape *shape)
{
    btCollisionShape *curshape = shape->getCollisionShape();
    assert(!curshape || curshape->isCompound());

    std::auto_ptr<btTriangleMesh> mesh(new btTriangleMesh);

    const Nif::NiTriShapeData *data = trishape->data.getPtr();
    const Ogre::Vector3 *vertices = &data->vertices[0];
    const short *triangles = &data->triangles[0];
    for(size_t i = 0;i < data->triangles.size();i+=3)
    {
        Ogre::Vector3 b1 = vertices[triangles[i+0]];
        Ogre::Vector3 b2 = vertices[triangles[i+1]];
        Ogre::Vector3 b3 = vertices[triangles[i+2]];
        mesh->addTriangle(btVector3(b1.x, b1.y, b1.z), btVector3(b2.x, b2.y, b2.z),
                          btVector3(b3.x, b3.y, b3.z));
    }

    Ogre::Matrix3 mat3;
    Ogre::Matrix4 transform = trishape->getWorldTransform();
    transform.extract3x3Matrix(mat3);
    Ogre::Vector3 pos(transform[0][3], transform[1][3], transform[2][3]);

    btTransform bttrans(btMatrix3x3(mat3[0][0], mat3[0][1], mat3[0][2],
                                    mat3[1][0], mat3[1][1], mat3[1][2],
                                    mat3[2][0], mat3[2][1], mat3[2][2]),
                        btVector3(pos.x, pos.y, pos.z));
    btCompoundShape *comp = (curshape ? static_cast<btCompoundShape*>(curshape) : new btCompoundShape());
    comp->addChildShape(bttrans, new TriangleMeshShape(mesh.release(), true));
    shape->setCollisionShape(comp);
}

}
