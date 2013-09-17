#include "bulletnifloader.hpp"

#include <btBulletCollisionCommon.h>

#include "../nif/niffile.hpp"
#include "../nif/node.hpp"

#include "bulletshape.hpp"


namespace NifBullet
{

class TriangleMesh : public btTriangleIndexVertexArray
{
    btAlignedObjectArray<btVector3> mVertices;
    btAlignedObjectArray<unsigned short> mIndices;

public:
    TriangleMesh()
    {
        btIndexedMesh meshIndex;
        meshIndex.m_numTriangles = 0;
        meshIndex.m_numVertices = 0;
        meshIndex.m_indexType = PHY_SHORT;
        meshIndex.m_triangleIndexBase = 0;
        meshIndex.m_triangleIndexStride = 3*sizeof(short);
        meshIndex.m_vertexBase = 0;
        meshIndex.m_vertexStride = sizeof(btVector3);
        m_indexedMeshes.push_back(meshIndex);
    }

    virtual void preallocateVertices(int) { }
    virtual void preallocateIndices(int) { }

    void addVertex(const btVector3 &vertex)
    {
        mVertices.push_back(vertex);
    }

    void addTriangleIndices(unsigned short idx1, unsigned short idx2, unsigned short idx3)
    {
        mIndices.push_back(idx1);
        mIndices.push_back(idx2);
        mIndices.push_back(idx3);
    }

    void finalise()
    {
        m_indexedMeshes[0].m_vertexBase = (unsigned char*)&mVertices[0];
        m_indexedMeshes[0].m_numVertices = mVertices.size();

        m_indexedMeshes[0].m_triangleIndexBase = (unsigned char*)&mIndices[0];
        m_indexedMeshes[0].m_numTriangles = mIndices.size()/3;
    }
};


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
        std::cerr<< "No collision shape found for "<<name <<std::endl;

    getBoundingBox(node, shape);
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
            if(ninode->children[i].empty())
                continue;
            if((node=findRootCollisionNode(ninode->children[i].getPtr())) != NULL)
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
            if(!ninode->children[i].empty())
                buildFromRootCollision(ninode->children[i].getPtr(), shape);
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
            if(!ninode->children[i].empty())
                buildFromModel(ninode->children[i].getPtr(), shape);
        }
    }
}


void BulletShapeLoader::loadTriShape(const Nif::NiTriShape *trishape, BulletShape *shape)
{
    btCollisionShape *curshape = shape->getCollisionShape();
    assert(!curshape || curshape->isCompound());

    TriangleMesh *mesh = new TriangleMesh;
    shape->mMeshIfaces.push_back(mesh);

    const Nif::NiTriShapeDataPtr data = trishape->data;
    const std::vector<Ogre::Vector3> &vertices = data->vertices;
    const std::vector<short> &indices = data->triangles;
    for(size_t i = 0;i < vertices.size();++i)
        mesh->addVertex(btVector3(vertices[i].x, vertices[i].y, vertices[i].z));
    for(size_t i = 0;i < indices.size();i+=3)
        mesh->addTriangleIndices(indices[i], indices[i+1], indices[i+2]);
    mesh->finalise();

    Ogre::Matrix4 transform = trishape->getWorldTransform();
    btTransform bttrans(btMatrix3x3(transform[0][0], transform[0][1], transform[0][2],
                                    transform[1][0], transform[1][1], transform[1][2],
                                    transform[2][0], transform[2][1], transform[2][2]),
                        btVector3(transform[0][3], transform[1][3], transform[2][3]));
    btCompoundShape *comp = (curshape ? static_cast<btCompoundShape*>(curshape) : new btCompoundShape());
    comp->addChildShape(bttrans, new btBvhTriangleMeshShape(mesh, true));
    shape->setCollisionShape(comp);
}


bool BulletShapeLoader::getBoundingBox(const Nif::Node* node, BulletShape* shape)
{
    if(node->hasBounds && node->name == "Bounding Box")
    {
        const Ogre::Matrix3 &mat3 = node->boundRot;
        shape->mBBoxTransform.setBasis(btMatrix3x3(mat3[0][0], mat3[0][1], mat3[0][2],
                                                   mat3[1][0], mat3[1][1], mat3[1][2],
                                                   mat3[2][0], mat3[2][1], mat3[2][2]));
        shape->mBBoxTransform.setOrigin(btVector3(node->boundPos.x, node->boundPos.y, node->boundPos.z));

        shape->mBBoxRadius.setX(node->boundXYZ.x);
        shape->mBBoxRadius.setY(node->boundXYZ.y);
        shape->mBBoxRadius.setZ(node->boundXYZ.z);

        return true;
    }

    const Nif::NiNode *ninode = dynamic_cast<const Nif::NiNode*>(node);
    if(ninode != NULL)
    {
        for(size_t i = 0;i < ninode->children.length();i++)
        {
            if(ninode->children[i].empty())
                continue;
            if(getBoundingBox(ninode->children[i].getPtr(), shape))
                return true;
        }
    }

    return false;
}

}
