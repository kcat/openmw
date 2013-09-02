#include "bulletshape.hpp"
#include "bulletnifloader.hpp"

#include <btBulletCollisionCommon.h>

namespace Ogre
{
template<>
NifBullet::BulletShapeManager *Singleton<NifBullet::BulletShapeManager>::msSingleton = 0;
}

namespace NifBullet
{

BulletShape::BulletShape(Ogre::ResourceManager* creator, const Ogre::String &name,
                         Ogre::ResourceHandle handle, const Ogre::String &group, bool isManual,
                         Ogre::ManualResourceLoader *loader)
  : Ogre::Resource(creator, name, handle, group, isManual, loader)
  , mCollisionShape(0)
{
    /* For consistency with StringInterface, but we don't add any parameters here
     * That's because the Resource implementation of StringInterface is to
     * list all the options that need to be set before loading, of which
     * we have none as such. Full details can be set through scripts.
     */
    createParamDictionary("BulletShape");
}

BulletShape::~BulletShape()
{
    unload();
}

void BulletShape::loadImpl()
{
    BulletShapeLoader loader;
    loader.load(mName, this);
}

void BulletShape::unloadImpl()
{
    ScaledShapeMap::iterator coliter = mScaledCollisionShapes.begin();
    for(;coliter != mScaledCollisionShapes.end();coliter++)
        destroyCollisionShape(coliter->second);
    mScaledCollisionShapes.clear();

    if(mCollisionShape)
        destroyCollisionShape(mCollisionShape);
    mCollisionShape = 0;
}

size_t BulletShape::calculateSize() const
{
    // TODO: Get resource data size
    return 1;
}


btCollisionShape* BulletShape::getScaledCollisionShape(float scale)
{
    assert(scale > 0.0f);
    unsigned int scaleidx = (unsigned int)(scale * 100.0f);

    if(scaleidx == 100)
        return mCollisionShape;

    ScaledShapeMap::iterator coliter = mScaledCollisionShapes.find(scaleidx);
    if(coliter != mScaledCollisionShapes.end())
        return coliter->second;

    // FIXME: Generate scaled collision shape
    return mCollisionShape;
}


void BulletShape::destroyCollisionShape(btCollisionShape *shape)
{
    if(shape->isCompound())
    {
        btCompoundShape *comp = static_cast<btCompoundShape*>(shape);
        int numShapes = comp->getNumChildShapes();
        for(int i = 0;i < numShapes;i++)
            destroyCollisionShape(comp->getChildShape(i));
    }
    delete shape;
}


BulletShapeManager *BulletShapeManager::getSingletonPtr()
{
    assert(msSingleton);
    return msSingleton;
}

BulletShapeManager &BulletShapeManager::getSingleton()
{  
    return *getSingletonPtr();
}

BulletShapeManager::BulletShapeManager()
{
    mResourceType = "BulletShape";

    // low, because it will likely reference other resources
    mLoadOrder = 30.0f;

    // this is how we register the ResourceManager with OGRE
    Ogre::ResourceGroupManager::getSingleton()._registerResourceManager(mResourceType, this);
}

BulletShapeManager::~BulletShapeManager()
{
    // and this is how we unregister it
    Ogre::ResourceGroupManager::getSingleton()._unregisterResourceManager(mResourceType);
}

BulletShapePtr BulletShapeManager::load(const Ogre::String &name, const Ogre::String &group)
{
    BulletShapePtr shape = getByName(name);
    if(shape.isNull())
        shape = create(name, group);

    shape->load();
    return shape;
}

Ogre::Resource *BulletShapeManager::createImpl(const Ogre::String &name, Ogre::ResourceHandle handle,
                                               const Ogre::String &group, bool isManual, Ogre::ManualResourceLoader *loader,
                                               const Ogre::NameValuePairList *createParams)
{
    return new BulletShape(this, name, handle, group, isManual, loader);
}


}
