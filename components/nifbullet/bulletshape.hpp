#ifndef NIFBULLET_BULLETSHAPE_HPP
#define NIFBULLET_BULLETSHAPE_HPP

#include <OgreResourceManager.h>
#include <OgreResource.h>

class btCollisionShape;
class btStridingMeshInterface;


namespace NifBullet
{

class BulletShape : public Ogre::Resource
{
    typedef std::map<unsigned int,btCollisionShape*> ScaledShapeMap;
    typedef std::vector<btStridingMeshInterface*> StridingMeshIfaceList;

    StridingMeshIfaceList mMeshIfaces;

    btCollisionShape *mCollisionShape;
    ScaledShapeMap mScaledCollisionShapes;

    // If this is false, "placeable" objects will not have collisions
    bool mHasRootCollision;

    static btCollisionShape *duplicateCollisionShape(btCollisionShape *shape);

    static void destroyCollisionShape(btCollisionShape *shape);

protected:
     virtual void loadImpl();
     virtual void unloadImpl();
     virtual size_t calculateSize() const;

     void setCollisionShape(btCollisionShape *shape)
     { mCollisionShape = shape; }

     friend class BulletShapeLoader;

public:
     BulletShape(Ogre::ResourceManager *creator, const Ogre::String &name,
                 Ogre::ResourceHandle handle, const Ogre::String &group, bool isManual = false, 
                 Ogre::ManualResourceLoader *loader = 0);
     virtual ~BulletShape();

     btCollisionShape *getCollisionShape() const
     { return mCollisionShape; }

     // Scale is accurate to 1/100ths
     btCollisionShape *getScaledCollisionShape(float scale);

     bool hasRootCollision() const
     { return mHasRootCollision; }
};

#if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))
typedef Ogre::SharedPtr<BulletShape> BulletShapePtr;
#else
class BulletShapePtr : public Ogre::SharedPtr<BulletShape>
{
public:
    BulletShapePtr() : Ogre::SharedPtr<BulletShape>() {}
    explicit BulletShapePtr(BulletShape *rep) : Ogre::SharedPtr<BulletShape>(rep) {}
    BulletShapePtr(const BulletShapePtr &r) : Ogre::SharedPtr<BulletShape>(r) {} 
    BulletShapePtr(const Ogre::ResourcePtr &r) : Ogre::SharedPtr<BulletShape>()
    {
        if( r.isNull() )
            return;
        // lock & copy other mutex pointer
        OGRE_LOCK_MUTEX(*r.OGRE_AUTO_MUTEX_NAME)
        OGRE_COPY_AUTO_SHARED_MUTEX(r.OGRE_AUTO_MUTEX_NAME)
        pRep = static_cast<BulletShape*>(r.getPointer());
        pUseCount = r.useCountPointer();
        useFreeMethod = r.freeMethod();
        if (pUseCount)
        {
            ++(*pUseCount);
        }
    }

    /// Operator used to convert a ResourcePtr to a TextFilePtr
    BulletShapePtr& operator=(const Ogre::ResourcePtr& r)
    {
        if(pRep == static_cast<BulletShape*>(r.getPointer()))
            return *this;
        release();
        if( r.isNull() )
            return *this; // resource ptr is null, so the call to release above has done all we need to do.
        // lock & copy other mutex pointer
        OGRE_LOCK_MUTEX(*r.OGRE_AUTO_MUTEX_NAME)
        OGRE_COPY_AUTO_SHARED_MUTEX(r.OGRE_AUTO_MUTEX_NAME)
        pRep = static_cast<TextFile*>(r.getPointer());
        pUseCount = r.useCountPointer();
        useFreeMethod = r.freeMethod();
        if (pUseCount)
        {
            ++(*pUseCount);
        }
        return *this;
    }
};
#endif


class BulletShapeManager : public Ogre::ResourceManager, public Ogre::Singleton<BulletShapeManager>
{
protected:
    // must implement this from ResourceManager's interface
    Ogre::Resource *createImpl(const Ogre::String &name, Ogre::ResourceHandle handle, 
                               const Ogre::String &group, bool isManual, Ogre::ManualResourceLoader *loader, 
                               const Ogre::NameValuePairList *createParams);

public:
    BulletShapeManager();
    virtual ~BulletShapeManager();

#if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))
    BulletShapePtr getByName(const Ogre::String &name, const Ogre::String &groupName=Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME)
    { return getResourceByName(name, groupName).staticCast<BulletShape>(); }

    BulletShapePtr create(const Ogre::String &name, const Ogre::String &group,
                          bool isManual=false, Ogre::ManualResourceLoader *loader=0,
                          const Ogre::NameValuePairList *createParams=0)
    { return createResource(name, group, isManual, loader, createParams).staticCast<BulletShape>(); }
#endif

    virtual BulletShapePtr load(const Ogre::String &name, const Ogre::String &group);

    static BulletShapeManager &getSingleton();
    static BulletShapeManager *getSingletonPtr();
};

}

#endif /* NIFBULLET_BULLETSHAPE_HPP */
