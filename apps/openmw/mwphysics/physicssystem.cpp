#include "physicssystem.hpp"

#include <stdexcept>

#include <OgreVector3.h>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

#include <components/nifbullet/bulletshape.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"

#include "../mwworld/class.hpp"

#include "debugdraw.hpp"
#include "object.hpp"
#include "actor.hpp"
#include "charactercontroller.hpp"


namespace
{
    struct FilteredRayResultCallback : public btCollisionWorld::RayResultCallback
    {
    private:
        MWWorld::Ptr mFilter;

    public:
        FilteredRayResultCallback(const MWWorld::Ptr &filter) : mFilter(filter)
        {
        }

        virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult &rayResult, bool /*normalInWorldSpace*/)
        {
            // Caller already does the filter on the m_closestHitFraction
            btAssert(rayResult.m_hitFraction <= m_closestHitFraction);

            m_collisionObject = rayResult.m_collisionObject;
            MWPhysics::ObjectInfo *inf;
            if((inf=static_cast<MWPhysics::ObjectInfo*>(m_collisionObject->getUserPointer())) != NULL)
            {
                if(inf->getPtr() == mFilter)
                    return btScalar(1.0f);
            }
            m_closestHitFraction = rayResult.m_hitFraction;

            return rayResult.m_hitFraction;
        }
    };
}


namespace MWPhysics
{
    // Heightfield container. Note that heightmaps will have an empty Ptr object in ObjectInfo.
    class Heightmap : public ObjectInfo
    {
        btHeightfieldTerrainShape *mShape;
        btCollisionObject *mCollisionObject;

    public:
        Heightmap(int x, int y, const float *heights, float yoffset, float triSize, int sqrtVerts)
          : ObjectInfo(MWWorld::Ptr())
          , mShape(0)
          , mCollisionObject(0)
        {
            // find the minimum and maximum heights (needed for bullet)
            float minh = heights[0];
            float maxh = heights[0];
            for(int i = 1;i < sqrtVerts*sqrtVerts;++i)
            {
                float h = heights[i];
                if(h > maxh) maxh = h;
                if(h < minh) minh = h;
            }

            mShape = new btHeightfieldTerrainShape(
                sqrtVerts, sqrtVerts, heights, 1,
                minh, maxh, 2,
                PHY_FLOAT, true
            );
            mShape->setUseDiamondSubdivision(true);
            mShape->setLocalScaling(btVector3(triSize, triSize, 1));

            btTransform transform(btQuaternion::getIdentity(),
                                  btVector3((x+0.5f) * triSize * (sqrtVerts-1.0f),
                                            (y+0.5f) * triSize * (sqrtVerts-1.0f),
                                            (maxh+minh)*0.5f));

            mCollisionObject = new btCollisionObject;
            mCollisionObject->setCollisionShape(mShape);
            mCollisionObject->setWorldTransform(transform);

            // Explicitly downcast to ensure the user pointer can be directly casted back to
            // ObjectInfo.
            mCollisionObject->setUserPointer(static_cast<ObjectInfo*>(this));
        }

        virtual ~Heightmap()
        {
            delete mCollisionObject;
            delete mShape;
        }

        btCollisionObject *getCollisionObject() const
        { return mCollisionObject; }
    };


    PhysicsSystem::PhysicsSystem(Ogre::SceneManager *sceneMgr)
      : mTimeAccum(0.0f)
    {
        mCollisionConfiguration = new btDefaultCollisionConfiguration();
        mDispatcher = new btCollisionDispatcher(mCollisionConfiguration);

        mSolver = new btSequentialImpulseConstraintSolver();
        mGhostPairCallback = new btGhostPairCallback();
        mBroadphase = new btDbvtBroadphase();
        mDynamicsWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadphase, mSolver, mCollisionConfiguration);
        mDynamicsWorld->setGravity(btVector3(0.0f, 0.0f, -627.2f));

        mDebugDraw = new DebugDraw(sceneMgr, mDynamicsWorld);

        mBroadphase->getOverlappingPairCache()->setInternalGhostPairCallback(mGhostPairCallback);
    }

    PhysicsSystem::~PhysicsSystem()
    {
        // Delete in reverse order that they were new'd
        delete mDebugDraw;
        delete mDynamicsWorld;
        delete mBroadphase;
        delete mGhostPairCallback;
        delete mSolver;
        delete mDispatcher;
        delete mCollisionConfiguration;
    }


    void PhysicsSystem::addObject(const MWWorld::Ptr &ptr, bool placeable)
    {
        const std::string name = Misc::StringUtils::lowerCase(ptr.getClass().getModel(ptr));
        // Don't add marker objects
        if(name.find("marker") != std::string::npos)
            return;

        NifBullet::BulletShapeManager &shapeMgr = NifBullet::BulletShapeManager::getSingleton();
        NifBullet::BulletShapePtr shape = shapeMgr.load(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        if(!shape->getCollisionShape())
            return;

        if(placeable && !shape->hasRootCollision())
            return;

        Object *obj = new Object(ptr, shape, this);
        mObjects.insert(std::make_pair(ptr.getRefData().getHandle(), obj));

        mDynamicsWorld->addCollisionObject(obj->getCollisionObject(),
                                           btBroadphaseProxy::StaticFilter,
                                           btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
    }

    void PhysicsSystem::addActor(const MWWorld::Ptr &ptr)
    {
        NifBullet::BulletShapeManager &shapeMgr = NifBullet::BulletShapeManager::getSingleton();
        const std::string name = Misc::StringUtils::lowerCase(ptr.getClass().getModel(ptr));
        NifBullet::BulletShapePtr shape = shapeMgr.load(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        if(shape->getBBoxRadius() == btVector3(0.0f, 0.0f, 0.0f))
        {
            // TODO: Generate a bounding box from the actual shape? Make a default?
            return;
        }

        Actor *actor = new Actor(ptr, shape, this);
        mActors.insert(std::make_pair(ptr.getRefData().getHandle(), actor));

        mDynamicsWorld->addCollisionObject(actor->getCollisionObject(),
                                           btBroadphaseProxy::CharacterFilter,
                                           btBroadphaseProxy::AllFilter);
        mDynamicsWorld->addAction(actor->getActionInterface());
    }

    void PhysicsSystem::updateObject(const std::string &handle, const MWWorld::Ptr &ptr)
    {
        ObjectMap::iterator obj = mObjects.find(handle);
        if(obj != mObjects.end())
        {
            obj->second->updatePtr(ptr);
            return;
        }

        ActorMap::iterator actor = mActors.find(handle);
        if(actor != mActors.end())
        {
            actor->second->updatePtr(ptr);
            return;
        }
    }

    void PhysicsSystem::removeObject(const std::string &handle)
    {
        ObjectMap::iterator obj = mObjects.find(handle);
        if(obj != mObjects.end())
        {
            mDynamicsWorld->removeCollisionObject(obj->second->getCollisionObject());
            delete obj->second;
            mObjects.erase(obj);
            return;
        }

        ActorMap::iterator actor = mActors.find(handle);
        if(actor != mActors.end())
        {
            mDynamicsWorld->removeAction(actor->second->getActionInterface());
            mDynamicsWorld->removeCollisionObject(actor->second->getCollisionObject());
            delete actor->second;
            mActors.erase(actor);
            return;
        }
    }

    void PhysicsSystem::moveObject(const MWWorld::Ptr& ptr)
    {
        ActorMap::iterator aiter(mActors.find(ptr.getRefData().getHandle()));
        if(aiter != mActors.end())
        {
            const float *pos = ptr.getRefData().getPosition().pos;

            Actor *actor = aiter->second;
            actor->setOrigin(btVector3(pos[0], pos[1], pos[2]));
        }
    }

    void PhysicsSystem::rotateObject(const MWWorld::Ptr& ptr)
    {
        /* Nothing to do for actors. */
    }

    void PhysicsSystem::scaleObject(const MWWorld::Ptr& ptr)
    {
        ObjectMap::iterator obj(mObjects.find(ptr.getRefData().getHandle()));
        if(obj != mObjects.end())
        {
            Object *object = obj->second;
            mDynamicsWorld->removeCollisionObject(object->getCollisionObject());
            object->resetCollisionObject();
            mDynamicsWorld->addCollisionObject(object->getCollisionObject(),
                                               btBroadphaseProxy::StaticFilter,
                                               btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
            return;
        }

        ActorMap::iterator act(mActors.find(ptr.getRefData().getHandle()));
        if(act != mActors.end())
        {
            Actor *actor = act->second;
            mDynamicsWorld->removeAction(actor->getActionInterface());
            mDynamicsWorld->removeCollisionObject(actor->getCollisionObject());
            actor->resetCollisionObject();
            mDynamicsWorld->addCollisionObject(actor->getCollisionObject(),
                                               btBroadphaseProxy::CharacterFilter,
                                               btBroadphaseProxy::AllFilter);
            mDynamicsWorld->addAction(actor->getActionInterface());
            return;
        }
    }


    void PhysicsSystem::addHeightField(int x, int y, const float *heights, float yoffset,
                                       float triSize, int sqrtVerts)
    {
        Heightmap *heightmap = new Heightmap(x, y, heights, yoffset, triSize, sqrtVerts);
        mHeightmaps[std::make_pair(x,y)] = heightmap;

        mDynamicsWorld->addCollisionObject(heightmap->getCollisionObject(),
                                           btBroadphaseProxy::StaticFilter,
                                           btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
    }

    void PhysicsSystem::removeHeightField(int x, int y)
    {
        HeightmapMap::iterator heightmap = mHeightmaps.find(std::make_pair(x,y));
        if(heightmap != mHeightmaps.end())
        {
            mDynamicsWorld->removeCollisionObject(heightmap->second->getCollisionObject());
            delete heightmap->second;
            mHeightmaps.erase(heightmap);
        }
    }


    bool PhysicsSystem::toggleCollisionMode()
    {
        ActorMap::iterator aiter = mActors.find("player");
        if(aiter == mActors.end())
            return false;

        Actor *actor = aiter->second;
        bool on = !actor->getCollisionObject()->getBroadphaseHandle()->m_collisionFilterMask;

        mDynamicsWorld->removeCollisionObject(actor->getCollisionObject());
        mDynamicsWorld->addCollisionObject(actor->getCollisionObject(),
                                           btBroadphaseProxy::CharacterFilter,
                                           (on ? btBroadphaseProxy::AllFilter : 0));
        if(on/*&& !flying && !swimming*/)
        {
            actor->getActionInterface()->setGravity(-627.2f);
            actor->getActionInterface()->setFallSpeed(-62720.0f);
        }
        else
        {
            actor->getActionInterface()->setGravity(0.0f);
            actor->getActionInterface()->setFallSpeed(0.0f);
        }

        return on;
    }


    bool PhysicsSystem::getObjectAABB(const MWWorld::Ptr &ptr, Ogre::Vector3 &min, Ogre::Vector3 &max)
    {
        return false;
    }


    std::pair<std::string,Ogre::Vector3> PhysicsSystem::getHitContact(const std::string &name,
                                                                      const Ogre::Vector3 &origin,
                                                                      const Ogre::Quaternion &orient,
                                                                      float queryDistance)
    {
        return std::make_pair(std::string(), Ogre::Vector3(0.0f));
    }


    bool PhysicsSystem::castRay(const Ogre::Vector3& from, const Ogre::Vector3& to, bool raycastingObjectOnly, bool ignoreHeightMap)
    {
        return false;
    }

    std::pair<bool,Ogre::Vector3> PhysicsSystem::castRay(const Ogre::Vector3 &orig, const Ogre::Vector3 &dir, float len)
    {
        return std::make_pair(false, Ogre::Vector3(0.0f));
    }

    std::pair<bool,Ogre::Vector3> PhysicsSystem::castRay(float mouseX, float mouseY)
    {
        return std::make_pair(false, Ogre::Vector3(0.0f));
    }

    std::vector<std::string> PhysicsSystem::getCollisions(const MWWorld::Ptr &ptr)
    {
        return std::vector<std::string>();
    }

    Ogre::Vector3 PhysicsSystem::traceDown(const MWWorld::Ptr &ptr)
    {
        return Ogre::Vector3(ptr.getRefData().getPosition().pos);
    }


    void PhysicsSystem::queueObjectMovement(const MWWorld::Ptr &ptr, const Ogre::Vector3 &movement)
    {
        ActorMap::iterator aiter = mActors.find(ptr.getRefData().getHandle());
        if(aiter != mActors.end())
        {
            Actor *actor = aiter->second;

            const float *rot = ptr.getRefData().getPosition().rot;
            Ogre::Matrix3 mat3;
            if(0/*isflying || isswimming*/ ||
               !actor->getCollisionObject()->getBroadphaseHandle()->m_collisionFilterMask)
                mat3.FromEulerAnglesZYX(Ogre::Radian(-rot[2]), Ogre::Radian(-rot[1]), Ogre::Radian(rot[0]));
            else
                mat3.FromAngleAxis(Ogre::Vector3::UNIT_Z, Ogre::Radian(-rot[2]));
            Ogre::Vector3 velocity = mat3 * movement;

            actor->updateVelocity(velocity);
        }
    }

    const PtrPositionList& PhysicsSystem::applyQueuedMovement(float dt)
    {
        mMovementResults.clear();

        mTimeAccum += dt;
        if(mTimeAccum >= 1.0f/60.0f)
        {
            mDynamicsWorld->stepSimulation(mTimeAccum, 4, btScalar(1.0f/30.0f));
            mTimeAccum = 0.0f;
        }

        ActorMap::const_iterator aiter = mActors.begin();
        for(;aiter != mActors.end();aiter++)
        {
            Actor *actor = aiter->second;
            btVector3 pos = actor->getOrigin();
            mMovementResults.push_back(std::make_pair(actor->getPtr(), Ogre::Vector3(pos.x(), pos.y(), pos.z())));
            actor->updateVelocity(Ogre::Vector3(0.0f));
        }

        return mMovementResults;
    }


    bool PhysicsSystem::toggleCollisionDebug()
    {
        bool active = !mDebugDraw->getDebugMode();
        mDebugDraw->setDebugMode(active);
        return active;
    }

    void PhysicsSystem::debugDraw() const
    {
        mDebugDraw->update();
    }


    void PhysicsSystem::queueWorldMovement(const MWWorld::Ptr &ptr, const btTransform &worldTrans)
    {
        // TODO: Handle rotation as well. Ogre::Matrix3::ToEulerAnglesXYZ should get what we need.
        const btVector3 &pos = worldTrans.getOrigin();
        mMovementResults.push_back(std::make_pair(ptr, Ogre::Vector3(pos.x(), pos.y(), pos.z())));
    }

}
