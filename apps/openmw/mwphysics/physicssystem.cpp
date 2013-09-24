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
    class ClosestNotMeRayResultCallback : public btCollisionWorld::ClosestRayResultCallback
    {
    private:
        btCollisionObject *mMe;

    public:
        ClosestNotMeRayResultCallback(btCollisionObject *me)
          : btCollisionWorld::ClosestRayResultCallback(btVector3(0, 0, 0), btVector3(0, 0, 0))
          , mMe(me)
        { }

        btScalar addSingleResult(btCollisionWorld::LocalRayResult &rayResult, bool normalInWorldSpace)
        {
            if(rayResult.m_collisionObject == mMe)
                return 1.0f;

            return btCollisionWorld::ClosestRayResultCallback::addSingleResult(rayResult, normalInWorldSpace);
        }
    };

    class DeepestNotMeContactTestResultCallback : public btCollisionWorld::ContactResultCallback
    {
        btCollisionObject *mMe;
        // Store the real origin, since the shape's origin is its center
        btVector3 mOrigin;

    public:
        MWWorld::Ptr mObject;
        btVector3 mContactPoint;
        btScalar mLeastDistSqr;

        DeepestNotMeContactTestResultCallback(btCollisionObject *me, const btVector3 &origin)
          : mMe(me), mOrigin(origin), mContactPoint(0,0,0),
            mLeastDistSqr(std::numeric_limits<float>::max())
        { }

        virtual btScalar addSingleResult(btManifoldPoint& cp,
                                         const btCollisionObject* col0, int partId0, int index0,
                                         const btCollisionObject* col1, int partId1, int index1)
        {
            const MWPhysics::ObjectInfo *info = static_cast<const MWPhysics::ObjectInfo*>(col1->getUserPointer());
            if(info && col1 != mMe)
            {
                btScalar distsqr = mOrigin.distance2(cp.getPositionWorldOnA());
                if(mObject.isEmpty() || distsqr < mLeastDistSqr)
                {
                    mObject = info->getPtr();
                    mLeastDistSqr = distsqr;
                    mContactPoint = cp.getPositionWorldOnA();
                }
            }

            return 0.f;
        }

        // No version macros to check, so we have to instead check for the header
        // btCollisionObjectWrapper is declared in.
#ifdef BT_COLLISION_OBJECT_WRAPPER_H
        virtual btScalar addSingleResult(btManifoldPoint &cp,
                                         const btCollisionObjectWrapper *col0, int partId0, int index0,
                                         const btCollisionObjectWrapper *col1, int partId1, int index1)
        {
            return addSingleResult(cp, col0->getCollisionObject(), partId0, index0,
                                       col1->getCollisionObject(), partId1, index1);
        }
#endif
    };

    class ClearObjectTimePairs {
        btCollisionWorld *mCollisionWorld;

    public:
        ClearObjectTimePairs(btCollisionWorld *collisionWorld)
          : mCollisionWorld(collisionWorld)
        { }

        void operator()(MWPhysics::ObjectTimePair &objtime) const
        {
            mCollisionWorld->removeCollisionObject(objtime.first);
            delete objtime.first->getCollisionShape();
            delete objtime.first;
        }
    };

    class ProcessObjectTimePairs {
        btCollisionWorld *mCollisionWorld;
        float mTimePassed;

    public:
        ProcessObjectTimePairs(btCollisionWorld *collisionWorld, float dt)
          : mCollisionWorld(collisionWorld), mTimePassed(dt)
        { }

        bool operator()(MWPhysics::ObjectTimePair &objtime) const
        {
            objtime.second -= mTimePassed;
            if(objtime.second > 0.0f)
                return false;

            mCollisionWorld->removeCollisionObject(objtime.first);
            delete objtime.first->getCollisionShape();
            delete objtime.first;
            objtime.first = 0;

            return true;
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
        Heightmap(int x, int y, const float *heights, float triSize, int sqrtVerts)
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
                                  btVector3((x+0.5f) * triSize * (sqrtVerts-1),
                                            (y+0.5f) * triSize * (sqrtVerts-1),
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
        std::for_each(mTempCollisionObjects.begin(), mTempCollisionObjects.end(),
                      ClearObjectTimePairs(mDynamicsWorld));
        mTempCollisionObjects.clear();

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

    void PhysicsSystem::updatePosition(const MWWorld::Ptr &ptr)
    {
        ActorMap::iterator aiter(mActors.find(ptr.getRefData().getHandle()));
        if(aiter != mActors.end())
        {
            const Ogre::Vector3 &pos = ptr.getRefData().getBaseNode()->getPosition();

            Actor *actor = aiter->second;
            actor->setOrigin(btVector3(pos.x, pos.y, pos.z));
        }
    }

    void PhysicsSystem::updateRotation(const MWWorld::Ptr &ptr)
    {
        ObjectMap::iterator oiter = mObjects.find(ptr.getRefData().getHandle());
        if(oiter != mObjects.end())
        {
            Object *object = oiter->second;

            const Ogre::Quaternion &rot = ptr.getRefData().getBaseNode()->getOrientation();
            object->setOrientation(rot);
            return;
        }

        /* Nothing to do for actors. */
    }

    void PhysicsSystem::updateScale(const MWWorld::Ptr &ptr)
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


    void PhysicsSystem::addHeightField(int x, int y, const float *heights, float triSize, int sqrtVerts)
    {
        Heightmap *heightmap = new Heightmap(x, y, heights, triSize, sqrtVerts);
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


    bool PhysicsSystem::isOnGround(const MWWorld::Ptr &ptr) const
    {
        ActorMap::const_iterator aiter(mActors.find(ptr.getRefData().getHandle()));
        if(aiter != mActors.end())
        {
            Actor *actor = aiter->second;
            return actor->getActionInterface()->onGround();
        }

        return false;
    }


    float PhysicsSystem::getActorHeight(const MWWorld::Ptr &ptr) const
    {
        ActorMap::const_iterator aiter(mActors.find(ptr.getRefData().getHandle()));
        if(aiter != mActors.end())
        {
            Actor *actor = aiter->second;
            return actor->getHeight();
        }

        return 0.0f;
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
            actor->getActionInterface()->setFallSpeed(-4005.5f);
        }
        else
        {
            actor->getActionInterface()->setGravity(0.0f);
            actor->getActionInterface()->setFallSpeed(0.0f);
        }

        return on;
    }

    bool PhysicsSystem::getCollisionMode(const MWWorld::Ptr &ptr) const
    {
        ActorMap::const_iterator aiter = mActors.find(ptr.getRefData().getHandle());
        if(aiter == mActors.end())
            return false;

        Actor *actor = aiter->second;
        return !!actor->getCollisionObject()->getBroadphaseHandle()->m_collisionFilterMask;
    }


    bool PhysicsSystem::getObjectAABB(const MWWorld::Ptr &ptr, Ogre::Vector3 &min, Ogre::Vector3 &max)
    {
        NifBullet::BulletShapeManager &shapeMgr = NifBullet::BulletShapeManager::getSingleton();
        const std::string name = Misc::StringUtils::lowerCase(ptr.getClass().getModel(ptr));
        NifBullet::BulletShapePtr shape = shapeMgr.load(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        if(shape->getCollisionShape())
        {
            float scale = ptr.getCellRef().mScale;
            btVector3 btmin, btmax;
            shape->getCollisionShape()->getAabb(btTransform::getIdentity(), btmin, btmax);
            min = Ogre::Vector3(btmin.x(), btmin.y(), btmin.z()) * scale;
            max = Ogre::Vector3(btmax.x(), btmax.y(), btmax.z()) * scale;
            return true;
        }

        return false;
    }


    std::pair<MWWorld::Ptr,Ogre::Vector3> PhysicsSystem::getHitContact(const std::string &name,
                                                                      const Ogre::Vector3 &origin,
                                                                      const Ogre::Quaternion &orient,
                                                                      float queryDistance)
    {
        ActorMap::const_iterator aiter = mActors.find(name);
        if(aiter == mActors.end())
            return std::make_pair(MWWorld::Ptr(), Ogre::Vector3(0.0f));
        Actor *actor = aiter->second;

        const MWWorld::Store<ESM::GameSetting> &store = MWBase::Environment::get().getWorld()->getStore().get<ESM::GameSetting>();

        btConeShape *shape;
        shape = new btConeShape(Ogre::Degree(store.find("fCombatAngleXY")->getFloat()/2.0f).valueRadians(),
                                queryDistance);
        shape->setLocalScaling(btVector3(1, 1, Ogre::Degree(store.find("fCombatAngleZ")->getFloat()/2.0f).valueRadians() /
                                               shape->getRadius()));

        // The shape origin is its center, so we have to move it forward by half the length. The
        // real origin will be provided to the callback to find the closest.
        Ogre::Vector3 center = origin + (orient * Ogre::Vector3(0.0f, queryDistance*0.5f, 0.0f));

        btCollisionObject *object = new btCollisionObject();
        object->setCollisionShape(shape);
        object->setWorldTransform(btTransform(btQuaternion(orient.x, orient.y, orient.z, orient.w),
                                              btVector3(center.x, center.y, center.z)));

        DeepestNotMeContactTestResultCallback callback(actor->getCollisionObject(),
                                                       btVector3(origin.x, origin.y, origin.z));
        mDynamicsWorld->contactTest(object, callback);

        // Temporarily add the contact object to the dynamics world so it can be seen in the debug
        // draw (no group or mask so nothing will hit it later).
        mDynamicsWorld->addCollisionObject(object, 0, 0);
        mTempCollisionObjects.push_back(std::make_pair(object, 1.0f));

        return std::make_pair(callback.mObject, Ogre::Vector3(&callback.mContactPoint[0]));
    }


    bool PhysicsSystem::castRay(const Ogre::Vector3& from, const Ogre::Vector3& to, bool raycastingObjectOnly, bool ignoreHeightMap)
    {
        return true;
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
        ActorMap::iterator aiter = mActors.find(ptr.getRefData().getHandle());
        if(aiter != mActors.end())
        {
            const float *pos = ptr.getRefData().getPosition().pos;
            btVector3 posFrom(pos[0], pos[1], pos[2]);
            btVector3 posTo(pos[0], pos[1], pos[2]-200.0f);

            Actor *actor = aiter->second;
            ClosestNotMeRayResultCallback callback(actor->getCollisionObject());
            mDynamicsWorld->rayTest(posFrom, posTo, callback);

            if(callback.hasHit())
            {
                btVector3 res = posFrom.lerp(posTo, callback.m_closestHitFraction);
                return Ogre::Vector3(res.x(), res.y(), res.z());
            }
        }

        return Ogre::Vector3(ptr.getRefData().getPosition().pos);
    }


    void PhysicsSystem::queueActorMovement(const MWWorld::Ptr &ptr, const Ogre::Vector3 &movement, bool walking)
    {
        ActorMap::iterator aiter = mActors.find(ptr.getRefData().getHandle());
        if(aiter != mActors.end())
        {
            Actor *actor = aiter->second;
            Ogre::Vector3 velocity;

            const float *rot = ptr.getRefData().getPosition().rot;
            Ogre::Matrix3 mat3;
            if(walking)
            {
                mat3.FromAngleAxis(Ogre::Vector3::UNIT_Z, Ogre::Radian(-rot[2]));
                if(movement.z > 0.0f)
                    actor->jump(movement.z);
                velocity = mat3 * Ogre::Vector3(movement.x, movement.y, 0.0f);
            }
            else
            {
                mat3.FromEulerAnglesZYX(Ogre::Radian(-rot[2]), Ogre::Radian(-rot[1]), Ogre::Radian(rot[0]));
                velocity = mat3 * movement;
            }

            actor->updateVelocity(velocity);
        }
    }

    const PtrPositionList& PhysicsSystem::applyQueuedMovement(float dt)
    {
        ObjectTimeList::iterator otiter = mTempCollisionObjects.begin();
        while((otiter=std::find_if(otiter, mTempCollisionObjects.end(),
                                   ProcessObjectTimePairs(mDynamicsWorld, dt))) != mTempCollisionObjects.end())
            otiter = mTempCollisionObjects.erase(otiter);

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
