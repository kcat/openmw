#include "physicssystem.hpp"

#include <stdexcept>

#include <OgreVector3.h>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#include <components/nifbullet/bulletshape.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"

#include "../mwworld/class.hpp"

#include "debugdraw.hpp"
#include "object.hpp"
#include "actor.hpp"


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
        btMotionState *mMotionState;
        btCollisionObject *mCollisionObject;

    public:
        Heightmap(int x, int y, const float *heights, float yoffset, float triSize, int sqrtVerts)
          : ObjectInfo(MWWorld::Ptr())
          , mShape(0)
          , mMotionState(0)
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
            mMotionState = new btDefaultMotionState(transform);

            btRigidBody::btRigidBodyConstructionInfo cinf = btRigidBody::btRigidBodyConstructionInfo(0.0f, mMotionState, mShape);
            mCollisionObject = new btRigidBody(cinf);

            // Explicitly downcast to ensure the user pointer can be directly casted back to
            // ObjectInfo.
            mCollisionObject->setUserPointer(static_cast<ObjectInfo*>(this));
        }

        virtual ~Heightmap()
        {
            delete mCollisionObject;
            delete mMotionState;
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
        mPairCache = new btSortedOverlappingPairCache();
        mBroadphase = new btDbvtBroadphase(mPairCache);
        mDynamicsWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadphase, mSolver, mCollisionConfiguration);
        mDynamicsWorld->setGravity(btVector3(0.0f, 0.0f, -627.2f));

        mDebugDraw = new DebugDraw(sceneMgr, mDynamicsWorld);
    }

    PhysicsSystem::~PhysicsSystem()
    {
        // Delete in reverse order that they were new'd
        delete mDebugDraw;
        delete mDynamicsWorld;
        delete mBroadphase;
        delete mPairCache;
        delete mSolver;
        delete mDispatcher;
        delete mCollisionConfiguration;
    }


    void PhysicsSystem::addObject(const MWWorld::Ptr &ptr, bool placeable)
    {
        NifBullet::BulletShapeManager &shapeMgr = NifBullet::BulletShapeManager::getSingleton();
        std::string name = ptr.getClass().getModel(ptr);
        NifBullet::BulletShapePtr shape = shapeMgr.load(Misc::StringUtils::toLower(name),
                                                        Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);

        if(!shape->getCollisionShape())
            return;

        if(placeable && !shape->hasRootCollision())
            return;

        Object *obj = new Object(ptr, shape, this);
        mObjects.insert(std::make_pair(ptr.getRefData().getHandle(), obj));

        mDynamicsWorld->addCollisionObject(obj->getCollisionObject());
    }

    void PhysicsSystem::addActor(const MWWorld::Ptr &ptr)
    {
        NifBullet::BulletShapeManager &shapeMgr = NifBullet::BulletShapeManager::getSingleton();
        std::string name = ptr.getClass().getModel(ptr);
        NifBullet::BulletShapePtr shape = shapeMgr.load(Misc::StringUtils::toLower(name),
                                                        Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);

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
            mDynamicsWorld->removeCollisionObject(actor->second->getCollisionObject());
            delete actor->second;
            mActors.erase(actor);
            return;
        }
    }

    void PhysicsSystem::moveObject(const MWWorld::Ptr& ptr)
    {
    }

    void PhysicsSystem::rotateObject(const MWWorld::Ptr& ptr)
    {
    }

    void PhysicsSystem::scaleObject(const MWWorld::Ptr& ptr)
    {
        ObjectMap::iterator obj(mObjects.find(ptr.getRefData().getHandle()));
        if(obj != mObjects.end())
        {
            Object *object = obj->second;
            mDynamicsWorld->removeCollisionObject(object->getCollisionObject());
            object->resetCollisionObject();
            mDynamicsWorld->addCollisionObject(object->getCollisionObject());
            return;
        }

        ActorMap::iterator act(mActors.find(ptr.getRefData().getHandle()));
        if(act != mActors.end())
        {
            Actor *actor = act->second;
            mDynamicsWorld->removeCollisionObject(actor->getCollisionObject());
            actor->resetCollisionObject();
            mDynamicsWorld->addCollisionObject(actor->getCollisionObject());
            return;
        }
    }


    void PhysicsSystem::addHeightField(int x, int y, const float *heights, float yoffset,
                                       float triSize, int sqrtVerts)
    {
        Heightmap *heightmap = new Heightmap(x, y, heights, yoffset, triSize, sqrtVerts);
        mHeightmaps[std::make_pair(x,y)] = heightmap;

        mDynamicsWorld->addCollisionObject(heightmap->getCollisionObject());
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
        return false;
    }


    std::pair<float,MWWorld::Ptr> PhysicsSystem::getFacedHandle(const MWWorld::Ptr &filter, const Ogre::Ray &ray, float queryDistance) const
    {
        const Ogre::Vector3 &origin_ = ray.getOrigin();
        const Ogre::Vector3 dest_ = ray.getPoint(queryDistance);

        btVector3 from(origin_.x, origin_.y, origin_.z);
        btVector3 to(dest_.x, dest_.y, dest_.z);
        FilteredRayResultCallback callback(filter);
        mDynamicsWorld->rayTest(from, to, callback);

        if(!(callback.m_closestHitFraction < 1.0f))
            return std::make_pair(queryDistance,MWWorld::Ptr());
        const ObjectInfo *info = static_cast<const ObjectInfo*>(callback.m_collisionObject->getUserPointer());
        return std::make_pair(callback.m_closestHitFraction*queryDistance, info->getPtr());
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
        PtrVelocityList::iterator iter = mMovementQueue.begin();
        for(;iter != mMovementQueue.end();iter++)
        {
            if(iter->first == ptr)
            {
                iter->second = movement;
                return;
            }
        }

        mMovementQueue.push_back(std::make_pair(ptr, movement));
    }

    const PtrVelocityList& PhysicsSystem::applyQueuedMovement(float dt)
    {
        mMovementResults.clear();

        mTimeAccum += dt;
        if(mTimeAccum >= 1.0f/60.0f)
        {
            const MWBase::World *world = MWBase::Environment::get().getWorld();
            PtrVelocityList::iterator iter = mMovementQueue.begin();
            for(;iter != mMovementQueue.end();iter++)
            {
                const ESM::Position &refpos = iter->first.getRefData().getPosition();
                Ogre::Vector3 position(refpos.pos);

                Ogre::Matrix3 mat3;
                mat3.FromEulerAnglesZYX(Ogre::Radian(-refpos.rot[2]), Ogre::Radian(-refpos.rot[1]), Ogre::Radian(refpos.rot[0]));
                Ogre::Vector3 newpos = mat3 * iter->second*mTimeAccum + position;

                ActorMap::iterator actor = mActors.find(iter->first.getRefData().getHandle());
                if(actor != mActors.end())
                {
                    // Rotate actors around Z only
                    mat3.FromAngleAxis(Ogre::Vector3::UNIT_Z, Ogre::Radian(-refpos.rot[2]));
                    btTransform trans(btMatrix3x3(mat3[0][0], mat3[0][1], mat3[0][2],
                                                  mat3[1][0], mat3[1][1], mat3[1][2],
                                                  mat3[2][0], mat3[2][1], mat3[2][2]),
                                      btVector3(newpos.x, newpos.y, newpos.z));
                    actor->second->updateTransform(trans);
                }

                // Bullet does not seem to call btMotionState::setWorldTransform for kinematic
                // objects, so queue world movement here.
                mMovementResults.push_back(std::make_pair(iter->first, newpos));
            }

            mDynamicsWorld->stepSimulation(mTimeAccum);
            mTimeAccum = 0.0f;
        }
        mMovementQueue.clear();

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
