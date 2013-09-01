#include "physicssystem.hpp"

#include <stdexcept>

#include <OgreVector3.h>

#include <btBulletDynamicsCommon.h>

#include <components/nifbullet/bulletshape.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"

#include "../mwworld/class.hpp"

#include "object.hpp"


namespace MWPhysics
{
    PhysicsSystem::PhysicsSystem() : mTimeAccum(0.0f)
    {
        mCollisionConfiguration = new btDefaultCollisionConfiguration();
        mDispatcher = new btCollisionDispatcher(mCollisionConfiguration);

        mSolver = new btSequentialImpulseConstraintSolver();
        mPairCache = new btSortedOverlappingPairCache();
        mBroadphase = new btDbvtBroadphase();
        mDynamicsWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadphase, mSolver, mCollisionConfiguration);
        mDynamicsWorld->setGravity(btVector3(0.0f, 0.0f, -627.2f));
    }

    PhysicsSystem::~PhysicsSystem()
    {
        // Delete in reverse order that they were new'd
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
        NifBullet::BulletShapePtr shape = shapeMgr.load(ptr.getClass().getModel(ptr),
                                                        Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);

        const ESM::Position &pos = ptr.getRefData().getPosition();
        btQuaternion ori;
        ori.setEulerZYX(pos.rot[2], pos.rot[1], pos.rot[0]);
        btTransform trans(ori, btVector3(pos.pos[0], pos.pos[1], pos.pos[2]));

        Object *obj = new Object(shape, trans);
        mObjects.insert(std::make_pair(ptr.getRefData().getHandle(), obj));

        mDynamicsWorld->addCollisionObject(obj->getCollisionObject());
    }

    void PhysicsSystem::addActor(const MWWorld::Ptr &ptr)
    {
    }

    void PhysicsSystem::removeObject(const std::string &handle)
    {
        ObjectMap::iterator obj = mObjects.find(handle);
        if(obj != mObjects.end())
        {
            mDynamicsWorld->removeCollisionObject(obj->second->getCollisionObject());
            mObjects.erase(obj);
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
    }

    bool PhysicsSystem::toggleCollisionMode()
    {
        return false;
    }

    bool PhysicsSystem::getObjectAABB(const MWWorld::Ptr &ptr, Ogre::Vector3 &min, Ogre::Vector3 &max)
    {
        return false;
    }


    std::pair<float,std::string> PhysicsSystem::getFacedHandle(float queryDistance)
    {
        return std::make_pair(1.0f, std::string());
    }

    std::vector<std::pair<float,std::string> > PhysicsSystem::getFacedHandles(float queryDistance)
    {
        return std::vector<std::pair<float,std::string> >();
    }

    std::vector<std::pair<float,std::string> > PhysicsSystem::getFacedHandles(float mouseX, float mouseY, float queryDistance)
    {
        return std::vector<std::pair<float,std::string> >();
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

    void PhysicsSystem::addHeightField(float *heights, int x, int y, float yoffset,
                                       float triSize, float sqrtVerts)
    {
    }

    void PhysicsSystem::removeHeightField(int x, int y)
    {
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

                Ogre::Vector3 newpos = (Ogre::Quaternion(Ogre::Radian(refpos.rot[2]), Ogre::Vector3::NEGATIVE_UNIT_Z)*
                                        Ogre::Quaternion(Ogre::Radian(refpos.rot[1]), Ogre::Vector3::NEGATIVE_UNIT_Y)*
                                        Ogre::Quaternion(Ogre::Radian(refpos.rot[0]), Ogre::Vector3::UNIT_X)) *
                                       iter->second*mTimeAccum + position;
                mMovementResults.push_back(std::make_pair(iter->first, newpos));
            }

            mDynamicsWorld->stepSimulation(mTimeAccum);
            mTimeAccum = 0.0f;
        }
        mMovementQueue.clear();

        return mMovementResults;
    }
}
