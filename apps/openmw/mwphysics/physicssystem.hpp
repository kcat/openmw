#ifndef MWPHYSICS_PHYSICSSYSTEM_H
#define MWPHYSICS_PHYSICSSYSTEM_H

#include <vector>
#include <string>
#include <map>

namespace Ogre
{
    class Vector3;
    class Quaternion;
    class SceneManager;
}

class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btOverlappingPairCache;
class btBroadphaseInterface;
class btSequentialImpulseConstraintSolver;
class btDiscreteDynamicsWorld;


namespace MWWorld
{
    class Ptr;
    class World;
}

namespace MWPhysics
{
    class DebugDraw;

    class Object;

    typedef std::vector<std::pair<MWWorld::Ptr,Ogre::Vector3> > PtrVelocityList;

    class PhysicsSystem
    {
        typedef std::map<std::string,Object*> ObjectMap;

        btDefaultCollisionConfiguration *mCollisionConfiguration;
        btCollisionDispatcher *mDispatcher;

        btOverlappingPairCache *mPairCache;
        btBroadphaseInterface* mBroadphase;
        btSequentialImpulseConstraintSolver *mSolver;
        btDiscreteDynamicsWorld *mDynamicsWorld;

        DebugDraw *mDebugDraw;

        ObjectMap mObjects;

        public:
            PhysicsSystem(Ogre::SceneManager *sceneMgr);
            ~PhysicsSystem();

            void addObject(const MWWorld::Ptr& ptr, bool placeable=false);

            void addActor(const MWWorld::Ptr& ptr);

            void addHeightField(float *heights, int x, int y, float yoffset,
                                float triSize, float sqrtVerts);

            void removeHeightField(int x, int y);

            // have to keep this as handle for now as unloadcell only knows scenenode names
            void removeObject(const std::string& handle);

            void moveObject(const MWWorld::Ptr& ptr);

            void rotateObject(const MWWorld::Ptr& ptr);

            void scaleObject(const MWWorld::Ptr& ptr);

            bool toggleCollisionMode();

            /// get handles this object collides with
            std::vector<std::string> getCollisions(const MWWorld::Ptr &ptr);

            Ogre::Vector3 traceDown(const MWWorld::Ptr &ptr);

            std::pair<float,std::string> getFacedHandle(float queryDistance);
            std::pair<std::string,Ogre::Vector3> getHitContact(const std::string &name,
                                                               const Ogre::Vector3 &origin,
                                                               const Ogre::Quaternion &orientation,
                                                               float queryDistance);
            std::vector<std::pair<float,std::string> > getFacedHandles(float queryDistance);
            std::vector<std::pair<float,std::string> > getFacedHandles(float mouseX, float mouseY, float queryDistance);

            // cast ray, return true if it hit something. if raycasringObjectOnlt is set to false, it ignores NPCs and objects with no collisions.
            bool castRay(const Ogre::Vector3 &from, const Ogre::Vector3 &to, bool raycastingObjectOnly = true, bool ignoreHeightMap = false);

            std::pair<bool,Ogre::Vector3> castRay(const Ogre::Vector3 &orig, const Ogre::Vector3 &dir, float len);

            std::pair<bool,Ogre::Vector3> castRay(float mouseX, float mouseY);
            ///< cast ray from the mouse, return true if it hit something and the first result (in OGRE coordinates)

            bool getObjectAABB(const MWWorld::Ptr &ptr, Ogre::Vector3 &min, Ogre::Vector3 &max);

            /// Queues velocity movement for a Ptr. If a Ptr is already queued, its velocity will
            /// be overwritten. Valid until the next call to applyQueuedMovement.
            void queueObjectMovement(const MWWorld::Ptr &ptr, const Ogre::Vector3 &velocity);

            const PtrVelocityList& applyQueuedMovement(float dt);

            bool toggleCollisionDebug();

            void debugDraw() const;

        private:
            PtrVelocityList mMovementQueue;
            PtrVelocityList mMovementResults;

            float mTimeAccum;

            PhysicsSystem(const PhysicsSystem&);
            PhysicsSystem& operator=(const PhysicsSystem&);
    };
}

#endif
