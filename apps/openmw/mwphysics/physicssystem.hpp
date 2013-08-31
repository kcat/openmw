#ifndef GAME_MWWORLD_PHYSICSSYSTEM_H
#define GAME_MWWORLD_PHYSICSSYSTEM_H

#include <OgreVector3.h>


class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btOverlappingPairCache;
class btBroadphaseInterface;
class btSequentialImpulseConstraintSolver;
class btDiscreteDynamicsWorld;


namespace OEngine
{
    namespace Render
    {
        class OgreRenderer;
    }
}

namespace MWWorld
{
    class Ptr;
    class World;
}

namespace MWPhysics
{
    typedef std::vector<std::pair<MWWorld::Ptr,Ogre::Vector3> > PtrVelocityList;

    class PhysicsSystem
    {
            btDefaultCollisionConfiguration *mCollisionConfiguration;
            btCollisionDispatcher *mDispatcher;

            btOverlappingPairCache *mPairCache;
            btBroadphaseInterface* mBroadphase;
            btSequentialImpulseConstraintSolver *mSolver;
            btDiscreteDynamicsWorld *mDynamicsWorld;

        public:
            PhysicsSystem();
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

        private:

            PtrVelocityList mMovementQueue;
            PtrVelocityList mMovementResults;

            float mTimeAccum;

            PhysicsSystem (const PhysicsSystem&);
            PhysicsSystem& operator= (const PhysicsSystem&);
    };
}

#endif
