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
    class Ray;
}

class btTransform;

class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btOverlappingPairCallback;
class btOverlappingPairCache;
class btBroadphaseInterface;
class btSequentialImpulseConstraintSolver;
class btDiscreteDynamicsWorld;

class btCollisionObject;
class btRigidBody;


namespace MWWorld
{
    class Ptr;
    class World;
}

namespace MWPhysics
{
    class DebugDraw;

    class Heightmap;
    class Object;
    class Actor;

    typedef std::vector<std::pair<MWWorld::Ptr,Ogre::Vector3> > PtrPositionList;

    typedef std::pair<btCollisionObject*,float> ObjectTimePair;
    typedef std::vector<ObjectTimePair> ObjectTimeList;

    class PhysicsSystem
    {
        typedef std::map<std::pair<int,int>,Heightmap*> HeightmapMap;
        typedef std::map<std::string,Object*> ObjectMap;
        typedef std::map<std::string,Actor*> ActorMap;

        btDefaultCollisionConfiguration *mCollisionConfiguration;
        btCollisionDispatcher *mDispatcher;

        btOverlappingPairCallback *mGhostPairCallback;
        btBroadphaseInterface* mBroadphase;
        btSequentialImpulseConstraintSolver *mSolver;
        btDiscreteDynamicsWorld *mDynamicsWorld;

        DebugDraw *mDebugDraw;

        HeightmapMap mHeightmaps;
        ObjectMap mObjects;
        ActorMap mActors;

        ObjectTimeList mTempCollisionObjects;

        public:
            PhysicsSystem(Ogre::SceneManager *sceneMgr);
            ~PhysicsSystem();

            void addObject(const MWWorld::Ptr& ptr, bool placeable=false);

            void addActor(const MWWorld::Ptr& ptr);

            /// Updates the named object/actor with a new Ptr (*MUST* have the same handle!)
            void updateObject(const std::string &handle, const MWWorld::Ptr &ptr);

            // have to keep this as handle for now as unloadcell only knows scenenode names
            void removeObject(const std::string& handle);

            void updatePosition(const MWWorld::Ptr &ptr);

            void updateRotation(const MWWorld::Ptr &ptr);

            void updateScale(const MWWorld::Ptr &ptr);

            void addHeightField(int x, int y, const float *heights, float triSize, int sqrtVerts);

            void removeHeightField(int x, int y);

            bool isOnGround(const MWWorld::Ptr &ptr) const;

            float getActorHeight(const MWWorld::Ptr &ptr) const;

            bool toggleCollisionMode();

            bool getCollisionMode(const MWWorld::Ptr &ptr) const;

            /// get handles this object collides with
            std::vector<std::string> getCollisions(const MWWorld::Ptr &ptr);

            Ogre::Vector3 traceDown(const MWWorld::Ptr &ptr);

            std::pair<MWWorld::Ptr,Ogre::Vector3> getHitContact(const std::string &name,
                                                                const Ogre::Vector3 &origin,
                                                                const Ogre::Quaternion &orientation,
                                                                float queryDistance);

            // cast ray, return true if it hit something. if raycasringObjectOnlt is set to false, it ignores NPCs and objects with no collisions.
            bool castRay(const Ogre::Vector3 &from, const Ogre::Vector3 &to, bool raycastingObjectOnly = true, bool ignoreHeightMap = false);

            std::pair<bool,Ogre::Vector3> castRay(float mouseX, float mouseY);
            ///< cast ray from the mouse, return true if it hit something and the first result (in OGRE coordinates)

            bool getObjectAABB(const MWWorld::Ptr &ptr, Ogre::Vector3 &min, Ogre::Vector3 &max);

            /// Queues velocity movement for \a ptr (which must have been added to the physics
            /// scene via addActor). If \a ptr is already queued, its velocity will be overwritten.
            /// Valid until the next call to applyQueuedMovement.
            void queueActorMovement(const MWWorld::Ptr &ptr, const Ogre::Vector3 &velocity, bool walking);

            const PtrPositionList& applyQueuedMovement(float dt);

            bool toggleCollisionDebug();

            void debugDraw() const;

            /// Queues movement to report back to world. USED ONLY BY PHYSICS OBJECT HANDLERS!
            void queueWorldMovement(const MWWorld::Ptr &ptr, const btTransform &worldTrans);

        private:
            PtrPositionList mMovementResults;

            float mTimeAccum;

            PhysicsSystem(const PhysicsSystem&);
            PhysicsSystem& operator=(const PhysicsSystem&);
    };
}

#endif
