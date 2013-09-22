#ifndef MWPHYSICS_CHARACTERCONTROLLER_HPP
#define MWPHYSICS_CHARACTERCONTROLLER_HPP

#include <utility>

#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h>
#include <BulletDynamics/Character/btCharacterControllerInterface.h>

namespace MWPhysics
{

class CharacterController : public btCharacterControllerInterface
{
private:
    btPairCachingGhostObject *mGhostObject;
    btConvexShape *mConvexShape;

    btScalar mVerticalVelocity;
    btScalar mFallSpeed;
    btScalar mJumpSpeed;
    btScalar mMaxSlopeRadians;
    btScalar mMaxSlopeCosine;
    btScalar mGravity;

    btScalar mStepHeight;

    btVector3 mWalkDirection;
    btVector3 mNormalizedDirection;

    btVector3 mCurrentPosition;
    btVector3 mTargetPosition;

    btManifoldArray mManifoldArray;

    bool mWasOnGround;
    bool mWasJumping;
    bool mUseGhostObjectSweepTest;
    bool mUseWalkDirection;
    btScalar mVelocityTimeInterval;
    int mUpAxis;

    btScalar mMass;


    static btVector3 *getUpAxisDirections()
    {
        static btVector3 sUpAxisDirection[3] = { btVector3(1, 0, 0),
                                                 btVector3(0, 1, 0),
                                                 btVector3(0, 0, 1)
                                               };
        return sUpAxisDirection;
    }

    static btVector3 getNormalizedVector(const btVector3 & v)
    {
        btVector3 n = v.normalized();
        if(n.length2() < SIMD_EPSILON)
            n.setValue(0, 0, 0);
        return n;
    }

    btVector3 computeReflectionDirection(const btVector3 &direction, const btVector3 &normal);
    btVector3 parallelComponent(const btVector3 &direction, const btVector3 &normal);
    btVector3 perpindicularComponent(const btVector3 &direction, const btVector3 &normal);

    std::pair<btScalar,btVector3> sweepTrace(btCollisionWorld *collisionWorld, const btTransform &start, const btTransform &end) const;

    void checkOnGround(btCollisionWorld *collisionWorld, bool forceUpdate=false);

    bool stepMove(btCollisionWorld *collisionWorld);

public:
    CharacterController(btPairCachingGhostObject *ghostObject, btConvexShape *convexShape,
                        btScalar stepHeight, int upAxis=1);

    bool recoverFromPenetration(btCollisionWorld *collisionWorld);
    void setRBForceImpulseBasedOnCollision();
    void updateTargetPositionBasedOnCollision(const btVector3 &hitNormal);
    void stepForwardAndStrafe(btCollisionWorld *collisionWorld, const btVector3 &walkMove);
    void setVelocityForTimeInterval(const btVector3 &velocity, btScalar timeInterval);

    void reset()
    {
    }

    void warp(const btVector3 &origin);
    void preStep(btCollisionWorld *collisionWorld);
    void playerStep(btCollisionWorld *collisionWorld, btScalar dt);
    void setFallSpeed(btScalar fallSpeed);
    void setJumpSpeed(btScalar jumpSpeed);
    bool canJump() const;
    void jump();
    void setGravity(const btScalar gravity);
    btScalar getGravity() const;
    void setMaxSlope(btScalar slopeRadians);
    btScalar getMaxSlope() const;
    bool onGround() const;
    void setWalkDirection(const btVector3 &walkDirection);
    void setWalkDirection(const btScalar x, const btScalar y, const btScalar z);
    btVector3 getWalkDirection() const;
    btVector3 getPosition() const;

    btPairCachingGhostObject *getGhostObject() const
    { return mGhostObject; }

    void debugDraw(btIDebugDraw *debugDrawer)
    {
    }

    void updateAction(btCollisionWorld *collisionWorld, btScalar dt);
};

}

#endif // MWPHYSICS_CHARACTERCONTROLLER_HPP
