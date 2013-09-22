#include "charactercontroller.hpp"

#include <iostream>

#include <BulletCollision/CollisionShapes/btConvexShape.h>


namespace
{
    class ClosestNotMeConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback
    {
    private:
        btCollisionObject *mMe;
        const btVector3 mUp;
        btScalar mMinSlopeDot;

    public:
        ClosestNotMeConvexResultCallback(btCollisionObject *me, const btVector3 &up, btScalar minSlopeDot)
          : btCollisionWorld::ClosestConvexResultCallback(btVector3(0, 0, 0), btVector3(0, 0, 0))
          , mMe(me)
          , mUp(up)
          , mMinSlopeDot(minSlopeDot)
        { }

        btScalar addSingleResult(btCollisionWorld::LocalConvexResult &convexResult, bool normalInWorldSpace)
        {
            if(convexResult.m_hitCollisionObject == mMe)
                return 1.0f;

            return btCollisionWorld::ClosestConvexResultCallback::addSingleResult(convexResult, normalInWorldSpace);
        }
    };
}

namespace MWPhysics
{

btVector3 CharacterController::computeReflectionDirection(const btVector3 &direction, const btVector3 &normal)
{
    return direction - (btScalar(2) * direction.dot(normal)) * normal;
}

btVector3 CharacterController::parallelComponent(const btVector3 &direction, const btVector3 &normal)
{
    btScalar magnitude = direction.dot(normal);
    return normal * magnitude;
}

btVector3 CharacterController::perpindicularComponent(const btVector3 &direction, const btVector3 &normal)
{
    return direction - parallelComponent(direction, normal);
}


std::pair<btScalar,btVector3> CharacterController::sweepTrace(btCollisionWorld *collisionWorld, const btTransform &start, const btTransform &end) const
{
    ClosestNotMeConvexResultCallback callback(mGhostObject, getUpAxisDirections()[mUpAxis], mMaxSlopeCosine);
    callback.m_collisionFilterGroup = getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup;
    callback.m_collisionFilterMask = getGhostObject()->getBroadphaseHandle()->m_collisionFilterMask;

    if(mUseGhostObjectSweepTest)
        mGhostObject->convexSweepTest(mConvexShape, start, end, callback, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);
    else
        collisionWorld->convexSweepTest(mConvexShape, start, end, callback, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);

    if(callback.hasHit())
        return std::make_pair(callback.m_closestHitFraction, callback.m_hitNormalWorld);
    return std::make_pair(btScalar(1.0f), getUpAxisDirections()[mUpAxis]);
}


void CharacterController::checkOnGround(btCollisionWorld *collisionWorld, bool forceUpdate)
{
    mWasOnGround = false;

    if(!(mGravity < 0.0f && mVerticalVelocity <= 0.0f))
        return;

    btHashedOverlappingPairCache *pairCache = mGhostObject->getOverlappingPairCache();
    if(forceUpdate)
    {
        btDispatcher *dispatcher = collisionWorld->getDispatcher();
        dispatcher->dispatchAllCollisionPairs(pairCache, collisionWorld->getDispatchInfo(), dispatcher);
    }

    const btBroadphasePairArray &collisionArray = pairCache->getOverlappingPairArray();
    int numpairs = pairCache->getNumOverlappingPairs();
    for(int i = 0;i < numpairs;++i)
    {
        mManifoldArray.resize(0);

        const btBroadphasePair &collisionPair = collisionArray[i];
        if(collisionPair.m_algorithm)
            collisionPair.m_algorithm->getAllContactManifolds(mManifoldArray);

        for(int j = 0;j < mManifoldArray.size();++j)
        {
            btPersistentManifold *manifold = mManifoldArray[j];

            btScalar dir = ((manifold->getBody0() == mGhostObject) ? btScalar(1.0f) : btScalar(-1.0f));
            for(int p = 0;p < manifold->getNumContacts();++p)
            {
                const btManifoldPoint &pt = manifold->getContactPoint(p);
                if(pt.getDistance() < 1.0f && getSlope(pt.m_normalWorldOnB * dir) <= mMaxSlopeRadians)
                {
                    mWasOnGround = true;
                    return;
                }
            }
        }
    }
}


bool CharacterController::stepMove(btCollisionWorld *collisionWorld)
{
    btVector3 moveDir = mTargetPosition - mCurrentPosition;

    // Step up
    btTransform start(btMatrix3x3::getIdentity(), mCurrentPosition + getUpAxisDirections()[mUpAxis]);
    btTransform end(btMatrix3x3::getIdentity(), mCurrentPosition + getUpAxisDirections()[mUpAxis]*mStepHeight);
    std::pair<btScalar,btVector3> res = sweepTrace(collisionWorld, start, end);
    if(res.first < SIMD_EPSILON)
        return false;
    btVector3 raisedPos = start.getOrigin().lerp(end.getOrigin(), res.first);

    // Try to keep moving
    start.setOrigin(raisedPos);
    end.setOrigin(raisedPos + moveDir);
    res = sweepTrace(collisionWorld, start, end);
    if(res.first < SIMD_EPSILON)
        return false;
    btScalar movedone = res.first;
    btVector3 movedPos = start.getOrigin().lerp(end.getOrigin(), res.first);

    // Set down
    start.setOrigin(movedPos);
    end.setOrigin(movedPos - getUpAxisDirections()[mUpAxis]*(mStepHeight+0.1f));
    res = sweepTrace(collisionWorld, start, end);
    if(res.first >= 1.0f || getSlope(res.second) > mMaxSlopeRadians)
        return false;

    mCurrentPosition = start.getOrigin().lerp(end.getOrigin(), res.first);
    mTargetPosition = mCurrentPosition + moveDir*(1.0f - movedone);
    return true;
}


CharacterController::CharacterController(btPairCachingGhostObject *ghostObject, btConvexShape *convexShape,
                                         btScalar stepHeight, int upAxis)
  : mGhostObject(ghostObject)
  , mConvexShape(convexShape)
  , mVerticalVelocity(0.0f)
  , mFallSpeed(-62720.0f /*100x gravity*/)
  , mJumpSpeed(10.0f)
  , mGravity(-627.2f)
  , mStepHeight(stepHeight)
  , mWalkDirection(0.0f, 0.0f, 0.0f)
  , mNormalizedDirection(0.0f, 0.0f, 0.0f)
  , mCurrentPosition(0.0f, 0.0f, 0.0f)
  , mWasOnGround(false)
  , mWasJumping(false)
  , mUseGhostObjectSweepTest(true)
  , mUseWalkDirection(true)
  , mVelocityTimeInterval(0.0f)
  , mUpAxis(upAxis)
  , mMass(20.0f)
{
    setMaxSlope(btRadians(60));
}

bool CharacterController::recoverFromPenetration(btCollisionWorld *collisionWorld)
{
    bool penetration = false;

    btDispatcher *dispatcher = collisionWorld->getDispatcher();
    btHashedOverlappingPairCache *pairCache = mGhostObject->getOverlappingPairCache();
    dispatcher->dispatchAllCollisionPairs(pairCache, collisionWorld->getDispatchInfo(), dispatcher);

    mCurrentPosition = mGhostObject->getWorldTransform().getOrigin();

    const btBroadphasePairArray &pairArray = pairCache->getOverlappingPairArray();
    int numpairs = pairCache->getNumOverlappingPairs();
    for(int i = 0;i < numpairs;++i)
    {
        mManifoldArray.resize(0);

        const btBroadphasePair &collisionPair = pairArray[i];
        if(collisionPair.m_algorithm)
            collisionPair.m_algorithm->getAllContactManifolds(mManifoldArray);

        for(int j = 0;j < mManifoldArray.size();++j)
        {
            btPersistentManifold *manifold = mManifoldArray[j];
            btScalar directionSign = (manifold->getBody0() == mGhostObject) ? btScalar(-1.0) : btScalar(1.0);
            for(int p = 0;p < manifold->getNumContacts();++p)
            {
                const btManifoldPoint &pt = manifold->getContactPoint(p);

                btScalar dist = pt.getDistance();
                if(dist <= -mGhostObject->getCollisionShape()->getMargin())
                {
                    mCurrentPosition += pt.m_normalWorldOnB * directionSign * dist * btScalar(0.2);
                    penetration = true;
                }
            }
        }
    }

    btTransform newTrans = mGhostObject->getWorldTransform();
    newTrans.setOrigin(mCurrentPosition);
    mGhostObject->setWorldTransform(newTrans);

    return penetration;
}

void CharacterController::setRBForceImpulseBasedOnCollision()
{
    if(!mWalkDirection.isZero())
    {
        const btBroadphasePairArray &collisionArray = mGhostObject->getOverlappingPairCache()->getOverlappingPairArray();
        for(int i = 0;i < collisionArray.size();i++)
        {
            const btBroadphasePair &collisionPair = collisionArray[i];

            btCollisionObject *obj = (btCollisionObject*)collisionPair.m_pProxy1->m_clientObject;
            btRigidBody *rb = btRigidBody::upcast(obj);
            if(rb && rb->getInvMass() != btScalar(0.0f) && mMass > rb->getInvMass())
            {
                btScalar resultMass = mMass - rb->getInvMass();
                btVector3 reflection = computeReflectionDirection(mWalkDirection * resultMass,
                                                                  getNormalizedVector(mWalkDirection));
                rb->applyCentralImpulse(reflection * -1);
            }
        }
    }
}

void CharacterController::updateTargetPositionBasedOnCollision(const btVector3 &hitNormal)
{
    btVector3 movementDirection = mTargetPosition - mCurrentPosition;
    btScalar movementLength = movementDirection.length();

    if(!(movementLength < SIMD_EPSILON))
    {
        movementDirection.normalize();

        btVector3 reflectdir = computeReflectionDirection(movementDirection, hitNormal);
        reflectdir.normalize();
        movementDirection = perpindicularComponent(reflectdir, hitNormal)*movementLength;

        mTargetPosition = mCurrentPosition + movementDirection;
    }
}

void CharacterController::stepForwardAndStrafe(btCollisionWorld *collisionWorld, const btVector3 &walkMove)
{
    btTransform start, end;
    start.setIdentity();
    end.setIdentity();

    mTargetPosition = mCurrentPosition + walkMove;
    btScalar distance2 = (mCurrentPosition - mTargetPosition).length2();
    int maxIter = 10;
    while(!(distance2 < SIMD_EPSILON) && maxIter-- > 0)
    {
        start.setOrigin(mCurrentPosition);
        end.setOrigin(mTargetPosition);

        btVector3 sweepDirNegative(mCurrentPosition - mTargetPosition);

        ClosestNotMeConvexResultCallback callback(mGhostObject, sweepDirNegative, 0);
        callback.m_collisionFilterGroup = getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup;
        callback.m_collisionFilterMask = getGhostObject()->getBroadphaseHandle()->m_collisionFilterMask;
        if (mUseGhostObjectSweepTest)
            mGhostObject->convexSweepTest(mConvexShape, start, end, callback, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);
        else
            collisionWorld->convexSweepTest(mConvexShape, start, end, callback, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);

        if(!callback.hasHit())
        {
            // Didn't hit anything, reached the target.
            mCurrentPosition = mTargetPosition;
            break;
        }

        // If we moved at all and hit a walkable surface, accept how far we got.
        if(callback.m_closestHitFraction > SIMD_EPSILON &&
           getSlope(callback.m_hitNormalWorld) <= mMaxSlopeRadians)
        {
            mCurrentPosition = mCurrentPosition.lerp(mTargetPosition, callback.m_closestHitFraction);
            mWasOnGround = (mGravity < 0.0f && mVerticalVelocity <= 0.0f);
            distance2 = (mCurrentPosition - mTargetPosition).length2();
            continue;
        }

        // Otherwise, try to step up onto what we hit.
        if(stepMove(collisionWorld))
        {
            mWasOnGround = (mGravity < 0.0f && mVerticalVelocity <= 0.0f);
            distance2 = (mCurrentPosition - mTargetPosition).length2();
            continue;
        }

        // Otherwise, try to find another point to move to.
        updateTargetPositionBasedOnCollision(callback.m_hitNormalWorld);

        btVector3 currentDir = mTargetPosition - mCurrentPosition;
        distance2 = currentDir.length2();
        if(!(distance2 < SIMD_EPSILON))
        {
            currentDir.normalize();
            if(currentDir.dot(mNormalizedDirection) <= 0)
                break;
        }
    }
}

void CharacterController::setVelocityForTimeInterval(const btVector3 &velocity, btScalar timeInterval)
{
    mUseWalkDirection = false;
    mWalkDirection = velocity;
    mNormalizedDirection = getNormalizedVector(mWalkDirection);
    mVelocityTimeInterval = timeInterval;
}

void CharacterController::warp(const btVector3 &origin)
{
    btTransform xform(btMatrix3x3::getIdentity(), origin);
    mGhostObject->setWorldTransform(xform);
}

void CharacterController::preStep(btCollisionWorld *collisionWorld)
{
    int numPenetrationLoops = 0;
    while(recoverFromPenetration(collisionWorld))
    {
        numPenetrationLoops++;
        if(numPenetrationLoops > 4)
            break;
    }

    mCurrentPosition = mGhostObject->getWorldTransform().getOrigin();
    mTargetPosition = mCurrentPosition;
}

void CharacterController::playerStep(btCollisionWorld *collisionWorld, btScalar dt)
{
    if(!mUseWalkDirection && mVelocityTimeInterval <= 0)
        return;

    checkOnGround(collisionWorld);
    if(mWasOnGround)
        mVerticalVelocity = 0.0f;

    setRBForceImpulseBasedOnCollision();

    if(mVerticalVelocity > 0 && mVerticalVelocity > mJumpSpeed)
        mVerticalVelocity = mJumpSpeed;
    else if(mVerticalVelocity < 0 && mVerticalVelocity < mFallSpeed)
        mVerticalVelocity = mFallSpeed;

    btTransform xform = mGhostObject->getWorldTransform();
    btVector3 inertia = getUpAxisDirections()[mUpAxis] * mVerticalVelocity;
    if(mUseWalkDirection)
        stepForwardAndStrafe(collisionWorld, (mWalkDirection + inertia)*dt);
    else
    {
        // still have some time left for moving!
        btScalar dtMoving = std::min(dt, mVelocityTimeInterval);
        mVelocityTimeInterval -= dt;

        // how far will we move while we are moving?
        btVector3 move = (mWalkDirection + inertia) * dtMoving;

        // okay, move
        stepForwardAndStrafe(collisionWorld, move);
    }

    if(!(mGravity < 0.0f && mVerticalVelocity <= 0.0f))
        mWasOnGround = false;
    else if(mWasOnGround)
    {
        btTransform start(btMatrix3x3::getIdentity(), mCurrentPosition + getUpAxisDirections()[mUpAxis]*0.5f);
        btTransform end(btMatrix3x3::getIdentity(), mCurrentPosition - getUpAxisDirections()[mUpAxis]*mStepHeight);
        std::pair<btScalar,btVector3> res = sweepTrace(collisionWorld, start, end);
        if(res.first < 1.0f)
            mCurrentPosition = start.getOrigin().lerp(end.getOrigin(), res.first);
    }

    xform.setOrigin(mCurrentPosition);
    mGhostObject->setWorldTransform(xform);

    checkOnGround(collisionWorld, true);
    if(!mWasOnGround)
        mVerticalVelocity += mGravity * dt;
    else
    {
        mVerticalVelocity = 0.0f;
        mWasJumping = false;
    }
}

void CharacterController::setFallSpeed(btScalar fallSpeed)
{
    mFallSpeed = fallSpeed;
}

void CharacterController::setJumpSpeed(btScalar jumpSpeed)
{
    mJumpSpeed = jumpSpeed;
}

bool CharacterController::canJump() const
{
    return onGround();
}

void CharacterController::jump()
{
    if(!canJump())
        return;

    mVerticalVelocity = mJumpSpeed;
    mWasJumping = true;
}

void CharacterController::setGravity(const btScalar gravity)
{
    mGravity = gravity;
}

btScalar CharacterController::getGravity() const
{
    return mGravity;
}

void CharacterController::setMaxSlope(btScalar slopeRadians)
{
    mMaxSlopeRadians = slopeRadians;
    mMaxSlopeCosine = btCos(slopeRadians);
}

btScalar CharacterController::getMaxSlope() const
{
    return mMaxSlopeRadians;
}

bool CharacterController::onGround() const
{
    return mWasOnGround && mVerticalVelocity == 0.0f && mGravity < 0.0f;
}

void CharacterController::setWalkDirection(const btVector3 &walkDirection)
{
    mUseWalkDirection = true;
    mWalkDirection = walkDirection;
    mNormalizedDirection = getNormalizedVector(mWalkDirection);
}

void CharacterController::setWalkDirection(const btScalar x, const btScalar y, const btScalar z)
{
    mUseWalkDirection = true;
    mWalkDirection.setValue(x, y, z);
    mNormalizedDirection = getNormalizedVector(mWalkDirection);
}

btVector3 CharacterController::getWalkDirection() const
{
    return mWalkDirection;
}

btVector3 CharacterController::getPosition() const
{
    return mGhostObject->getWorldTransform().getOrigin();
}

void CharacterController::updateAction(btCollisionWorld *collisionWorld, btScalar dt)
{
    preStep(collisionWorld);
    playerStep(collisionWorld, dt);
}

}
