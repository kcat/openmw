#include "actor.hpp"

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

#include <OgreSceneNode.h>
#include <OgreMatrix3.h>

#include "charactercontroller.hpp"

namespace MWPhysics
{

Actor::Actor(const MWWorld::Ptr &ptr, const NifBullet::BulletShapePtr &shape, PhysicsSystem *phys)
  : ObjectInfo(ptr)
  , mPhysics(phys)
  , mShape(shape)
  , mCollisionShape(0)
  , mCollisionObject(0)
  , mActionIface(0)
{
    resetCollisionObject();
}

Actor::~Actor()
{
    delete mActionIface;
    delete mCollisionObject;
    delete mCollisionShape;
}

void Actor::resetCollisionObject()
{
    delete mActionIface;
    delete mCollisionObject;
    delete mCollisionShape;

    const Ogre::Vector3 &scale = mPtr.getRefData().getBaseNode()->getScale();
    // Use a capsule shape to help avoid actors getting caught on edges
    float minradius = std::min(mShape->getBBoxRadius().x(), mShape->getBBoxRadius().y());
    float maxradius = std::max(mShape->getBBoxRadius().x(), mShape->getBBoxRadius().y());
    float height = std::max(0.0f, mShape->getBBoxRadius().z()*2.0f - minradius*2.0f);
    mCollisionShape = new btCapsuleShapeZ(minradius, height);
    mCollisionShape->setLocalScaling(btVector3(scale.x,
                                               scale.y * maxradius/minradius,
                                               scale.z));
    mBBoxTransform.setOrigin(mShape->getBBoxTransform().getOrigin() * btVector3(scale.x, scale.y, scale.z));
    mBBoxTransform.setBasis(mShape->getBBoxTransform().getBasis());

    const Ogre::Vector3 &pos = getPtr().getRefData().getBaseNode()->getPosition();
    btTransform startTrans(btMatrix3x3::getIdentity(),
                           btVector3(pos.x, pos.y, pos.z) + mBBoxTransform.getOrigin());

    mCollisionObject = new btPairCachingGhostObject();
    mCollisionObject->setCollisionShape(mCollisionShape);
    mCollisionObject->setWorldTransform(startTrans);

    mCollisionObject->setCollisionFlags(mCollisionObject->getCollisionFlags() |
                                        btCollisionObject::CF_CHARACTER_OBJECT);
    mCollisionObject->setActivationState(DISABLE_DEACTIVATION);

    // Explicitly downcast to ensure the user pointer can be directly casted back to ObjectInfo.
    mCollisionObject->setUserPointer(static_cast<ObjectInfo*>(this));

    mActionIface = new CharacterController(mCollisionObject, mCollisionShape, 32.0f, 2);
}

btVector3 Actor::getOrigin() const
{
    return mActionIface->getPosition() - mBBoxTransform.getOrigin();
}

void Actor::setOrigin(const btVector3 &newOrigin)
{
    mActionIface->warp(newOrigin + mBBoxTransform.getOrigin());
}

void Actor::jump(float vel)
{
    mActionIface->setJumpSpeed(vel);
    mActionIface->jump();
}

void Actor::updateVelocity(const Ogre::Vector3 &velocity)
{
    mActionIface->setWalkDirection(velocity.x, velocity.y, velocity.z);
}

}
