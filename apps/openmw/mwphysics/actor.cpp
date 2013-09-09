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
    // Use a cylinder shape to help avoid actors getting caught by bounding box edges
    mCollisionShape = new btCylinderShapeZ(mShape->getBBoxRadius());

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

    const Ogre::Vector3 &scale = mPtr.getRefData().getBaseNode()->getScale();
    mCollisionShape->setLocalScaling(btVector3(scale.x, scale.x, scale.x));
    mBBoxTransform.setOrigin(mShape->getBBoxTransform().getOrigin() * scale.x);
    mBBoxTransform.setBasis(mShape->getBBoxTransform().getBasis());

    const float *pos = getPtr().getRefData().getPosition().pos;
    btTransform startTrans(btMatrix3x3::getIdentity(),
                           btVector3(pos[0], pos[1], pos[2]) + mBBoxTransform.getOrigin());

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

void Actor::updateVelocity(const Ogre::Vector3 &velocity)
{
    mActionIface->setWalkDirection(velocity.x, velocity.y, velocity.z);
}

}
