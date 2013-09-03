#include "object.hpp"

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <OgreSceneNode.h>

namespace MWPhysics
{

Object::Object(const MWWorld::Ptr &ptr, const NifBullet::BulletShapePtr &shape, const btTransform &startTrans)
  : ObjectInfo(ptr)
  , mShape(shape)
  , mMotionState(0)
  , mCollisionObject(0)
{
    mMotionState = new btDefaultMotionState(startTrans);

    const Ogre::Vector3 &scale = mPtr.getRefData().getBaseNode()->getScale();
    btRigidBody::btRigidBodyConstructionInfo cinf(0.0f, mMotionState, shape->getScaledCollisionShape(scale.x));
    mCollisionObject = new btRigidBody(cinf);

    // Explicitly downcast to ensure the user pointer can be directly casted back to ObjectInfo.
    mCollisionObject->setUserPointer(static_cast<ObjectInfo*>(this));
}

Object::~Object()
{
    delete mCollisionObject;
    delete mMotionState;
}

void Object::resetCollisionObject()
{
    delete mCollisionObject;

    const Ogre::Vector3 &scale = mPtr.getRefData().getBaseNode()->getScale();
    btRigidBody::btRigidBodyConstructionInfo cinf(0.0f, mMotionState, mShape->getScaledCollisionShape(scale.x));
    mCollisionObject = new btRigidBody(cinf);

    mCollisionObject->setUserPointer(static_cast<ObjectInfo*>(this));
}

}
