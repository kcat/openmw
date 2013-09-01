#include "object.hpp"

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>

namespace MWPhysics
{

Object::Object(const MWWorld::Ptr &ptr, const NifBullet::BulletShapePtr &shape, const btTransform &startTrans)
  : ObjectInfo(ptr)
  , mShape(shape)
  , mMotionState(0)
  , mCollisionObject(0)
{
    mMotionState = new btDefaultMotionState(startTrans);

    btRigidBody::btRigidBodyConstructionInfo cinf(0.0f, mMotionState, shape->getCollisionShape());
    mCollisionObject = new btRigidBody(cinf);

    // Explicitly downcast to ensure the user pointer can be directly casted back to ObjectInfo.
    mCollisionObject->setUserPointer(static_cast<ObjectInfo*>(this));
}

Object::~Object()
{
    delete mCollisionObject;
    delete mMotionState;
}


}
