#include "object.hpp"

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>

namespace MWPhysics
{

Object::Object(const NifBullet::BulletShapePtr &shape, const btTransform &startTrans)
  : mShape(shape)
  , mMotionState(0)
  , mCollisionObject(0)
{
    mMotionState = new btDefaultMotionState(startTrans);

    btRigidBody::btRigidBodyConstructionInfo cinf(0.0f, mMotionState, shape->getCollisionShape());
    mCollisionObject = new btRigidBody(cinf);
}

Object::~Object()
{
    delete mCollisionObject;
    delete mMotionState;
}


}
