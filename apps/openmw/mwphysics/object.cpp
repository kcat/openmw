#include "object.hpp"

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>

#include <OgreSceneNode.h>
#include <OgreMatrix3.h>

namespace MWPhysics
{

Object::Object(const MWWorld::Ptr &ptr, const NifBullet::BulletShapePtr &shape, PhysicsSystem *phys)
  : ObjectInfo(ptr)
  , mPhysics(phys)
  , mShape(shape)
  , mCollisionObject(0)
{
    const Ogre::Quaternion &rot = getPtr().getRefData().getBaseNode()->getOrientation();
    const Ogre::Vector3 &pos = getPtr().getRefData().getBaseNode()->getPosition();

    mCurrentTrans = btTransform(btQuaternion(rot.x, rot.y, rot.z, rot.w),
                                btVector3(pos.x, pos.y, pos.z));

    resetCollisionObject();
}

Object::~Object()
{
    delete mCollisionObject;
}

void Object::resetCollisionObject()
{
    delete mCollisionObject;

    const Ogre::Vector3 &scale = mPtr.getRefData().getBaseNode()->getScale();
    btRigidBody::btRigidBodyConstructionInfo cinf(0.0f, this, mShape->getScaledCollisionShape(scale.x));
    mCollisionObject = new btRigidBody(cinf);

    // Explicitly downcast to ensure the user pointer can be directly casted back to ObjectInfo.
    mCollisionObject->setUserPointer(static_cast<ObjectInfo*>(this));
}


void Object::setOrientation(const Ogre::Quaternion &rot)
{
    mCurrentTrans.setRotation(btQuaternion(rot.x, rot.y, rot.z, rot.w));
    mCollisionObject->setWorldTransform(mCurrentTrans);
}


void Object::setWorldTransform(const btTransform &worldTrans)
{
    mPhysics->queueWorldMovement(mPtr, worldTrans);
}

void Object::getWorldTransform(btTransform &worldTrans) const
{
    worldTrans = mCurrentTrans;
}

}
