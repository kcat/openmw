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
    Ogre::Matrix3 mat3;
    const ESM::Position &pos = getPtr().getRefData().getPosition();

    // Generate a rotation matrix that rotates first around Z, then Y, then X.
    mat3.FromEulerAnglesXYZ(Ogre::Radian(-pos.rot[0]), Ogre::Radian(-pos.rot[1]), Ogre::Radian(-pos.rot[2]));
    mCurrentTrans = btTransform(btMatrix3x3(mat3[0][0], mat3[0][1], mat3[0][2],
                                            mat3[1][0], mat3[1][1], mat3[1][2],
                                            mat3[2][0], mat3[2][1], mat3[2][2]),
                                btVector3(pos.pos[0], pos.pos[1], pos.pos[2]));

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
