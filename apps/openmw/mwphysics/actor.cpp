#include "actor.hpp"

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>

#include <OgreSceneNode.h>
#include <OgreMatrix3.h>

namespace MWPhysics
{

Actor::Actor(const MWWorld::Ptr &ptr, const NifBullet::BulletShapePtr &shape, PhysicsSystem *phys)
  : ObjectInfo(ptr)
  , mPhysics(phys)
  , mShape(shape)
  , mCollisionShape(0)
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

    // Use a cylinder shape to help avoid actors getting caught by bounding box edges
    mCollisionShape = new btCylinderShapeZ(mShape->getBBoxRadius());

    const Ogre::Vector3 &scale = mPtr.getRefData().getBaseNode()->getScale();
    mCollisionShape->setLocalScaling(btVector3(scale.x, scale.x, scale.x));
    mBBoxTransform.setOrigin(mShape->getBBoxTransform().getOrigin() * scale.x);
    mBBoxTransform.setBasis(mShape->getBBoxTransform().getBasis());

    btRigidBody::btRigidBodyConstructionInfo cinf(0.0f, this, mCollisionShape);
    mCollisionObject = new btRigidBody(cinf);

    mCollisionObject->setCollisionFlags(mCollisionObject->getCollisionFlags() |
                                        btCollisionObject::CF_KINEMATIC_OBJECT);
    mCollisionObject->setActivationState(DISABLE_DEACTIVATION);

    // Explicitly downcast to ensure the user pointer can be directly casted back to ObjectInfo.
    mCollisionObject->setUserPointer(static_cast<ObjectInfo*>(this));
}

Actor::~Actor()
{
    delete mCollisionObject;
    delete mCollisionShape;
}

void Actor::resetCollisionObject()
{
    delete mCollisionObject;

    const Ogre::Vector3 &scale = mPtr.getRefData().getBaseNode()->getScale();
    mCollisionShape->setLocalScaling(btVector3(scale.x, scale.x, scale.x));
    mBBoxTransform.setOrigin(mShape->getBBoxTransform().getOrigin() * scale.x);

    btRigidBody::btRigidBodyConstructionInfo cinf(0.0f, this, mCollisionShape);
    mCollisionObject = new btRigidBody(cinf);

    mCollisionObject->setCollisionFlags(mCollisionObject->getCollisionFlags() |
                                        btCollisionObject::CF_KINEMATIC_OBJECT);
    mCollisionObject->setActivationState(DISABLE_DEACTIVATION);

    mCollisionObject->setUserPointer(static_cast<ObjectInfo*>(this));
}


void Actor::setWorldTransform(const btTransform &worldTrans)
{
    //mPhysics->queueWorldMovement(mPtr, mBBoxTransform.inverseTimes(worldTrans));
}

void Actor::getWorldTransform(btTransform &worldTrans) const
{
    worldTrans = mBBoxTransform * mCurrentTrans;
}

}
