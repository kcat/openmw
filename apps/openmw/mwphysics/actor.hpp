#ifndef MWPHYSICS_ACTOR_HPP
#define MWPHYSICS_ACTOR_HPP

#include <components/nifbullet/bulletshape.hpp>

#include "../mwworld/ptr.hpp"

#include "physicssystem.hpp"
#include "object.hpp"

class btActionInterface;


namespace MWPhysics
{
    class CharacterAction;

    class Actor : public ObjectInfo
    {
        PhysicsSystem *mPhysics;

        NifBullet::BulletShapePtr mShape;
        btTransform mBBoxTransform;
        btCollisionShape *mCollisionShape;
        btCollisionObject *mCollisionObject;

        CharacterAction *mActionIface;

        btTransform mCurrentTrans;

    protected:

    public:
        Actor(const MWWorld::Ptr &ptr, const NifBullet::BulletShapePtr &shape, PhysicsSystem *phys);
        virtual ~Actor();

        btCollisionObject *getCollisionObject() const
        { return mCollisionObject; }
        btActionInterface *getActionInterface() const;

        void resetCollisionObject();

        const btTransform &getTransform() const
        { return mCurrentTrans; }
        void setTransform(const btTransform &newTrans)
        { mCurrentTrans = newTrans; }

        const btTransform& getBBoxTransform() const
        { return mBBoxTransform; }

        void updateVelocity(const Ogre::Vector3 &velocity);
    };
}

#endif /* MWPHYSICS_ACTOR_HPP */
