#ifndef MWPHYSICS_ACTOR_HPP
#define MWPHYSICS_ACTOR_HPP

#include <LinearMath/btMotionState.h>

#include <components/nifbullet/bulletshape.hpp>

#include "../mwworld/ptr.hpp"

#include "physicssystem.hpp"
#include "object.hpp"

namespace MWPhysics
{
    class Actor : public ObjectInfo, public btMotionState
    {
        PhysicsSystem *mPhysics;

        NifBullet::BulletShapePtr mShape;
        btTransform mBBoxTransform;
        btCollisionShape *mCollisionShape;
        btCollisionObject *mCollisionObject;

        btTransform mCurrentTrans;

    protected:
        virtual void setWorldTransform(const btTransform &worldTrans);
        virtual void getWorldTransform(btTransform &worldTrans) const;

    public:
        Actor(const MWWorld::Ptr &ptr, const NifBullet::BulletShapePtr &shape, PhysicsSystem *phys);
        virtual ~Actor();

        btCollisionObject *getCollisionObject() const
        { return mCollisionObject; }

        void resetCollisionObject();
    };
}

#endif /* MWPHYSICS_ACTOR_HPP */
