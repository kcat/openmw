#ifndef MWPHYSICS_OBJECT_HPP
#define MWPHYSICS_OBJECT_HPP

#include <LinearMath/btMotionState.h>

#include <components/nifbullet/bulletshape.hpp>

#include "../mwworld/ptr.hpp"

#include "physicssystem.hpp"

class btCollisionObject;
class btTransform;

namespace MWPhysics
{
    // FIXME: Should be neutral. For actors and objects.
    class ObjectInfo
    {
    protected:
        MWWorld::Ptr mPtr;

    public:
        ObjectInfo(const MWWorld::Ptr &ptr) : mPtr(ptr)
        { }
        virtual ~ObjectInfo() { }

        const MWWorld::Ptr& getPtr() const
        { return mPtr; }
    };

    class Object : public ObjectInfo, public btMotionState
    {
        PhysicsSystem *mPhysics;

        NifBullet::BulletShapePtr mShape;
        btCollisionObject *mCollisionObject;

        btTransform mCurrentTrans;

    protected:
        virtual void setWorldTransform(const btTransform &worldTrans);
        virtual void getWorldTransform(btTransform &worldTrans) const;

    public:
        Object(const MWWorld::Ptr &ptr, const NifBullet::BulletShapePtr &shape, PhysicsSystem *phys);
        virtual ~Object();

        btCollisionObject *getCollisionObject() const
        { return mCollisionObject; }

        void resetCollisionObject();
    };
}

#endif /* MWPHYSICS_OBJECT_HPP */
