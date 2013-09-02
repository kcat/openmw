#ifndef MWPHYSICS_OBJECT_HPP
#define MWPHYSICS_OBJECT_HPP

#include <components/nifbullet/bulletshape.hpp>

#include "../mwworld/ptr.hpp"

#include "physicssystem.hpp"

class btCollisionObject;
class btMotionState;
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

    class Object : public ObjectInfo
    {
        NifBullet::BulletShapePtr mShape;

        btMotionState *mMotionState;
        btCollisionObject *mCollisionObject;

    public:
        Object(const MWWorld::Ptr &ptr, const NifBullet::BulletShapePtr &shape, const btTransform &startTrans);
        virtual ~Object();

        btCollisionObject *getCollisionObject() const
        { return mCollisionObject; }
    };
}

#endif /* MWPHYSICS_OBJECT_HPP */
