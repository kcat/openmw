#ifndef MWPHYSICS_OBJECT_HPP
#define MWPHYSICS_OBJECT_HPP

#include <components/nifbullet/bulletshape.hpp>

class btCollisionObject;
class btMotionState;
class btTransform;

namespace MWPhysics
{
    class Object
    {
        NifBullet::BulletShapePtr mShape;

        btMotionState *mMotionState;
        btCollisionObject *mCollisionObject;

    public:
        Object(const NifBullet::BulletShapePtr &shape, const btTransform &startTrans);
        virtual ~Object();

        btCollisionObject *getCollisionObject() const
        { return mCollisionObject; }
    };
}

#endif /* MWPHYSICS_OBJECT_HPP */
