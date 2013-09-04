#ifndef MWPHYSICS_ACTOR_HPP
#define MWPHYSICS_ACTOR_HPP

#include <components/nifbullet/bulletshape.hpp>

#include "../mwworld/ptr.hpp"

#include "physicssystem.hpp"
#include "object.hpp"

class btCylinderShape;
class btActionInterface;


namespace MWPhysics
{
    class CharacterAction;

    class Actor : public ObjectInfo
    {
        PhysicsSystem *mPhysics;

        NifBullet::BulletShapePtr mShape;
        btTransform mBBoxTransform;
        btCylinderShape *mCollisionShape;
        btCollisionObject *mCollisionObject;

        CharacterAction *mActionIface;

    public:
        Actor(const MWWorld::Ptr &ptr, const NifBullet::BulletShapePtr &shape, PhysicsSystem *phys);
        virtual ~Actor();

        btCollisionObject *getCollisionObject() const
        { return mCollisionObject; }
        btActionInterface *getActionInterface() const;

        void resetCollisionObject();

        /// Gets the collision object's current position, corrected for the bounding box offset
        btVector3 getOrigin() const;

        /// Sets the collision object's current position, correcting for the bounding box offset
        void setOrigin(const btVector3 &newOrigin);

        void updateVelocity(const Ogre::Vector3 &velocity);
    };
}

#endif /* MWPHYSICS_ACTOR_HPP */
