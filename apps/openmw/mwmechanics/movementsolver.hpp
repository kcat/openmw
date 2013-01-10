#ifndef  GAME_MWMECHANICS_MOVEMENTSOLVER_H
#define  GAME_MWMECHANICS_MOVEMENTSOLVER_H

#include "../openmw/libs/openengine/bullet/trace.h"
#include "../openmw/libs/openengine/bullet/physic.hpp"

#include "../openmw/apps/openmw/mwworld/ptr.hpp"

namespace MWMechanics
{
    class MovementSolver
    {
        public:
            MovementSolver(OEngine::Physic::PhysicEngine* engine, const MWWorld::Ptr &ptr);
            virtual ~MovementSolver();
            
            Ogre::Vector3 move(Ogre::Vector3 oldPosition, Ogre::Vector3 attemptedPosition, float time, Ogre::Vector3 halfExtents);
        protected:
        private:
            bool stepMove(Ogre::Vector3& position, Ogre::Vector3 velocity, float remainingTime, float verticalRotation, Ogre::Vector3 halfExtents, bool isInterior);
            
            void clipVelocity(const Ogre::Vector3& in, const Ogre::Vector3& normal, Ogre::Vector3& out, const float overbounce);

            OEngine::Physic::PhysicEngine* mEngine;
            MWWorld::Ptr mPtr;
            OEngine::Physic::PhysicActor* mPhysicActor;

            float verticalVelocity;
            
    };
}

#endif //  GAME_MWMECHANICS_MOVEMENTSOLVER_H
