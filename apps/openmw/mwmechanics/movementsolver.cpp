#include "../openmw/apps/openmw/mwmechanics/movementsolver.hpp"

namespace MWMechanics
{

    MovementSolver::MovementSolver(OEngine::Physic::PhysicEngine* engine, const MWWorld::Ptr& ptr)
        : mEngine(engine)
        , mPtr(ptr)
    {
        mPhysicActor=mEngine->getCharacter(mPtr.getRefData().getHandle());
    }

    MovementSolver::~MovementSolver()
    {
        //nothing to do. we don't use new.
    }

    void MovementSolver::clipVelocity(const Ogre::Vector3& in, const Ogre::Vector3& normal, Ogre::Vector3& out, const float overbounce)
    {
        //Math stuff. Basically just project the velocity vector onto the plane represented by the normal.
        //More specifically, it projects velocity onto the normal, takes that result, multiplies it by overbounce and then subtracts it from velocity.
        float backoff;
        
        backoff = in.dotProduct(normal);
        
        if ( backoff < 0 )
            backoff *= overbounce;
        else
            backoff /= overbounce;

        float changex = normal.x * backoff;
        out.x = in.x - changex;
        float changey = normal.y * backoff;
        out.y = in.y - changey;
        float changez = normal.z * backoff;
        out.z = in.z - changez;
    }

    bool MovementSolver::stepMove(Ogre::Vector3& position, Ogre::Vector3 velocity, float remainingTime, float verticalRotation, Ogre::Vector3 halfExtents, bool isInterior)
    {
        traceResults trace; //no initialization needed
    
        newtrace(&trace, position+Ogre::Vector3(0,0,STEPSIZE), position+Ogre::Vector3(0,0,STEPSIZE)+velocity*remainingTime, halfExtents, verticalRotation, isInterior, mEngine);
        if (trace.fraction==0)
        {
            return false;
        }
        newtrace(&trace, trace.endpos, trace.endpos-Ogre::Vector3(0,0,STEPSIZE), halfExtents, verticalRotation, isInterior, mEngine);
        if (trace.planenormal.dotProduct(Ogre::Vector3(0,0,1))>.50)
        {
            position=trace.endpos; //only step down onto semi-horizontal surfaces. don't step down onto the side of a house or a wall.
            return true;
        }
        else
        {
            return false;
        }
    }


    Ogre::Vector3 MovementSolver::move(Ogre::Vector3 oldPosition, Ogre::Vector3 attemptedPosition, float time, Ogre::Vector3 halfExtents)
    {
        traceResults trace; //no initialization needed
        int iterations=0, maxIterations=50; //arbitrary number. To prevent infinite loops. They shouldn't happen but it's good to be prepared.

        Ogre::Vector3 horizontalVelocity=(attemptedPosition-oldPosition)/time;
        Ogre::Vector3 velocity(horizontalVelocity.x, horizontalVelocity.y, verticalVelocity); // we need a copy of the velocity before we start clipping it for steps
        Ogre::Vector3 clippedVelocity(horizontalVelocity.x, horizontalVelocity.y, verticalVelocity);
        
        float remainingTime=time;
        bool collisions=mPhysicActor->getCollisionMode();
        bool isInterior=!mPtr.getCell()->isExterior();
        float verticalRotation=mPhysicActor->getRotation().getYaw().valueDegrees();
        
        Ogre::Vector3 lastNormal(0,0,0);
        Ogre::Vector3 up(0, 0, 1);
        Ogre::Vector3 newPosition=oldPosition;

        if (!collisions)
        {
            return attemptedPosition;
        }

        clippedVelocity.z-=time*400;
        
        newtrace(&trace, newPosition, newPosition+Ogre::Vector3(0,0,-10), halfExtents, verticalRotation, isInterior, mEngine);
        if (trace.planenormal.dotProduct(up)>.50 && trace.fraction!=1)
        {
            clipVelocity(clippedVelocity, trace.planenormal, clippedVelocity, OVERCLIP);
            if (clippedVelocity.length()<10)
            {
                return oldPosition;
            }
        }

        verticalVelocity=clippedVelocity.z;

        do
        {
            Ogre::Vector3 end=newPosition+clippedVelocity*remainingTime; //how far would you go if there was nothing in the way?
            newtrace(&trace, newPosition, end, halfExtents, verticalRotation, isInterior, mEngine);
            newPosition=trace.endpos;
            remainingTime=remainingTime*(1-trace.fraction);

            //nothing else to do unless trace hit something
            if (trace.fraction!=1)
            {
                //since we don't have a better indicator of whether the polygon hit was the same as the last one, use normal
                if (lastNormal.directionEquals(trace.planenormal, Ogre::Radian(0.005)) || (up.dotProduct(trace.planenormal)<.50 && up.dotProduct(trace.planenormal)>0))
                {
                    //step up!
                    if (stepMove(newPosition, velocity, remainingTime, verticalRotation, halfExtents, isInterior))
                    {
                        std::cout<<"step\n";
                        remainingTime=0;
                        break;
                    }
                    else
                    {
                        //step failed, so clip the velocity to slide along vertical surfaces instead of sticking into them
                        //clipVelocity(clippedVelocity, trace.planenormal, clippedVelocity, OVERCLIP);
                    }
                }
                else
                {
                    //clip velocity for the next iteration
                    clipVelocity(clippedVelocity, trace.planenormal, clippedVelocity, OVERCLIP);
                }
                lastNormal=trace.planenormal;
            }
            iterations++;
        } while (iterations<=maxIterations && remainingTime>0); //limit iterations, wait until move is complete
        
        return newPosition;
    }
}
