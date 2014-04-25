#ifndef MWPHYSICS_DEBUGDRAW_HPP
#define MWPHYSICS_DEBUGDRAW_HPP

#include <btBulletDynamicsCommon.h>

#include "dynamiclinedrawer.hpp"

namespace MWPhysics
{

/** Implements the bullet debug draw interface for drawing with Ogre. */
class DebugDraw : public btIDebugDraw
{
public:
    DebugDraw(Ogre::SceneManager* Scene, btDynamicsWorld *World);
    ~DebugDraw();

    /** Moves the data from the world to the ogre object.
     *  Should be called once every frame.
     */
    void update();

    void setDebugMode(int mode);
    int getDebugMode() const;

protected:
    // these implement the debug/drawing interface, no need to call them from the outside
    void drawLine(const btVector3& from, const btVector3& to, const btVector3& color);
    void drawContactPoint(const btVector3& pointOnB, const btVector3& normalOnB,
                          btScalar distance, int lifeTime, const btVector3& color);
    void reportErrorWarning(const char *warningString);
    void draw3dText(const btVector3& location, const char *textString);

private:
    std::vector<Ogre::Vector2> mUnitCircle;

    DynamicLineDrawer *mDebugLineDrawer;
    Ogre::SceneNode   *mDebugDrawerNode;
    btDynamicsWorld   *mWorld;
    bool              mIsActive;
};

}

#endif
