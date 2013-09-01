#include "debugdraw.hpp"

#ifndef M_PI
#define M_PI    3.14159265358979323846f
#endif

namespace MWPhysics
{

DebugDraw::DebugDraw(Ogre::SceneManager *scene, btDynamicsWorld *world)
  : mWorld(world), mIsActive(false)
{
    // generate a unit sphere
    static const std::size_t sCircleSubdivs = 12;
    mUnitCircle.reserve(sCircleSubdivs+1);

    for(std::size_t i = 0;i < sCircleSubdivs;++i)
    {
        float angle = (M_PI*2.f*i) / sCircleSubdivs;
        mUnitCircle.push_back(Ogre::Vector2(std::cos(angle), std::sin(angle)));
    }
    mUnitCircle.push_back(mUnitCircle.front());

    // setup rendering properties
    mDebugLineDrawer = new DynamicLineDrawer();
    if(!scene->hasSceneNode("DebugDrawer"))
        mDebugDrawerNode = scene->getRootSceneNode()->createChildSceneNode("DebugDrawer");
    else
    {
        mDebugDrawerNode = scene->getSceneNode("DebugDrawer");
        Ogre::SceneNode::ObjectIterator iter = mDebugDrawerNode->getAttachedObjectIterator();
        while(iter.hasMoreElements())
            scene->destroyEntity(dynamic_cast<Ogre::Entity*>(iter.getNext()));
    }
    mDebugDrawerNode->attachObject(mDebugLineDrawer);

    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName("DebugLines");
    if(material.isNull())
    {
        material = Ogre::MaterialManager::getSingleton().create("DebugLines", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->setReceiveShadows(false);
        material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        material->setDepthBias(0.1f, 0.0f);

        Ogre::Technique *tech = material->getTechnique(0);
        tech->setLightingEnabled(false);
        Ogre::Pass *pass = tech->getPass(0);
        pass->setVertexColourTracking(Ogre::TVC_AMBIENT | Ogre::TVC_DIFFUSE | Ogre::TVC_EMISSIVE);
    }
    mDebugLineDrawer->setMaterial(material->getName());

    mWorld->setDebugDrawer(this);
}

DebugDraw::~DebugDraw()
{
    mWorld->setDebugDrawer(NULL);
    delete mDebugLineDrawer;
}

void DebugDraw::update()
{
    if(mIsActive)
    {
        mWorld->debugDrawWorld();
        mDebugLineDrawer->update();
        mDebugDrawerNode->needUpdate();
        mDebugLineDrawer->clear();
    }
    else
    {
        mDebugLineDrawer->clear();
        mDebugLineDrawer->update();
        mDebugDrawerNode->needUpdate();
    }
}

void DebugDraw::setDebugMode(int mode)
{
    mIsActive = mode!=0;
    if(!mIsActive)
        mDebugLineDrawer->clear();
}

int DebugDraw::getDebugMode() const
{
    return mIsActive ? 1 : 0;
}

void DebugDraw::drawLine(const btVector3& from, const btVector3& to,
                         const btVector3& color )
{
    mDebugLineDrawer->addPoint(Ogre::Vector3(from.getX(), from.getY(), from.getZ()),
                               Ogre::ColourValue(color.getX(), color.getY(), color.getZ()));
    mDebugLineDrawer->addPoint(Ogre::Vector3(to.getX(), to.getY(), to.getZ()),
                               Ogre::ColourValue(color.getX(), color.getY(), color.getZ()));
}

void DebugDraw::drawContactPoint(const btVector3&, const btVector3&,
                                 btScalar, int, const btVector3&)
{
}

void DebugDraw::reportErrorWarning(const char* WarningString)
{
    std::cout << WarningString << std::flush;
}

void DebugDraw::draw3dText(const btVector3&, const char*)
{
}

}
