#ifndef _GAME_RENDER_ANIMATION_H
#define _GAME_RENDER_ANIMATION_H

#include <vector>

#include <components/nifogre/ogre_nif_loader.hpp>
#include <openengine/ogre/renderer.hpp>
#include "../mwworld/actiontalk.hpp"
#include <components/nif/node.hpp>
#include <openengine/bullet/physic.hpp>




namespace MWRender {

class Animation {
    struct Group {
        NifOgre::TextKeyMap::const_iterator mStart;
        NifOgre::TextKeyMap::const_iterator mStop;
        NifOgre::TextKeyMap::const_iterator mLoopStart;
        NifOgre::TextKeyMap::const_iterator mLoopStop;

        size_t mLoops;

        Group() : mLoops(0)
        { }
    };

protected:
    Ogre::SceneNode* mInsert;

    float mTime;
    Group mCurGroup;
    Group mNextGroup;

    bool mSkipFrame;

    NifOgre::EntityList mEntityList;
    NifOgre::TextKeyMap mTextKeys;

    bool findGroupTimes(const std::string &groupname, Group *group);

public:
    Animation();
    virtual ~Animation();

    void playGroup(std::string groupname, int mode, int loops);
    void skipAnim();
    virtual void runAnimation(float timepassed);
};

}
#endif
