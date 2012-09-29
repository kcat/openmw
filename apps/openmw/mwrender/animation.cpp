#include "animation.hpp"

#include <OgreHardwarePixelBuffer.h>
#include <OgreSkeletonInstance.h>
#include <OgreEntity.h>
#include <OgreBone.h>
#include <OgreSubMesh.h>
#include <OgreSceneManager.h>


namespace MWRender
{

Animation::Animation()
    : mInsert(NULL)
    , mTime(0.0f)
    , mSkipFrame(false)
{
}

Animation::~Animation()
{
    Ogre::SceneManager *sceneMgr = mInsert->getCreator();
    for(size_t i = 0;i < mEntityList.mEntities.size();i++)
        sceneMgr->destroyEntity(mEntityList.mEntities[i]);
    mEntityList.mEntities.clear();
}


struct checklow {
    bool operator()(const char &a, const char &b) const
    {
        return ::tolower(a) == ::tolower(b);
    }
};

bool Animation::findGroupInfo(const std::string &groupname, const std::string &begin, const std::string &beginloop, const std::string &endloop, const std::string &end, Animation::Group *group)
{
    group->mStart = mTextKeys.end();
    group->mLoopStart = mTextKeys.end();
    group->mLoopStop = mTextKeys.end();
    group->mStop = mTextKeys.end();

    const std::string &start = groupname+": "+begin;
    const std::string &startloop = groupname+": "+beginloop;
    const std::string &stoploop = groupname+": "+endloop;
    const std::string &stop = groupname+": "+end;

    NifOgre::TextKeyMap::const_iterator iter;
    for(iter = mTextKeys.begin();iter != mTextKeys.end();iter++)
    {
        std::string::const_iterator strpos = iter->second.begin();
        std::string::const_iterator strend = iter->second.end();
        size_t strlen = strend-strpos;

        if(start.size() <= strlen && std::mismatch(strpos, strend, start.begin(), checklow()).first == strend)
        {
            group->mStart = iter;
            group->mLoopStart = iter;
        }
        else if(stop.size() <= strlen && std::mismatch(strpos, strend, stop.begin(), checklow()).first == strend)
        {
            group->mStop = iter;
            if(group->mLoopStop == mTextKeys.end())
                group->mLoopStop = iter;
        }
        if(startloop.size() <= strlen && std::mismatch(strpos, strend, startloop.begin(), checklow()).first == strend)
        {
            group->mLoopStart = iter;
        }
        else if(stoploop.size() <= strlen && std::mismatch(strpos, strend, stoploop.begin(), checklow()).first == strend)
        {
            group->mLoopStop = iter;
        }
        if(group->mStart != mTextKeys.end() && group->mLoopStart != mTextKeys.end() &&
           group->mLoopStop != mTextKeys.end() && group->mStop != mTextKeys.end())
        {
            group->mNext = group->mStart;
            return true;
        }
    }

    return false;
}


void Animation::processGroup(Group &group, float time)
{
    while(time >= group.mNext->first)
    {
        // TODO: Process group.mNext->second
        group.mNext++;
    }
}


void Animation::playAnim(const std::string &groupname, const std::string &begin, const std::string &end)
{
    Group group;
    group.mLoops = 1;

    if(!findGroupInfo(groupname, begin, begin, end, end, &group))
        throw std::runtime_error("Failed to find animation group "+groupname);

    mCurGroup = group;
    mNextGroup.mLoops = 0;
    mTime = mCurGroup.mStart->first;
}

void Animation::loopAnim(const std::string &groupname,
                const std::string &begin, const std::string &beginloop,
                const std::string &endloop, const std::string &end, int loops)
{
    Group group;
    group.mLoops = loops;

    if(!findGroupInfo(groupname, begin, beginloop, endloop, end, &group))
        throw std::runtime_error("Failed to find animation group "+groupname);

    mCurGroup = group;
    mNextGroup.mLoops = 0;
    mTime = mCurGroup.mStart->first;
}


void Animation::playGroup(std::string groupname, int mode, int loops)
{
    Group group;
    group.mLoops = loops;

    if(groupname == "all")
    {
        group.mStart = group.mLoopStart = group.mLoopStop = group.mStop = mTextKeys.end();

        NifOgre::TextKeyMap::const_iterator iter = mTextKeys.begin();
        if(iter != mTextKeys.end())
        {
            group.mStart = group.mLoopStart = iter;
            iter = mTextKeys.end();
            group.mLoopStop = group.mStop = --iter;
        }
    }
    else if(!findGroupInfo(groupname, "start", "start loop", "stop loop", "stop", &group))
        throw std::runtime_error("Failed to find animation group "+groupname);

    if(mode == 0 && mCurGroup.mLoops > 0)
        mNextGroup = group;
    else
    {
        mCurGroup = group;
        mNextGroup.mLoops = 0;
        mTime = ((mode==2) ? mCurGroup.mLoopStart->first : mCurGroup.mStart->first);
    }
}

void Animation::skipAnim()
{
    mSkipFrame = true;
}

void Animation::runAnimation(float timepassed)
{
    if(mCurGroup.mLoops > 0 && !mSkipFrame)
    {
        mTime += timepassed;
    do_more:
        while(mCurGroup.mLoops > 1 && mTime >= mCurGroup.mLoopStop->first)
        {
            processGroup(mCurGroup, mCurGroup.mLoopStop->first);
            mCurGroup.mNext = mCurGroup.mLoopStart;

            mCurGroup.mLoops--;
            mTime = mTime - mCurGroup.mLoopStop->first + mCurGroup.mLoopStart->first;
        }
        if(mCurGroup.mLoops <= 1 && mTime >= mCurGroup.mStop->first)
        {
            processGroup(mCurGroup, mCurGroup.mStop->first);
            if(mNextGroup.mLoops > 0)
            {
                mTime = mTime - mCurGroup.mStop->first + mNextGroup.mStart->first;
                mCurGroup = mNextGroup;
                mNextGroup.mLoops = 0;
                goto do_more;
            }
            mTime = mCurGroup.mStop->first;
            mCurGroup.mLoops = 0;
        }
        else
            processGroup(mCurGroup, mTime);

        if(mEntityList.mSkelBase)
        {
            Ogre::AnimationStateSet *aset = mEntityList.mSkelBase->getAllAnimationStates();
            Ogre::AnimationStateIterator as = aset->getAnimationStateIterator();
            while(as.hasMoreElements())
            {
                Ogre::AnimationState *state = as.getNext();
                state->setTimePosition(mTime);
            }
        }
    }
    mSkipFrame = false;
}

}
