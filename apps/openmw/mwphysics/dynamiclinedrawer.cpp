#include "dynamiclinedrawer.hpp"

namespace MWPhysics
{

enum {
  POSITION_BINDING,
  COLOUR_BINDING
};

DynamicLineDrawer::DynamicLineDrawer(void)
{
    initialize();
    mDirty = true;
}

DynamicLineDrawer::~DynamicLineDrawer(void)
{
    delete mRenderOp.vertexData;
    delete mRenderOp.indexData;
}

void DynamicLineDrawer::initialize()
{
    mRenderOp.operationType = Ogre::RenderOperation::OT_LINE_LIST;
    mRenderOp.useIndexes = false;
    mRenderOp.vertexData = new Ogre::VertexData;
    
    // Reset buffer capacities
    mVertexBufferCapacity = 0;
    
    // Create vertex declaration
    createVertexDeclaration();
}

Ogre::Real DynamicLineDrawer::getBoundingRadius() const
{
    return Ogre::Math::Sqrt(std::max(mBox.getMaximum().squaredLength(), mBox.getMinimum().squaredLength()));
}

Ogre::Real DynamicLineDrawer::getSquaredViewDepth(const Ogre::Camera* cam) const
{
    Ogre::Vector3 vmin, vmax, vmid, vdist;
    vmin = mBox.getMinimum();
    vmax = mBox.getMaximum();
    vmid = ((vmax-vmin) * 0.5f) + vmin;
    vdist = cam->getDerivedPosition() - vmid;

    return vdist.squaredLength();
}

void DynamicLineDrawer::addPoint(const Ogre::Vector3 &point, const Ogre::ColourValue &colour)
{
    mPoints.push_back(point);
    mColours.push_back(colour);
    mDirty = true;
}

void DynamicLineDrawer::addPoint(Ogre::Real x, Ogre::Real y, Ogre::Real z, Ogre::Real r, Ogre::Real g, Ogre::Real b)
{
    mPoints.push_back(Ogre::Vector3(x, y, z));
    mColours.push_back(Ogre::ColourValue(r, g, b));
    mDirty = true;
}

void DynamicLineDrawer::setPoint(unsigned short index, const Ogre::Vector3 &value, const Ogre::ColourValue &colourValue)
{
    assert(index < mPoints.size() && "Point index is out of bounds!!");

    mPoints[index] = value;
    mColours[index] = colourValue;
    mDirty = true;
}

const Ogre::Vector3& DynamicLineDrawer::getPoint(unsigned short index) const
{
    assert(index < mPoints.size() && "Point index is out of bounds!!");
    return mPoints[index];
}

unsigned short DynamicLineDrawer::getNumPoints() const
{
    return (unsigned short)mPoints.size();
}

void DynamicLineDrawer::clear()
{
    mPoints.clear();
    mColours.clear();
    mDirty = true;
}

void DynamicLineDrawer::update()
{
    if(mDirty)
        fillHardwareBuffers();
}

void DynamicLineDrawer::createVertexDeclaration()
{
    // Points
    Ogre::VertexDeclaration *decl = mRenderOp.vertexData->vertexDeclaration;
    decl->addElement(POSITION_BINDING, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    
    // Colour
    decl->addElement(COLOUR_BINDING, 0, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
}

void DynamicLineDrawer::prepareHardwareBuffers(size_t vertexCount)
{
    // Prepare vertex buffer
    size_t newVertCapacity = mVertexBufferCapacity;
    if((vertexCount > mVertexBufferCapacity) || !mVertexBufferCapacity)
    {
        // vertexCount exceeds current capacity!
        // It is necessary to reallocate the buffer.

        // Check if this is the first call
        if(!newVertCapacity)
            newVertCapacity = 1;

        // Make capacity the next power of two
        while(newVertCapacity < vertexCount)
            newVertCapacity <<= 1;
    }
    else if(vertexCount < (mVertexBufferCapacity>>1))
    {
        // Make capacity the previous power of two
        while(vertexCount < (newVertCapacity>>1))
            newVertCapacity >>= 1;
    }

    if(newVertCapacity != mVertexBufferCapacity)
    {
        mVertexBufferCapacity = newVertCapacity;

        // Create new vertex buffer
        Ogre::HardwareVertexBufferSharedPtr vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
            mRenderOp.vertexData->vertexDeclaration->getVertexSize(0),
            mVertexBufferCapacity,
            Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY
        );

        // Bind buffer
        mRenderOp.vertexData->vertexBufferBinding->setBinding(0, vbuf);

        // Create new colour buffer
        size_t offset = Ogre::VertexElement::getTypeSize(Ogre::VET_COLOUR);
        vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
            offset,
            mVertexBufferCapacity,
            Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY
        );

        // Bind buffer
        mRenderOp.vertexData->vertexBufferBinding->setBinding(1, vbuf);
    }
    // Update vertex count in the render operation
    mRenderOp.vertexData->vertexCount = vertexCount;
}

void DynamicLineDrawer::fillHardwareBuffers()
{
    size_t size = mPoints.size();

    prepareHardwareBuffers(size);
    if(!size)
    { 
        mBox.setExtents(Ogre::Vector3::ZERO, Ogre::Vector3::ZERO);
        mDirty = false;
        return;
    }

    Ogre::Vector3 AABMin = mPoints[0];
    Ogre::Vector3 AABMax = mPoints[0];

    Ogre::HardwareVertexBufferSharedPtr vbuf =
        mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);

    Ogre::HardwareVertexBufferSharedPtr cbuf = 
        mRenderOp.vertexData->vertexBufferBinding->getBuffer(1);

    // get rendersystem to pack colours
    Ogre::RenderSystem* RS = Ogre::Root::getSingleton().getRenderSystem();

    Ogre::Real* VPrPos = static_cast<Ogre::Real*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
    Ogre::RGBA* CPrPos = static_cast<Ogre::RGBA*>(cbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
    for(size_t i = 0;i < size;i++)
    {
        AABMin.x = std::min(mPoints[i].x, AABMin.x);
        AABMin.y = std::min(mPoints[i].y, AABMin.y);
        AABMin.z = std::min(mPoints[i].z, AABMin.z);

        AABMax.x = std::max(mPoints[i].x, AABMax.x);
        AABMax.y = std::max(mPoints[i].y, AABMax.y);
        AABMax.z = std::max(mPoints[i].z, AABMax.z);

        *VPrPos++ = mPoints[i].x;
        *VPrPos++ = mPoints[i].y;
        *VPrPos++ = mPoints[i].z;

        Ogre::RGBA color;
        RS->convertColourValue(mColours[i], &color);
        *CPrPos++ = color;
    }
    vbuf->unlock();
    cbuf->unlock();

    mBox.setExtents(AABMin, AABMax);

    mDirty = false;
}

}
