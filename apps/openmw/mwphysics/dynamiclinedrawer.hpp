#ifndef DUALITY_DYNAMIC_LINE_DRAWER
#define DUALITY_DYNAMIC_LINE_DRAWER

#include <Ogre.h>

namespace MWPhysics
{

/** Class that manages hardware buffers for btDebugDraw.
 *  Most of it taken from: http://www.ogre3d.org/wiki/index.php/DynamicGrowingBuffers
 */
class DynamicLineDrawer : public Ogre::SimpleRenderable
{
public:
    DynamicLineDrawer(void);
    ~DynamicLineDrawer(void);

    /** Initializes the dynamic renderable.
     *  @remarks This function should only be called once. It initializes the
     *           render operation, and calls the abstract function
     *           createVertexDeclaration().
     */
    void initialize();

    /// Implementation of Ogre::SimpleRenderable
    virtual Ogre::Real getBoundingRadius(void) const;
    /// Implementation of Ogre::SimpleRenderable
    virtual Ogre::Real getSquaredViewDepth(const Ogre::Camera* Cam) const;

    /// Add a point to the point list
    void addPoint(const Ogre::Vector3 &Point, const Ogre::ColourValue &Colour);
    /// Add a point to the point list
    void addPoint(Ogre::Real X, Ogre::Real Y, Ogre::Real Z, Ogre::Real R, Ogre::Real G, Ogre::Real B);

    /// Change the location of an existing point in the point list
    void setPoint(unsigned short Index, const Ogre::Vector3 &Value, const Ogre::ColourValue &ColourValue);

    /// Return the location of an existing point in the point list
    const Ogre::Vector3& getPoint(unsigned short Index) const;

    /// Return the total number of points in the point list
    unsigned short getNumPoints() const;

    /// Remove all points from the point list
    void clear();

    /// Call this to update the hardware buffer after making changes.  
    void update();

protected:
    /// Maximum capacity of the currently allocated vertex buffer.
    size_t mVertexBufferCapacity;
    
    virtual void createVertexDeclaration();

    /** Prepares the hardware buffers for the requested vertex and index counts.
     *  @remarks This function must be called before locking the buffers in
     *           fillHardwareBuffers(). It guarantees that the hardware buffers
     *           are large enough to hold at least the requested number of
     *           vertices. The buffers are possibly reallocated to achieve this.
     *  @param vertexCount The number of vertices the buffer must hold.
     */
    void prepareHardwareBuffers(size_t vertexCount);

    /** Fills the hardware vertex and index buffers with data.
     *  @remarks This function must call prepareHardwareBuffers() before locking
     *           the buffers to ensure the they are large enough for the data to
     *           be written. Afterwards the vertex buffer can be locked, and data 
     *           can be written to it. 
     */
    virtual void fillHardwareBuffers();

private:
    std::vector<Ogre::Vector3> mPoints;
    std::vector<Ogre::ColourValue> mColours;
    bool mDirty;
};

}

#endif
