#ifndef NIFBULLET_NIFBULLETLOADER_HPP
#define NIFBULLET_NIFBULLETLOADER_HPP

#include <string>


namespace NifBullet
{

class BulletShape;

class BulletShapeLoader
{
public:
    void load(const std::string &name, BulletShape *shape);
};

}

#endif /* NIFBULLET_NIFBULLETLOADER_HPP */
