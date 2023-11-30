#ifndef IMAGES_H_
#define IMAGES_H_

#include "object_identifier/image.h"

namespace object_identifier
{
class Images
{
public:
    Images() :
        id(0) {}

    Images(int _id) :
        id(_id) {}

    void set_equ_image(Image _equ) { equ = _equ; }
    void set_rgb_image(Image _rgb) { rgb = _rgb; }

    Image equ;
    Image rgb;

    int id;

private:
};
} // namespace object_identifier

#endif  // IMAGES_H_