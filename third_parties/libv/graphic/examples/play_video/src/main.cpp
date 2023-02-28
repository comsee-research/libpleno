/// \example
/// Play a video.
/// \author Alexis Wilhelm (2013)
/// \see viewers
/// \privatesection

#include <libv/core/image/image.hpp>
#include <libv/graphic/viewer_context.hpp>
#include <libv/io/codecs/video.hpp>

using namespace v;

int main(int ac, char **av)
{
  for(int i = 1; i < ac; ++i)
  {
    Video *video = Video::open(av[i]);

    while(*video)
    {
      // get the next image
      ImageU8cp image;
      *video >> image;

      // show the image
      v::viewer().clear().add_image(0, 0, image).update();

      // slow down
      usleep(10000);
    }

    delete video;
  }
}
