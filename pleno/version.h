//! version of the Pleno library
#define PLENO_VERSION_MAJOR 0
#define PLENO_VERSION_MINOR 1
#define PLENO_VERSION_PATCH 0

//! version of the Pleno library as string
#define PLENO_VERSION "0.1.0"

//! version of the Pleno library as an int, i.e., 100 * (100 * version_major() + version_minor()) + version_patch();
#define PLENO_VERSION_INT (100 * (100 * PLENO_VERSION_MAJOR + PLENO_VERSION_MINOR) + PLENO_VERSION_PATCH)
