## Packages and external dependencies installation

The following sections give the installation instructions for the required distribution packages and external dependencies for the supported platforms.

## Required distribution packages

### Ubuntu 16.04 LTS

```
sudo apt-get install build-essential liblas-c-dev mesa-common-dev libsdl2-dev libeigen3-dev libgeographic-dev doxygen
```

### MacOS (Experimental)

The dependencies are here installed through the _homebrew_ tool :

```
xcode-select --install
brew install liblas 
brew install mesa 
brew install sdl2 
brew install eigen 
brew install geographiclib
```

If these dependencies are installed as a frameworks, it is necessary to change the line _MAKE_FLNK_ (39) of the _eratosthene-client_ software _Makefile_. Considering _SDL2_ installed as a framework, the correction is :

    -lSDL2 replaced by -framework SDL2

and the same for the other dependencies.

### liberatosthene

Fulfill the requirements following the [instructions](https://github.com/nils-hamel/liberatosthene/blob/v1.3/DEPEND.md).