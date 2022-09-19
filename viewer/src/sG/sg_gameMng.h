#pragma once

#include "utils/sas_stringid.h"
#include "utils/sas_singleton.h"

#include "threading/sas_thread.h"
#include "threading/sas_semaphore.h"
#include "threading/sas_atomic.h"


class sasCamera;
class sgInputMng;
class sgGameObjectMng;
class sgCameraMng;
class seEditor;
class sgPlayfieldMng;
class saAudioMng;
class sgBoidMng;

namespace sg
{
  namespace gamecode
  {
    class Player;
    class World;
  }
}

class sgGameMng : public sasSingleton< sgGameMng >
{
  friend uint32_t sasStdCall gameCodeThreadFn( void* args );

public:
  sgGameMng();
  ~sgGameMng();

  void initialize( HWND hwnd, const sasConfig& config );
  void destroy();

  void loadPf( sasStringId pfId )                   {}
  void loadScene( sasStringId pfId );

  void step();

  void loadSingleRsc( const std::string& path )     {}

  const char* defaultRscPath() const                { return _rscPath.c_str(); }

  sgInputMng* inputMng() const                      { return _inputMng; }

  void stepGamecodeThread( float dt );

private:
  void initializeSplash();
  void destroySplash();
  void stepCamera( float dt );

private:
  sgCameraMng*          _camMng;
  sgInputMng*           _inputMng;
  sasCamera*            _camera;

  std::string           _rscPath;

  sasStringId           _loadingSplashScreen;

  struct GameCodeThreadData
  {
    sasSemaphore  _syncWithEngineDone;
    sasSemaphore  _gamecodeDone;
    sasAtomic     _threadAlive;
    sgGameMng*    _gameMng;
  };
  GameCodeThreadData  _threadData;
  sasThread           _gamecodeThread;
  


  sgBoidMng* _boidMng;
};
