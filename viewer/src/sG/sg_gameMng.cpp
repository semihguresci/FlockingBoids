#include "sg_pch.h"
#include "sg_gameMng.h"
#include "input/sg_inputmng.h"
#include "utils/sas_path.h"
#include "utils/sas_timer.h"
#include "sg_cameraMng.h"
#include "utils/sas_camera.h"

#include "sg_boidMng.h"
#include "sg_boid.h"


#define NUM_SMALL_BOIDS 900
#define NUM_BIG_BOIDS 4


sgGameMng::sgGameMng()
{
  _inputMng = nullptr;
  _camera = nullptr;
  _boidMng = nullptr;

  _threadData._threadAlive = 1;  
}

sgGameMng::~sgGameMng()
{
}


uint32_t sasStdCall gameCodeThreadFn( void* args )
{
  sgGameMng::GameCodeThreadData* gctd = reinterpret_cast< sgGameMng::GameCodeThreadData* >( args );

  while( gctd->_threadAlive )
  {
    sasWaitForSemaphore( gctd->_syncWithEngineDone );
    
    float dt = sasTimer::singletonPtr()->getElapsedTime();
    gctd->_gameMng->stepGamecodeThread( dt );

    sasSignalSemaphore( gctd->_gamecodeDone );
  }

  return 0;
}

void sgGameMng::initialize( HWND hwnd, const sasConfig& config )
{
  _rscPath = config.resourcePath;

  bool r = sasSystemsInitialize();
  sasASSERT( r );

  _inputMng = sasNew sgInputMng();
  sasASSERT( _inputMng != NULL );

  sasConfig renderSetupConfig = config;
  renderSetupConfig.vrHmd = _inputMng->ovrMng()->ovrHmdPtr();
  r = sasInitialize( renderSetupConfig );
  sasASSERT( r );

  _camMng = sasNew sgCameraMng();
  sasASSERT( _camMng != NULL );
    
  initializeSplash();

  _camera = sasCreateCamera();
  _camMng->addCamera( _camera );

  _boidMng = sasNew sgBoidMng( NUM_SMALL_BOIDS, NUM_BIG_BOIDS );

  //create gamecode thread
  _threadData._gameMng = this;
  _gamecodeThread.start( gameCodeThreadFn, &_threadData );
  sasSignalSemaphore( _threadData._gamecodeDone );
}


void sgGameMng::initializeSplash()
{
  _loadingSplashScreen = sasStringId::build( "sasEngineLogo" );
  std::string texturePath = _rscPath;
  texturePath += "ui/sas_loading_black.tga";
  sasCreateSplashScreen( _loadingSplashScreen, texturePath.c_str(), smVec2( 0.f, 0.f ), smVec2( 0.75f, 0.75f ) );
}

void sgGameMng::loadScene( sasStringId pfId )
{
  sasSetSplashScreen( _loadingSplashScreen );

  //flush a frame to display splash screen before stalling for load. Should be fixed by threading off loading
  sasSystemsUpdate();
  _inputMng->step();
  sasUpdate(); 

  //temp camera pos loading
  sasCameraSetPosition( smVec4(0.f, 30.f, 0.f, 0.0f), _camera );
  sasCameraLookAt( smVec4(0,-1,0,1), smVec4(0,0,1,0), _camera );

  //disable loading splash screen
  sasUnbindSplashScreen();
}

void sgGameMng::destroy()
{
  sasAtomicExchange( &( _threadData._threadAlive ), 0 );
  _gamecodeThread.kill();

  sasDelete _boidMng;

  if( _camera )
  {
    _camMng->removeCamera( _camera );
    sasDestroyCamera( _camera );
  }

  sasUnloadAllResources();

  sasDelete _camMng;

  sasDelete _inputMng;

  sasShutdown();
  sasSystemsShutdown();
}


void sgGameMng::step()
{
  // wait for gamecode thread to finish its work
  sasWaitForSemaphore( _threadData._gamecodeDone );

  // sync window in between render and game thread start
  sasSystemsUpdate();

  float dt = sasTimer::singletonPtr()->getElapsedTime();
  _inputMng->step();
  stepCamera( dt );

  _boidMng->syncWindowStep();

  // sync window in between render and game thread end

  // kick gamecode thread
  sasSignalSemaphore( _threadData._syncWithEngineDone );

  //kick off rendering
  sasUpdate();  

}

void sgGameMng::stepCamera( float dt )
{
  if( _inputMng->controllerMng()->isControllerAvailable() )
  {
    _inputMng->controllerMng()->setControllerInvertedY( sasGetControllerInverted() );
  }

  _camMng->step( _inputMng, dt );
}

void sgGameMng::stepGamecodeThread( float dt )
{
  _boidMng->stepSimulation(dt);
  _boidMng->flushRenderCommands();
}
