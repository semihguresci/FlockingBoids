#pragma once 

#include "utils/sas_singleton.h"
#include "sg_boid.h"
#include "entt/entt.hpp"
#include <comp/sg_moveCmp.h>
#include "sys/sg_VelocitySystem.h"


static float getRandomizedFloatInRange(float minRange, float maxRange)
{
    float r = ((float)rand()) / (float)RAND_MAX;
    return r * (maxRange - minRange) + minRange;
}

class sgBoidMng : public sasSingleton< sgBoidMng >
{
    sasNO_COPY( sgBoidMng );
    sasMEM_OP( sgBoidMng );

public:
    sgBoidMng(uint32_t smallBoidCount, uint32_t largeBoidCount );
    ~sgBoidMng();


    void stepSimulation(const float dt);
    void flushRenderCommands();
    void syncWindowStep(); //happens when render and game threads are in sync
    void createBoid(const ecsBoid::BoidSettings& setting, const smVec3& _worldMin, const smVec3& _worldMax);
private:
    
    std::unique_ptr <ecsBoid::sg_VelocitySystem> m_VelocitySystem;
    std::shared_ptr <entt::registry> m_registery;


    //std::vector< sgBoid* > _boids;
    void* _renderDataMemBlock[ sgBoidType::count ][ 2 ];
    uint32_t _renderDataIndex; //We ping pong in between 2 buffers per boid type since game logic runs on different thread than render thread
    sasStringId _boidRenderObjectId[ sgBoidType::count ];
    uint32_t _boidCount[ sgBoidType::count ];
    bool _renderDataInitialized;
 
    
};
