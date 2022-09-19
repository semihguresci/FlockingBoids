#include "sg_pch.h"
#include "sg_boidMng.h"
#include "sg_boid.h"




sgBoidMng::sgBoidMng( uint32_t smallBoidCount, uint32_t largeBoidCount )
    : _renderDataIndex( 0 )
    , _renderDataInitialized( false )
{
    smVec3 _worldMin(-25.f, -8.f, -25.f);
    smVec3 _worldMax(25.f, 8.f, 25.f);


    m_registery = std::make_shared < entt::registry > ();
    m_VelocitySystem = std::make_unique<ecsBoid::sg_VelocitySystem>(m_registery, _worldMin, _worldMax);

    _boidCount[ sgBoidType::small ] = smallBoidCount;
    _boidCount[ sgBoidType::big ] = largeBoidCount;
    


    ecsBoid::BoidSettings small = {
        ._radius = 0.2f,
        ._proximityTestRange = 5.f,
        ._boidVelocity = 8.f,
        ._alignmentWeight = 2.5f,
        ._cohesionWeight = 1.f,
        ._cohesionWeightWithOtherBoidType = 0.f,
        ._separationWeight = 1.8f,
        ._separationWeightWithOtherBoidType = 4.f,
        ._boidType = sgBoidType::small
    };

    ecsBoid::BoidSettings large =
    {
        ._radius = 1.0f,
        ._proximityTestRange = 10.f,
        ._boidVelocity = 7.f,
        ._alignmentWeight = 0.0f,
        ._cohesionWeight = 0.0f,
        ._cohesionWeightWithOtherBoidType = 0.5f,
        ._separationWeight = 2.f,
        ._separationWeightWithOtherBoidType = 0.0f,
        ._boidType = sgBoidType::big
    };



    for( uint32_t i = 0; i < smallBoidCount; i++ )
    {
        createBoid(small, _worldMin, _worldMax);
    }

    for( uint32_t i = 0; i < largeBoidCount; i++ )
    {
        createBoid(large, _worldMin, _worldMax);
    }

    m_VelocitySystem->ConstructSceneKDTree();

    for( uint32_t i = 0; i < 2; i++ )
    {
        _renderDataMemBlock[ sgBoidType::small ][ i ] = sasMalloc( sizeof( smVec4 ) * smallBoidCount);
        _renderDataMemBlock[ sgBoidType::big ][ i ] = sasMalloc( sizeof( smVec4 ) * largeBoidCount);
    }

    _boidRenderObjectId[ sgBoidType::small ] = sasStringId::build( "smallBoidMeshInst" );
    _boidRenderObjectId[ sgBoidType::big ] = sasStringId::build( "bigBoidMeshInst" );
    
}

sgBoidMng::~sgBoidMng()
{
    sasDeleteModelInstance( _boidRenderObjectId[ sgBoidType::small ] );
    sasDeleteModelInstance( _boidRenderObjectId[ sgBoidType::big ] );
    
    for( uint32_t i = 0; i < 2; i++ )
    {
        sasFree( _renderDataMemBlock[ sgBoidType::small ][ i ] );
        sasFree( _renderDataMemBlock[ sgBoidType::big ][ i ] );
    }
}


void sgBoidMng::createBoid(const ecsBoid::BoidSettings& setting, const smVec3& _worldMin, const smVec3& _worldMax)
{
    entt::entity entity = m_registery->create();
    smVec3 _pos = smVec3(getRandomizedFloatInRange(_worldMin.X, _worldMax.X), getRandomizedFloatInRange(_worldMin.Y, _worldMax.Y), getRandomizedFloatInRange(_worldMin.Z, _worldMax.Z));
    smVec3 _vel = smVec3(getRandomizedFloatInRange(-1.f, 1.f), getRandomizedFloatInRange(-0.5f, 0.5f), getRandomizedFloatInRange(-1.f, 1.f)); 
    smNormalize3(_vel, _vel);

    m_registery->emplace<ecsBoid::transformComponent>(entity, _pos);
    m_registery->emplace<ecsBoid::movementComponent>(entity, _vel * setting._boidVelocity, setting._boidVelocity);
    m_registery->emplace<ecsBoid::boidComponent>(entity, setting);
}


void sgBoidMng::stepSimulation(const float dt)
{
    m_VelocitySystem->UpdateMovement(dt);

    // secrified for single loop
    // m_VelocitySystem->UpdateTransform(dt);
    
    m_VelocitySystem->ConstructSceneKDTree();
    
    // secrified for single loop
    //m_VelocitySystem->CheckCollusion(dt);
}

void sgBoidMng::flushRenderCommands()
{
    smVec4* smallBoidRenderData = static_cast< smVec4* >( _renderDataMemBlock[ sgBoidType::small ][ _renderDataIndex ] );
    smVec4* largeBoidRenderData = static_cast< smVec4* >( _renderDataMemBlock[ sgBoidType::big ][ _renderDataIndex ] );

    for (auto&& [entity, transform, b] : m_registery->group<ecsBoid::transformComponent>(entt::get<ecsBoid::boidComponent>).each()) 
    {
        if( b._boidType == sgBoidType::small )
        {
            *smallBoidRenderData = smVec4(transform._pos.X, transform._pos.Y, transform._pos.Z, b._radius);
            smallBoidRenderData++;
        }
        else
        {
            *largeBoidRenderData = smVec4(transform._pos.X, transform._pos.Y, transform._pos.Z, b._radius);
            largeBoidRenderData++;
        }
    }
   
}

void sgBoidMng::syncWindowStep()
{ 
    _renderDataIndex = 1 - _renderDataIndex; 

    if( !_renderDataInitialized )
    {
        _renderDataInitialized = true;
        sasCreateBlobModelInstance( _boidRenderObjectId[ sgBoidType::small ] );
        sasCreateBlobModelInstance( _boidRenderObjectId[ sgBoidType::big ] );
        sasSetInstanceTintColour( _boidRenderObjectId[ sgBoidType::small ], smVec4( 1.f, 0.f, 0.5f, 1.f ) );
        sasSetInstanceTintColour( _boidRenderObjectId[ sgBoidType::big ], smVec4( 0.f, 1.f, 1.f, 1.f ) );
    }

    sasSetModelInstanceData( _boidRenderObjectId[ sgBoidType::small ], _renderDataMemBlock[ sgBoidType::small ][ _renderDataIndex ], _boidCount[ sgBoidType::small ] * sizeof( smVec4 ), _boidCount[ sgBoidType::small ] );
    sasSetModelInstanceData( _boidRenderObjectId[ sgBoidType::big ], _renderDataMemBlock[ sgBoidType::big ][ _renderDataIndex ], _boidCount[ sgBoidType::big ] * sizeof( smVec4 ), _boidCount[ sgBoidType::big ] );
}

