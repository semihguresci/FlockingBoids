#pragma once
/// Semih Guresci semih.guresci@gmail.com
#include <sm_vector.h>
#include <sg_boid.h>

namespace ecsBoid 
{

	struct transformComponent
	{
		transformComponent()= default;
		transformComponent(const transformComponent& other) = default;

		transformComponent(smVec3 pos) :_pos ( pos)
		{
		}

		smVec3 _pos;
	};

	struct movementComponent
	{
		movementComponent() = default;
		movementComponent(const movementComponent& other) = default;

		movementComponent(smVec3 vel, float velCoef) : _vel(vel), _velCoef(velCoef)
		{
		}

		smVec3 _vel;
		float _velCoef;
		
	};

	struct BoidSettings
	{
		float  _radius;
		float _proximityTestRange;
		float _boidVelocity;
		float _alignmentWeight;
		float _cohesionWeight;
		float _cohesionWeightWithOtherBoidType;
		float _separationWeight;
		float _separationWeightWithOtherBoidType;
		sgBoidType::Enum _boidType;
	};

	struct boidComponent 
	{
		float _proximityTestRange;
		float _radius;
		sgBoidType::Enum _boidType;

		float _alignmentWeight{0.f};
		float _cohesionWeight{ 0.f };
		float _cohesionWeightWithOtherBoidType{ 0.f };
		float _separationWeight{ 1.f };
		float _separationWeightWithOtherBoidType{ 1.f };
		
		boidComponent() = default;
		boidComponent(const boidComponent& other) = default;
		boidComponent(float proximityTestRange, float radius, sgBoidType::Enum boidType,
		float alignmentWeight,
		float cohesionWeight,
		float cohesionWeightWithOtherBoidType,
		float separationWeight,
		float separationWeightWithOtherBoidType) :
			_proximityTestRange(proximityTestRange), 
			_radius(radius), 
			_boidType(boidType),
			_alignmentWeight(alignmentWeight), 
			_cohesionWeight(cohesionWeight),
			_cohesionWeightWithOtherBoidType(cohesionWeightWithOtherBoidType),
			_separationWeight(separationWeight),
			_separationWeightWithOtherBoidType(separationWeightWithOtherBoidType)
		{

		}

		boidComponent(const BoidSettings& other) 
		{
			_proximityTestRange = other._proximityTestRange;
			_radius = other._radius;
			_boidType = other._boidType;
			_alignmentWeight = other._alignmentWeight;
			_cohesionWeight = other._cohesionWeight;
			_cohesionWeightWithOtherBoidType = other._cohesionWeightWithOtherBoidType;
			_separationWeight= other._separationWeight;
			_separationWeightWithOtherBoidType= other._separationWeightWithOtherBoidType;
		};
		


	};

	
}