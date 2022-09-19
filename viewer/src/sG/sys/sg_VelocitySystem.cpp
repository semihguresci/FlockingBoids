#include "sg_pch.h"

#include "sg_VelocitySystem.h"
#include <execution>
#include <platform/win32/sas_pl_timer.h>

using namespace ecsKDTree;

ecsBoid::sg_VelocitySystem::sg_VelocitySystem(const std::shared_ptr<entt::registry> m_reg, const smVec3 worldMin, const smVec3 worldMax) : m_registery(m_reg), _worldMin(worldMin), _worldMax(worldMax)
{
	_edgePoint[0] = smVec3(_worldMin.X, 0.f, 0.f);
	_edgePoint[1] = smVec3(_worldMax.X, 0.f, 0.f);
	_edgePoint[2] = smVec3(0.f, _worldMin.Y, 0.f);
	_edgePoint[3] = smVec3(0.f, _worldMax.Y, 0.f);
	_edgePoint[4] = smVec3(0.f, 0.f, _worldMin.Z);
	_edgePoint[5] = smVec3(0.f, 0.f, _worldMax.Z);
	m_tree = std::make_unique<KdTree>(nodes);
}

void ecsBoid::sg_VelocitySystem::UpdateMovement(float dt)
{
	const float *epsl = &epsilon;
	smVec3* edgeNormals = _edgeNormals;
	smVec3 *edgePoint = _edgePoint;
	ecsKDTree::KdTree* kdtreeptr = m_tree.get();
	smVec3 worldMin (_worldMin);
	smVec3 worldMax (_worldMax);
	entt::basic_view view = m_registery->view<transformComponent, movementComponent , boidComponent>();
	

	std::for_each(std::execution::par_unseq, view.begin(), view.end(), [&view, &dt, &edgePoint, &edgeNormals, &worldMin, &worldMax, &kdtreeptr, epsl](auto entity)
	{
		transformComponent& m_transform = view.get<transformComponent>(entity);
		movementComponent& m_movement = view.get<movementComponent>(entity);
		const boidComponent& m_boid = view.get<boidComponent>(entity);
		
		//alignment with other boids of same type
		smVec3 nearbySameFlockVel(0.f);
		float nearbyBoidSameWeight = 0.f;
		uint32_t numNearbySameBoids = 0;
		smVec3 alignmentSameForce(0.f);

		float _proximityTestRangesq = m_boid._proximityTestRange;

		//cohesion with other boids of same type
		smVec3 cohesionForce(0.f);
		smVec3 nearbySameFlockPos(0.f);

		//cohesion with other boids of different type
		smVec3 nearbyOtherTypeFlockPos(0.f);
		uint32_t numNearbyOtherBoids = 0;
		smVec3 cohesionForceWithOtherType(0.f);

		// seperation same type
		smVec3 posToNearbySameFlock(0.f);
		smVec3 separationSameForce(0.f);

		// separation different type
		smVec3 posToNearbyFlockOfOtherType(0.f);
		smVec3 separationForceWithOtherType(0.f);
		float  nearbyOtherBoidWeight = 0.f;

		smVec3 colDist(0.f);

		std::vector<entt::entity>  closest;
		kdtreeptr->range_nearest_neighbors(m_transform._pos, (m_boid._proximityTestRange), closest);

		std::for_each(std::execution::seq, closest.begin(), closest.end(), [&view, &entity, &m_boid ,&m_transform,
				&_proximityTestRangesq, &nearbySameFlockVel,
				&nearbyBoidSameWeight, &numNearbySameBoids, &nearbySameFlockPos,
				&nearbyOtherTypeFlockPos, &numNearbyOtherBoids, &posToNearbySameFlock,
				&posToNearbyFlockOfOtherType, &nearbyOtherBoidWeight, &colDist]
				 (entt::entity oentity)
	{
		const transformComponent& o_transform = view.get<transformComponent>(oentity);
		const boidComponent& o_boid = view.get<boidComponent>(oentity);
	    const movementComponent& o_movement = view.get<movementComponent>(oentity);

		float dist = smLength3(o_transform._pos - m_transform._pos);
		float impactWeight = 1.f - dist / _proximityTestRangesq;

		if(entity != oentity && dist < _proximityTestRangesq && m_boid._boidType == o_boid._boidType)
		{
			// alignment
			nearbySameFlockVel += o_movement._vel* impactWeight;
			nearbyBoidSameWeight += impactWeight;
			++numNearbySameBoids;

			// cohesion
			nearbySameFlockPos += o_transform._pos;

			// separation
			posToNearbySameFlock += (o_transform._pos - m_transform._pos) * impactWeight;
		}

		if(entity != oentity && dist < _proximityTestRangesq && m_boid._boidType != o_boid._boidType)
		{
			nearbyOtherBoidWeight += impactWeight;
			++numNearbyOtherBoids;

			 //cohesion
			nearbyOtherTypeFlockPos += o_transform._pos;

			//separation from other boids of different type
			posToNearbyFlockOfOtherType += (o_transform._pos - m_transform._pos) * impactWeight;
			
		}

		// single pass collusion check
		if (entity != oentity && dist < (m_boid._radius + o_boid._radius) )
		{
			smVec3 unitPosToBoid;
			smNormalize3(o_transform._pos - m_transform._pos, unitPosToBoid);
			colDist -= (unitPosToBoid * (m_boid._radius + o_boid._radius)) - (o_transform._pos - m_transform._pos);
		}
		
	});
			
		// alignment & separation same type
		if (nearbyBoidSameWeight > 0)
		{
			// alignement
			nearbySameFlockVel /= static_cast<float>(numNearbySameBoids);
			smNormalize3(nearbySameFlockVel, alignmentSameForce);
			alignmentSameForce *= m_boid._alignmentWeight;

			// seperation
			posToNearbySameFlock /= static_cast<float>(numNearbySameBoids);
			posToNearbySameFlock *= -1.f;
			smNormalize3(posToNearbySameFlock, separationSameForce);
			separationSameForce *= m_boid._separationWeight;
		
		}
		// cohesion with same
		if (numNearbySameBoids > 0)
		{
			nearbySameFlockPos /= static_cast<float>(numNearbySameBoids);
			smVec3 posToAvgFlockPos = nearbySameFlockPos - m_transform._pos;
			smNormalize3(posToAvgFlockPos, cohesionForce);
			cohesionForce *= m_boid._cohesionWeight;
		}
		// cohesion with others
		if (numNearbyOtherBoids > 0)
		{
			nearbyOtherTypeFlockPos /= static_cast<float>(numNearbyOtherBoids);
			smVec3 posToAvgFlockPos = nearbyOtherTypeFlockPos - m_transform._pos;
			smNormalize3(posToAvgFlockPos, cohesionForceWithOtherType);
			cohesionForceWithOtherType *= m_boid._cohesionWeightWithOtherBoidType;
		}
		// separation other type
		if (nearbyOtherBoidWeight > 0)
		{
			posToNearbyFlockOfOtherType /= static_cast<float>(numNearbyOtherBoids);
			posToNearbyFlockOfOtherType *= -1.f;
			smNormalize3(posToNearbyFlockOfOtherType, separationForceWithOtherType);
			separationForceWithOtherType *= m_boid._separationWeightWithOtherBoidType;
		}

		//separation from edges
		smVec3 posToNearbyEdges(0.f);
		float nearbyEdgeWeight = 0.f;
		uint32_t numNearbyEdges = 0;
		smVec3 separationForceWithEdge(0.f);
		
		for (uint32_t i = 0; i < 6; i++)
		{
			float distToPlane = smDot3(m_transform._pos - edgePoint[i], edgeNormals[i]);
			if (distToPlane < _proximityTestRangesq)
			{
				float w = 1.f - (distToPlane / _proximityTestRangesq);
				posToNearbyEdges += distToPlane * edgeNormals[i] * w;
				nearbyEdgeWeight += w;
				numNearbyEdges++;
			}
		}
		if (nearbyEdgeWeight > 0)
		{
			posToNearbyEdges /= static_cast<float>(numNearbyEdges);
			smNormalize3(posToNearbyEdges, separationForceWithEdge);
			separationForceWithEdge *= m_boid._separationWeightWithOtherBoidType * 1.f;
		}

		m_movement._vel += alignmentSameForce + separationSameForce + separationForceWithOtherType + cohesionForce + cohesionForceWithOtherType + separationForceWithEdge ;
		smNormalize3(m_movement._vel, m_movement._vel);
		m_movement._vel *= m_movement._velCoef;

		
		// Single Pass Translation and collusion Check
		smVec3 calcpos = m_transform._pos + m_movement._vel * dt;
		calcpos += colDist;
		
		calcpos.X = std::clamp(calcpos.X, worldMin.X + (*epsl), worldMax.X - (*epsl));
		calcpos.Y = std::clamp(calcpos.Y, worldMin.Y + (*epsl), worldMax.Y - (*epsl));
		calcpos.Z = std::clamp(calcpos.Z, worldMin.Z + (*epsl), worldMax.Z - (*epsl));

	
		m_transform._pos = calcpos;
		
		});

}

void ecsBoid::sg_VelocitySystem::ConstructSceneKDTree()
{
	
	std::vector<KdNode>* nds = &nodes;
	auto group = m_registery->view<ecsBoid::transformComponent>();
	if (group.size() != nds->size())
		nds->resize(group.size());
	size_t startindex = 0;
	std::for_each(std::execution::seq, group.begin(), group.end(), [&group, &nds, &startindex]
	(auto entity)
		{
			const transformComponent& transform = group.get<transformComponent>(entity);
			nds->at(startindex).id = entity;
			nds->at(startindex).point = transform._pos;
			startindex++;
		});


	m_tree->UpdateNodes(nodes);


}

