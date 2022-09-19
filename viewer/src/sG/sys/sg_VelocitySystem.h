#pragma once
/// Semih Guresci semih.guresci@gmail.com
#include "entt/entt.hpp"
#include <comp/sg_moveCmp.h>
#include <comp/kdtree.h>
#include "utils/sas_singleton.h"
#include "threading/sas_thread.h"
#include "threading/sas_semaphore.h"
#include "threading/sas_atomic.h"

namespace ecsBoid
{
	/// <summary>
	/// Boid ECS Movement System
	/// </summary>
	class sg_VelocitySystem : public sasSingleton< sg_VelocitySystem >
	{

	private :
		const float epsilon = 0.05f;
		const std::shared_ptr<entt::registry> m_registery;
		std::unique_ptr<ecsKDTree::KdTree> m_tree;
		const smVec3 _worldMin;
		const smVec3 _worldMax;
		smVec3 _edgeNormals[6] = 
						{ smVec3(1.f, 0.f, 0.f),
						smVec3(-1.f, 0.f, 0.f),
						smVec3(0.f, 1.f, 0.f),
						smVec3(0.f, -1.f, 0.f),
						smVec3(0.f, 0.f, 1.f),
						smVec3(0.f, 0.f, -1.f) };

		 smVec3 _edgePoint[6];
		 std::vector<ecsKDTree::KdNode>  nodes;

		 //PhysicsThreadData data;
		 //sasThread _KDThread;
	public:
		sg_VelocitySystem(const std::shared_ptr<entt::registry> m_reg, const smVec3 worldMin, const smVec3 worldMax);
		void UpdateMovement(const float dt);
		void ConstructSceneKDTree();
	
	};
}

