#pragma once
/// Semih Guresci semih.guresci@gmail.com
#include "entt/entt.hpp"
#include <comp/sg_moveCmp.h>
#include <comp/kdtree.h>
#include "utils/sas_singleton.h"
#include "threading/sas_thread.h"
#include "threading/sas_semaphore.h"
#include "threading/sas_atomic.h"
#include <chrono>

namespace ecsBoid
{
	/// <summary>
	/// Boid ECS Movement System
	/// </summary>
	class sg_VelocitySystem : public sasSingleton< sg_VelocitySystem >
	{
	private :
		// min dist between flocks
		const float epsilon = 0.05f;
		// ECS Registery
		const std::shared_ptr<entt::registry> m_registery;
		// KD Tree
		std::unique_ptr<KDTree::KdTree> m_tree;

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
		 // KD Tree Nores
		 std::vector<KDTree::KdNode>  nodes;

		 //PhysicsThreadData data;
		 //sasThread _KDThread;
		 bool usePhysicsUpdate = true;
		 std::chrono::high_resolution_clock::time_point physicsUpdateTime; // = std::chrono::high_resolution_clock::now();
		 std::chrono::milliseconds PhysicsUpdate {30};
	public:
		// Update Flock
		sg_VelocitySystem(const std::shared_ptr<entt::registry> m_reg, const smVec3 worldMin, const smVec3 worldMax);

		void UpdateMovement(const float dt);
		void ConstructSceneKDTree();

	
	};
}

