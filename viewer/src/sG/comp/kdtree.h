#pragma once
// Semih Guresci semih.guresci@gmail.com
// KD tree algorithm to reduce nearest  neighboor loops
#include <sm_vector.h>
#include <entt/entt.hpp>
#include <stdexcept>
#include <algorithm>

namespace KDTree 
{
	struct KdNode
	{
		entt::entity id;
		smVec3 point;
	};

	struct nn4heap 
	{
		size_t dataindex;  // index of actual kdnode in *allnodes*
		float distance;   // distance of this neighbor from *point*
		nn4heap(size_t i, double d) 
		{
			dataindex = i;
			distance = d;
		}
	};
	
	struct compare_nn4heap 
	{
		bool operator()(const nn4heap& n, const nn4heap& m) 
		{
			return (n.distance < m.distance);
		}
	};
	
	struct DistanceMeasure {
	public:
		DistanceMeasure() {}
		virtual ~DistanceMeasure() {}
		float distance(const smVec3& p, const smVec3& q)
		{
			return smLength3(q - p);
		}
		float distanceSq(const smVec3& p, const smVec3& q)
		{
			return smLengthSq3(q - p);
		}

		double coordinate_distance(const double& x,const double& y) 
		{
				return (x - y) * (x - y);
		}
	};

	struct compare_vec3 {
		compare_vec3() { }
		bool operator()(const KdNode& p, const KdNode& q) {
			return (p.point < q.point);
		}
		size_t d;
	};

	struct compare_dimension{
		compare_dimension(size_t dim) { d = dim; }
		bool operator()(const KdNode& p, const KdNode& q) {
			return (p.point[d] < q.point[d]);
		}
		size_t d;
	};

	struct kdtree_node {
	public:
		kdtree_node() {
			dataindex = 0;
			loson = hison = (kdtree_node*)NULL;
		}
		~kdtree_node() {
			if (loson) delete loson;
			if (hison) delete hison;
		}
		// index of node data in kdtree array "allnodes"
		size_t dataindex;
	
		// cutting dimension
		size_t cutdim;

		smVec3 point;
		//  roots of the two subtrees
		kdtree_node* loson, * hison;
		// bounding rectangle of this node's subtree
		smVec3 lobound, upbound;

	};

	struct KdTree
	{
		smVec3 lobound;
		smVec3 upbound;

		std::vector<KdNode>* allnodes;
		kdtree_node* root{ nullptr };
		DistanceMeasure* distance {nullptr};

		size_t dimension;

		KdTree(std::vector<KdNode>& nodes)
		{		
			allnodes = &nodes;
			if (distance) delete distance;
				distance = new DistanceMeasure();
		}

		void UpdateNodes(std::vector<KdNode>& nodes)
		{
			if (!allnodes || allnodes->size() == 0 || nodes.size() == 0 )
			{
				if (allnodes)
					allnodes->clear();
				return;
			}
			
			// TODO: A pooling system can be used for KdNode
			if (root) delete root;
			allnodes = &nodes;

		
			// compute global bounding box
			lobound = allnodes->begin()->point;
			upbound = allnodes->begin()->point;
			dimension = 3;
			
			for (std::vector<KdNode>::iterator it=allnodes->begin(); it!= allnodes->end();it++)
			{
				smMin(lobound, (it)->point, lobound);
				smMax(upbound, (it)->point, upbound);
			}
			size_t i;
			for (i = 1; i < allnodes->size(); i++) 
			{
				lobound.X = std::min(lobound.X, allnodes->at(i).point.X);
				upbound.X = std::max(upbound.X, allnodes->at(i).point.X);

				lobound.Y = std::min(lobound.Y, allnodes->at(i).point.Y);
				upbound.Y = std::max(upbound.Y, allnodes->at(i).point.Y);

				lobound.Z = std::min(lobound.Z, allnodes->at(i).point.Z);
				upbound.Z = std::max(upbound.Z, allnodes->at(i).point.Z);
			}

			// build tree recursively
			if(allnodes->size()>0)
				root = build_tree(0, 0, allnodes->size());
		}

		kdtree_node* build_tree(size_t depth, size_t a, size_t b) 
			{
				size_t m;
				float temp, cutval;
				kdtree_node* node = new kdtree_node();
				node->lobound = lobound;
				node->upbound = upbound;
				node->cutdim = depth % dimension;
				if (b - a <= 1) {
					node->dataindex = a;
					node->point = allnodes->at(a).point;
				}
				else {
					m = (a + b) / 2;
					std::nth_element(allnodes->begin() + a, allnodes->begin() + m, allnodes->begin() + b, compare_dimension(node->cutdim));
					node->point = allnodes->at(m).point;
					cutval = allnodes->at(m).point[node->cutdim];
					node->dataindex = m;
					if (m - a > 0) {
						temp = upbound[node->cutdim];
						upbound[node->cutdim] = cutval;
						node->loson = build_tree(depth + 1, a, m);
						upbound[node->cutdim] = temp;
					}
					if (b - m > 1) {
						temp = lobound[node->cutdim];
						lobound[node->cutdim] = cutval;
						node->hison = build_tree(depth + 1, m + 1, b);
						lobound[node->cutdim] = temp;
					}
				}
				return node;
			}

		bool bounds_overlap_ball(const smVec3& point, float dist, kdtree_node* node)
			{

			double distsum = 0.0;
			if (point.X < node->lobound.X) 
			{  // lower than low boundary
				distsum += distance->coordinate_distance(point.X, node->lobound.X);
			}
			else if (point.X > node->upbound.X)
			{  
				// higher than high boundary
				distsum += distance->coordinate_distance(point.X, node->upbound.X);
			}

			if (point.Y < node->lobound.Y)
			{  // lower than low boundary
				distsum += distance->coordinate_distance(point.Y, node->lobound.Y);
			}
			else if (point.Y > node->upbound.Y)
			{
				// higher than high boundary
				distsum += distance->coordinate_distance(point.Y, node->upbound.Y);
			}

			if (point.Z < node->lobound.Z)
			{  
				// lower than low boundary
				distsum += distance->coordinate_distance(point.Z, node->lobound.Z);
			}
			else if (point.Z > node->upbound.Z)
			{
				// higher than high boundary
				distsum += distance->coordinate_distance(point.Z, node->upbound.Z);
			}

			if (distsum > dist) return false;

			return true;
			}
				
		void range_search(std::vector<size_t>& range_result, const smVec3& point, kdtree_node* node, float r) {
			float curdist = distance->distanceSq(point, node->point);
			if (curdist <= r) {
				range_result.push_back(node->dataindex);
			}
			if (node->loson != NULL && this->bounds_overlap_ball(point, r, node->loson)) {
				range_search(range_result, point, node->loson, r);
			}
			if (node->hison != NULL && this->bounds_overlap_ball(point, r, node->hison)) {
				range_search(range_result ,point, node->hison, r);
			}
		}
		
		void range_nearest_neighbors(const smVec3& point, float r, std::vector<entt::entity>& result) 
		{
				KdNode temp;
				std::vector<size_t> range_result;

				// squared distance
				r *= r;

				// collect result in neighborheap
				range_search(range_result, point, root, r);

				// copy over result
				for (std::vector<size_t>::iterator i = range_result.begin(); i != range_result.end(); ++i) {
					result.push_back(allnodes->at(*i).id);
				}

				// clear vector
				range_result.clear();
			}

		~KdTree() 
		{
			allnodes->clear();
			if (root) delete root;
			delete distance;
		}

	};

}