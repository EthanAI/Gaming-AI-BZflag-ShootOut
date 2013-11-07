/******************************************************************************************
*                                                                                        *
*    Part of                                                                             *
*    Yet Another Graph-Search Based Planning Library (YAGSBPL)                           *
*    A template-based C++ library for graph search and planning                          *
*    Version 1.0                                                                         *
*    ----------------------------------------------------------                          *
*    Copyright (C) 2010  Subhrajit Bhattacharya                                          *
*                                                                                        *
*    This program is free software: you can redistribute it and/or modify                *
*    it under the terms of the GNU General Public License as published by                *
*    the Free Software Foundation, either version 3 of the License, or                   *
*    (at your option) any later version.                                                 *
*                                                                                        *
*    This program is distributed in the hope that it will be useful,                     *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of                      *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                       *
*    GNU General Public License for more details <http://www.gnu.org/licenses/>.         *
*                                                                                        *
*                                                                                        *
*    Contact: subhrajit@gmail.com, http://fling.seas.upenn.edu/~subhrabh/                *
*                                                                                        *
*                                                                                        *
******************************************************************************************/
//    For a detailed tutorial and download, visit 
//    http://fling.seas.upenn.edu/~subhrabh/cgi-bin/wiki/index.php?n=Projects.ProgrammingLibraries-YAGSBPL


// =======================
// YAGSBPL libraries
#include "yagsbpl_base.h"
#include "A_star.h"

#include "World.h"
#include "BZDBCache.h"
#include <cmath>

// A node of the graph
//myNode also handles the mapping between absolute points of the game world and the centroids of the zones we use for path planning
class myNode
{
public:
	//record of what we multiply between planning and real coordinates (really just a gap in investigation resolution)
	int conversionFactor; 

	//default constructutor sets conversion factor
	//Tanks should be able to contact any point on the field if this value is set to tankradius or smaller. 
	//Unfortunately, that value still causes too much lag on my poor laptop. (tanks drift real far waiting for dt to update)
	//Consider implementing pathnfinding over multiple dt instead of all at once. 
	myNode() { 
		conversionFactor = 10; //floor(BZDBCache::tankRadius); 								 
	};

	//rounding forces nodes to center on the planning points and skip all the real world points in between. 
	//Essential to reduce number of points checked in path planning
	void roundNode() {
		x = x - (x % conversionFactor); 
		y = y - (y % conversionFactor);
	}
	int x, y; 
	// == operator defined for nodes so if you are within our desired level of resolution, 
	// you are close enough to be considered the same node
	// roundNode() should make this unecessary, but just in case. 
	bool operator==(const myNode& n) { 
		return (abs(n.x - x) <= ceil((float)n.conversionFactor/2) && abs(n.y - y) <= ceil((float)n.conversionFactor/2)); 
	};
};


// ============================================================
// Functions that describe the graph, placed inside a class
// Here the class "GraphFunctionContainer" is redefining the virtual functions declared in "SearchGraphDescriptorFunctionContainer"
// Thus the names of the functions are important here.

class GraphFunctionContainer : public SearchGraphDescriptorFunctionContainer<myNode,double>
{
public:

	int getHashBin(myNode& n) // Use the absolute value of x coordinate as hash bin counter. Not a good choice though!
	{
		return ((int)fabs((double)n.x));
	}

	//Is the node a node the tank can drive to?
	//Checks 1. node center is on the bloody map (can't believe that needed to be added)
	//       2. node center is far enough away from any objects that our tank center could fit there
	bool isAccessible(myNode& n)
	{
		//to compare against objects in the world we need to reExpand the coordinate frame
		float position[3] = {n.x, n.y, 0}; 
		//new code to add what I think is good distance to stay away from obstacles
		bool inBuilding = World::getWorld()->inBuilding(position, BZDBCache::tankRadius, BZDBCache::tankHeight);
		//also check if were out of the bloody map! That won't be in a building.
		float xMapMin = -1*BZDBCache::worldSize/2;
		float xMapMax = BZDBCache::worldSize/2;
		float yMapMin = -1*BZDBCache::worldSize/2;
		float yMapMax = BZDBCache::worldSize/2;
		bool onMap = (n.x >= xMapMin && n.x <= xMapMax && n.y >= yMapMin && n.y <= yMapMax);

		bool available = (onMap && !inBuilding);
		return(available);

		// Environment with (0,0) at center, and a circular obstacle of radius 'circleRadius' centered at (0,0)
		//return ( n.x*n.x + n.y*n.y > circleRadius*circleRadius && n.x>=Xmin && n.x<=Xmax && n.y>=Ymin && n.y<=Ymax );
	}

	/* This is how yagsbpl gets more nodes to test in path planning. Restricting this is how we 1. Reduce the points checked to make Astar faster and
		2. set the domain explored to be centroids of squares the size of a tank instead of every point on the map
		FINALPROJECTTODO: pass in targetNode, get distance, increase resolution when distance is small. 
							That way we can have cheap planning to get near the target, and still have a precise arrival node
	*/
	void getSuccessors(myNode& n, std::vector<myNode>* s, std::vector<double>* c) // Define a 8-connected graph //also where the nodes get defined
	{
		// This function needn't account for obstacles or size of environment. That's done by "isAccessible"
		myNode tn;		 
		s->clear(); c->clear(); // Planner is supposed to clear these. Still, for safety we clear it again.
		for (int a=-1; a<=1; a++)
			for (int b=-1; b<=1; b++) {
				if (a==0 && b==0) continue;
				tn.x = n.x + a * n.conversionFactor;  
				tn.y = n.y + b * n.conversionFactor;
				s->push_back(tn);
				c->push_back(sqrt((double)(a*a+b*b)));
			}
	}

	//Heuristic for guessing target path cost. Using a different way to calculate distance but its the same result
	//FINALPROJECTTODO: alter cost based on enemy concentrations, enemy bases, ally locations, our flag location etc. In swarm mode we can hold multiple flags
	double getHeuristics(myNode& n1, myNode& n2)
	{
		int dx = abs(n1.x - n2.x);
		int dy = abs(n1.y - n2.y);
		//return (sqrt((double)(dx*dx + dy*dy))); // Euclidean distance as heuristics //Looks like thats what we want. Maybe replace with hypotf() for efficiency?
		return( hypotf(dx, dy) ); //get true distance for ease of understanding 
	}

	// -------------------------------
	// constructors
	GraphFunctionContainer (int cr, std::vector<int> ranges) 
		{ circleRadius = cr; Xmin = ranges[0]; Xmax = ranges[1]; Ymin = ranges[2]; Ymax = ranges[3]; }
	GraphFunctionContainer () 
		{ circleRadius = 10; Xmin = -20; Xmax = 20; Ymin = -20; Ymax = 20; } //probably not used anymore

private:

	int circleRadius;
	int Xmin, Xmax, Ymin, Ymax;

};


