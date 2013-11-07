#include "AStarNode.h"
#include <math.h>
#include "common.h"
#include "BZDBCache.h"
#include "World.h"
#include "playing.h" // needed for controlPanel

int GraphFunctionContainer::Xmin, GraphFunctionContainer::Xmax;
int GraphFunctionContainer::Ymin, GraphFunctionContainer::Ymax;

AStarNode::AStarNode(void)
{
	x = y = 0;
}

AStarNode::~AStarNode(void)
{
}

AStarNode::AStarNode(const float location[3])
{
	x = (location[0] > 0.0) ? (int)floor(location[0]/SCALE + 0.5f) : (int)ceil(location[0]/SCALE - 0.5f);
	y = (location[1] > 0.0) ? (int)floor(location[1]/SCALE + 0.5f) : (int)ceil(location[1]/SCALE - 0.5f);
	if (AStarNode::isAccessible(x, y)) return;
	for (int a=-1; a<=1; a++)
		for (int b=-1; b<=1; b++) {
			if (a==0 && b==0) continue;
			x += a;
			y += b;
			if (AStarNode::isAccessible(x, y)) return;
		}
	char buffer[128];
	sprintf (buffer, "***AStarNode: could not find any isAccessible node for (%f, %f, %f)***", location[0], location[1], location[2]);
	controlPanel->addMessage(buffer);
}
AStarNode::AStarNode(int xi, int yi)
{
	x = xi;
	y = yi;
}
int GraphFunctionContainer::getHashBin(AStarNode& n)
{
	//int index = (abs(n.getX() - Xmin) * abs(Ymax - Ymin) + abs(n.getY() - Ymin)) % 1023;
	int index = abs(n.getX() * n.getY()) % 1023;
	return index;
}

bool AStarNode::isAccessible(int x, int y)
{
	// check for world boundaries
	if (x<GraphFunctionContainer::Xmin || x>GraphFunctionContainer::Xmax ||
		y<GraphFunctionContainer::Ymin || y>GraphFunctionContainer::Ymax)
		return false;
	// if not inside an obstacle
	float pos[3];
	pos[0] = x * SCALE;
	pos[1] = y * SCALE;
	pos[2] = 0.0f;

	return !(World::getWorld()->inBuilding(pos, BZDBCache::tankRadius/2, BZDBCache::tankHeight));
}

bool GraphFunctionContainer::isAccessible(AStarNode& n)
{
	return AStarNode::isAccessible(n.getX(), n.getY());
}

void GraphFunctionContainer::getSuccessors(AStarNode& n, std::vector<AStarNode>* s, std::vector<double>* c) // Define a 8-connected graph
{
	// This function needn't account for obstacles or size of environment. That's done by "isAccessible"
	AStarNode tn;
	s->clear(); c->clear(); // Planner is supposed to clear these. Still, for safety we clear it again.
	for (int a=-1; a<=1; a++)
		for (int b=-1; b<=1; b++) {
			if (a==0 && b==0) continue;
			tn.setX(n.getX() + a);
			tn.setY(n.getY() + b);
			s->push_back(tn);
		}
	*c = ConstCostVector; // Using the fixed cost vector

}

double GraphFunctionContainer::getHeuristics(AStarNode& n1, AStarNode& n2)
{
	// Euclidean distance as heuristics
	return (hypotf(n2.getX() - n1.getX(), n2.getY() - n1.getY()));
}

// -------------------------------
// constructors
GraphFunctionContainer::GraphFunctionContainer (float worldSize) 
{ 
	int size = (int)worldSize/SCALE/2;
	Xmin = -size; Xmax = size; Ymin = -size; Ymax = size;
	    
	// Defining the fixed transition costs
    double SQRT2 = sqrt(2.0);
    ConstCostVector.push_back(SQRT2); ConstCostVector.push_back(1.0);
    ConstCostVector.push_back(SQRT2); ConstCostVector.push_back(1.0);
    ConstCostVector.push_back(1.0); ConstCostVector.push_back(SQRT2);
    ConstCostVector.push_back(1.0); ConstCostVector.push_back(SQRT2);


}
