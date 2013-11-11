/*
 * Defines the classes used for decision trees.
 *
 * Part of the Artificial Intelligence for Games system.
 *
 * Copyright (c) Ian Millington 2003-2006. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */
#include "dectree.h"

namespace aicore
{

    DecisionTreeNode* Decision::makeDecision(RobotPlayer* bot, float dt)
    {
        // Choose a branch based on the getBranch method
        if (getBranch(bot, dt)) {
            // Make sure its not null before recursing.
            if (trueBranch == NULL) 
				return NULL;
            else 
				return trueBranch->makeDecision(bot, dt);
        } 
		else {
            // Make sure its not null before recursing.
            if (falseBranch == NULL) 
				return NULL;
            else 
				return falseBranch->makeDecision(bot, dt);
        }
    }
    
	DecisionTreeNode* DecisionPtr::makeDecision(RobotPlayer* bot, float dt)
    {
        // Choose a branch based on the getBranch method
        if ( getBranch(bot, dt) ) {
            // Make sure its not null before recursing.
            if (trueBranch == NULL) 
				return NULL;
            else 
				return trueBranch->makeDecision(bot, dt);
        } 
		else {
            // Make sure its not null before recursing.
            if (falseBranch == NULL) 
				return NULL;
            else 
				return falseBranch->makeDecision(bot, dt);
        }
    }

	bool DecisionPtr::getBranch(RobotPlayer* bot, float dt)
	{
		return (bot->*decFuncPtr)(dt);
	}

	// Set up the trees
	void DecisionTrees::init()
	{
		/*
		Q0: bot alive? 
		yes: Q1. 
		no: do nothing.
		*/
		doUpdateMotionDecisions[0].decFuncPtr	= &RobotPlayer::amAlive;
		doUpdateMotionDecisions[0].trueBranch	= &doUpdateMotionDecisions[1];
		doUpdateMotionDecisions[0].falseBranch	= &doUpdateMotionActions[0];
		/*
		Q1: is there is a non-invisible, non-isExpired() shot about to hit bot? 
		yes: evade. 
		no: follow A* search path.
		*/
		doUpdateMotionDecisions[1].decFuncPtr	= &RobotPlayer::isShotDanger;
		doUpdateMotionDecisions[1].trueBranch	= &doUpdateMotionActions[1];
		doUpdateMotionDecisions[1].falseBranch	= &doUpdateMotionActions[2];

		doUpdateMotionActions[0].actFuncPtr = &RobotPlayer::doNothing;
		doUpdateMotionActions[1].actFuncPtr = &RobotPlayer::evade;
		doUpdateMotionActions[2].actFuncPtr = &RobotPlayer::followPath;

		/////////////////////////////doUpdateShoot////////////////////////////////////////
		/*
		Q0: bot alive? 
		yes: Q1. 
		no: do nothing.
		*/
		doUpdateShootDecisions[0].decFuncPtr = &RobotPlayer::amAlive;
		doUpdateShootDecisions[0].trueBranch = &doUpdateShootDecisions[1];
		doUpdateShootDecisions[0].falseBranch = &doUpdateShootActions[0];
		/*
		Q1: is firing status == Ready? 
		yes: Q2. 
		no: do nothing.
		*/
		doUpdateShootDecisions[1].decFuncPtr = &RobotPlayer::isShotReady;
		doUpdateShootDecisions[1].trueBranch = &doUpdateShootDecisions[2];
		doUpdateShootDecisions[1].falseBranch = &doUpdateShootActions[0];
		/*
		Q2: has shot timer elapsed? 
		yes: Q3. 
		no: do nothing.
		*/
		doUpdateShootDecisions[2].decFuncPtr = &RobotPlayer::isShotTimerElapsed;
		doUpdateShootDecisions[2].trueBranch = &doUpdateShootDecisions[3];
		doUpdateShootDecisions[2].falseBranch = &doUpdateShootActions[0];
		/*
		Q3: will shot miss target by less than half a tanklength? 
		yes: Q4. 
		no: do nothing.
		*/
		doUpdateShootDecisions[3].decFuncPtr = &RobotPlayer::isAimGood;
		doUpdateShootDecisions[3].trueBranch = &doUpdateShootDecisions[4];
		doUpdateShootDecisions[3].falseBranch = &doUpdateShootActions[0];
		/*
		Q4: are there any buildings in the way? 
		yes: do nothing. 
		no: Q5.
		*/
		doUpdateShootDecisions[4].decFuncPtr	= &RobotPlayer::isShotObstructedBuilding;
		doUpdateShootDecisions[4].trueBranch	= &doUpdateShootActions[1];
		doUpdateShootDecisions[4].falseBranch	= &doUpdateShootDecisions[5]; //never gets returned though code path runs this assignment...
		/*
		Q5: are there any teammates in the way? 
		yes: set shot timer to 0.1f. 
		no: fireShot() and reset shot timer.
		*/
		doUpdateShootDecisions[5].decFuncPtr	= &RobotPlayer::isShotObstructedAlly;  //[5] never got written into memory properly, trying this workaround
		doUpdateShootDecisions[5].trueBranch	= &doUpdateShootActions[1];
		doUpdateShootDecisions[5].falseBranch	= &doUpdateShootActions[2];

		//actions
		doUpdateShootActions[0].actFuncPtr = &RobotPlayer::doNothing;
		doUpdateShootActions[1].actFuncPtr = &RobotPlayer::delayShot;
		doUpdateShootActions[2].actFuncPtr = &RobotPlayer::shoot;

		///////////////////////////////////////doUpdate of flags/////////////////////////////////////////////////////////////////
		/*
		The drop flags decision tree of doUpdate() looks like:
		Q0: bot alive? 
		yes: Q1. 
		no: do nothing.
		*/
		doUpdateFlagsDecisions[0].decFuncPtr = &RobotPlayer::amAlive;
		doUpdateFlagsDecisions[0].trueBranch = &doUpdateFlagsDecisions[1];
		doUpdateFlagsDecisions[0].falseBranch = &doUpdateFlagsActions[0];
		/*
		Q1: is bot holding a flag? 
		yes: Q2. 
		no: do nothing.
		*/
		doUpdateFlagsDecisions[1].decFuncPtr = &RobotPlayer::isHoldingFlag;
		doUpdateFlagsDecisions[1].trueBranch = &doUpdateFlagsDecisions[2];
		doUpdateFlagsDecisions[1].falseBranch = &doUpdateFlagsActions[0];
		/*
		Q2: is the flag sticky? 
		yes: do nothing. 
		no: Q3.
		*/
		doUpdateFlagsDecisions[2].decFuncPtr = &RobotPlayer::isFlagSticky;
		doUpdateFlagsDecisions[2].trueBranch = &doUpdateFlagsActions[0];
		doUpdateFlagsDecisions[2].falseBranch = &doUpdateFlagsDecisions[3];
		/*
		Q3: is the flag a team flag? 
		yes: Q4. 
		no: drop flag.
		*/
		doUpdateFlagsDecisions[3].decFuncPtr  = &RobotPlayer::isHoldingTeamFlag;
		doUpdateFlagsDecisions[3].trueBranch  = &doUpdateFlagsDecisions[4];
		doUpdateFlagsDecisions[3].falseBranch = &doUpdateFlagsActions[2]; //not working somehow
		/*
		Q4: is the flag's team == my team? 
		yes: drop flag (note that this can be the same ActionPtr as the yes part of Q5). 
		no: do nothing.
		*/
		doUpdateFlagsDecisions[4].decFuncPtr  = &RobotPlayer::isHoldingMyTeamFlag;
		doUpdateFlagsDecisions[4].trueBranch  = &doUpdateFlagsActions[2];
		doUpdateFlagsDecisions[4].falseBranch = &doUpdateFlagsActions[0];

		//actions
		doUpdateFlagsActions[0].actFuncPtr = &RobotPlayer::doNothing;
		doUpdateFlagsActions[1].actFuncPtr = &RobotPlayer::delayShot;//try to eliminate bug. It hates [1] for some reason
		doUpdateFlagsActions[2].actFuncPtr = &RobotPlayer::dropFlag;
	}

	DecisionPtr DecisionTrees::doUpdateMotionDecisions[2];
	DecisionPtr DecisionTrees::doUpdateShootDecisions[6];
	DecisionPtr DecisionTrees::doUpdateFlagsDecisions[5];
	ActionPtr DecisionTrees::doUpdateMotionActions[3];
	ActionPtr DecisionTrees::doUpdateShootActions[3];
	ActionPtr DecisionTrees::doUpdateFlagsActions[3];

}; // end of namespace
