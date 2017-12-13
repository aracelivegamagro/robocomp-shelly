/*
 * Copyright 2017 pbustos <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "pathfinder.h"

using namespace robocomp::pathfinder;

///////////////////////////////////////////////////////////////////
///  Public Interface
//////////////////////////////////////////////////////////////////

void PathFinder::go(float x, float z, const ParameterMap &parameters)
{
	qDebug() << "--------------------------------------------------------------------------";
	qDebug() << __FILE__ << __FUNCTION__ << "PathFinder::go New target arrived:" << x << z;
	Road &road = getRoad();
		controller.stopTheRobot();
		road.reset();
		road.setRequiresReplanning(true);
		currenttarget->setTranslation(QVec::vec3(x,0,z));
	releaseRoad();
};

///////////////////////////////////////////////////////////////////
void PathFinder::initialize( const InnerModelMgr &innerModel_, 
							 const shared_ptr< RoboCompCommonBehavior::ParameterList > &configparams_, 
							 LaserPrx laser_prx, OmniRobotPrx omnirobot_proxy )
{
	innerModel = innerModel_;
	configparams = configparams_;
	
	//Initialize global state class 
	state = std::make_shared<NavigationState>();
	
	
	/// Initialize the elastic road
	road.initialize(innerModel, state, configparams);
	
	/// Initialize currentarget
	currenttarget = std::make_shared<CurrentTarget>();
	
	#ifdef USE_QTGUI
		viewer = new InnerViewer( innerModel, "Social Navigation");  //InnerViewer copies internally innerModel so it has to be resynchronized
		viewer->start();	
	#endif
	
	/// Initialize the Planner
	pathplanner.initialize(currenttarget, innerModel, state, configparams);
	
	/// Initialize the Projector
	projector.initialize(innerModel, currenttarget, state, configparams, laser_prx);

	/// Initialize the low level controller that drives the robot on the road	
	controller.initialize(innerModel.get(), laser_prx, configparams, omnirobot_proxy);

	rDebug2(("PathFinder: All objects initialized"));
		
	/// Threads
	thread_planner = std::thread(&PathPlanner::run, &pathplanner, std::bind(&PathFinder::getRoad, this), std::bind(&PathFinder::releaseRoad, this));
	
	thread_projector = std::thread(&Projector::run, &projector, std::bind(&PathFinder::getRoad, this), std::bind(&PathFinder::releaseRoad, this));
	
	thread_controller = std::thread(&Controller::run, &controller, std::bind(&PathFinder::getRoad, this), std::bind(&PathFinder::releaseRoad, this));
	
	std::cout << __FUNCTION__ << "PathFinder: All threads initialized" << std::endl;
	rDebug2(("PathFinder: All threads initialized"));
}

void PathFinder::run()
{
 	while(true)
	{	
		Road &road = getRoad();
			QVec robotpos = innerModel->transformS6D("world", robotname);
			viewer->updateTransformValues(QString::fromStdString(robotname), robotpos);
			road.update();
// 			if( road.isBlocked())
// 				road.reset();
// 				road.setRequiresReplanning(true);
			drawroad.draw(road, viewer, currenttarget);
			drawroad.drawmap(pathplanner, viewer, pathplanner.fmap);
			
		releaseRoad();
	
		std::this_thread::sleep_for(200ms);
	
	}
}

void PathFinder::innerModelChanged ( InnerModelMgr &innerModel_, bool structural )
{
	if(structural) //replace all objects with copies of InnerModel. Broadcast a signal to subscribed objects
	{
		qDebug()<<"innerModelChanged TRUE";
		Road &road = getRoad(); //to block the threads
			pathplanner.reloadInnerModel(innerModel) ;  
			// road.reloadInnerModel( innerModel ) ;  
			projector.reloadInnerModel(innerModel ) ;  
			// controller.reloadInnerModel( innerModel );
		releaseRoad();
		//viewer->reloadInnerModel(innerModel);
	}
	else
	{	
		
		#ifdef USE_QTGUI
		qDebug()<<"innerModelChanged FALSE";
			
			if(viewer != nullptr)
			{
				Road &road = getRoad(); //to block the threads
					QVec robotpos = innerModel->transformS6D("world", robotname);
					viewer->updateTransformValues(QString::fromStdString(robotname), robotpos);
				releaseRoad();
			}
		#endif
	}
}


//////////////////////////////////////////////////

Road& PathFinder::getRoad()
{
	mymutex.lock();
	innerModel.lock();
	return road;	
}

void PathFinder::releaseRoad()
{
	innerModel.unlock();
	mymutex.unlock();
}



