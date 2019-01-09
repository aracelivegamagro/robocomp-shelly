/*		SOCIAL NAVIGATION NUEVO
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <math.h> 

#define PI M_PI

/**
 * \brief Default constructor
 */

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	qDebug() << __FUNCTION__ ;
	active = false;
	active = false;	
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	haveTarget = false;
	innerModel = std::make_shared<InnerModel>();



	// 	world = AGMModel::SPtr(new AGMModel());

	//Timed slot to read TrajectoryRobot2D state
	connect(&trajReader, SIGNAL(timeout()), &aE, SLOT(readTrajState()));
	connect(gaussiana,SIGNAL(clicked()),&socialrules, SLOT(calculateGauss()));
//	connect(por,SIGNAL(clicked()),&socialrules, SLOT(PassOnRight()));
	connect(objint,SIGNAL(clicked()),&socialrules, SLOT(objectInteraction()));
	connect(datos,SIGNAL(clicked()),&socialrules, SLOT(saveData()));
	
	connect(gotoperson,SIGNAL(clicked()),&socialrules, SLOT(goToPerson()));
	//trajReader.start(1000);
    connect(follow, SIGNAL (clicked()),&socialrules,SLOT(checkstate()));
    connect(accompany, SIGNAL (clicked()),&socialrules,SLOT(checkstate()));
    connect(por, SIGNAL (clicked()),&socialrules,SLOT(checkstate()));

    socialrules.idselected = idselect;
    socialrules.follow = follow;
    socialrules.accompany = accompany;
    socialrules.por = por;

}
/**
 * \brief Default destructor
 */
SpecificWorker::~SpecificWorker()
{
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList paramsL)
{

	//Extract robot name 
	try{ robotname = paramsL.at("RobotName").value;} 
	catch(const std::exception &e){ std::cout << e.what() << "SpecificWorker::SpecificWorker - Robot name defined in config. Using default 'robot' " << std::endl;}
	
	
	#ifdef USE_QTGUI
		viewer = std::make_shared<InnerViewer>(innerModel, "Social Navigation");  //InnerViewer copies internally innerModel so it has to be resynchronized
		//viewer->start();	
	#endif

	std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams = std::make_shared<RoboCompCommonBehavior::ParameterList>(paramsL);

//   	innerModel = std::make_shared<InnerModel>("/home/robocomp/robocomp/components/robocomp-araceli/etcSim/simulation.xml");
// 	innerModel->getNode<InnerModelJoint>("armX1")->setAngle(-1);
// 	innerModel->getNode<InnerModelJoint>("armX2")->setAngle(2.5);
	
	try
	{		
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		structuralChange(w);
		// Initializing PathFinder
		pathfinder.initialize(innerModel, viewer, configparams, laser_proxy, omnirobot_proxy);
		// Initializing SocialRules
		socialrules.initialize(socialnavigationgaussian_proxy, agmexecutive_proxy, mutex, &pathfinder, worldModel, innerModel);
	
		structuralChange(w);
		rDebug2(("Leaving Structural Change"));
	}		
	catch(...)
	{	rDebug2(("SetParams ---- The executive is probably not running, waiting for first AGM model publication...")); }

	// releasing pathfinder
	//thread_pathfinder = std::thread(&robocomp::pathfinder::PathFinder::run, &pathfinder);
	//rDebug2(("Pathfinder up and running"));


	qLog::getInstance()->setProxy("both", logger_proxy);
	rDebug2(("NavigationAgent started"));

	Period = 200;
	timer.start(Period);
	
	//Proxies for actionExecution
	//aE.logger_proxy = logger_proxy;
	//aE.agmexecutive_proxy = agmexecutive_proxy;
	//aE.omnirobot_proxy = omnirobot_proxy;
	//aE.trajectoryn2d_proxy = trajectoryrobot2d_proxy;
		
	rDebug2(("Leaving setParams"));
	
	return true;
}

/**
 * \brief Check if persons are included in the AGM. 
 * Then, their pose is stored.
 * Everytime a person has moved, its position is updated
 */
void SpecificWorker::compute() {

	static bool first = true;
	if (first) {
		qLog::getInstance()->setProxy("both", logger_proxy);
		rDebug2(("navigationAgent started"));
		first = false;
	}

//	// PROVISIONAL read robot position from proxy
//	try {
//
//		omnirobot_proxy->getBaseState(bState);
//		innerModel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
//		//	qDebug() << "SpecificWorker::compute" << bState.x << bState.z << bState.alpha;
//	}
//	catch (const Ice::Exception &ex) {
//		printf("Reading robot position-----The executive is probably not running, waiting for first AGM model publication...");
//		std::cout << ex << std::endl;
//	}

	pathfinder.run(); //projector is comentated because the social navigation cant acceded to the laser if the astraRGBD is working
	//update viewer
//	QVec robotpos = innerModel->transformS6D("world", robotname);
//	viewer->ts_updateTransformValues(QString::fromStdString(robotname), robotpos);
	viewer->run();


// 	qDebug()<<"Update actionEx";
// 	aE.Update(action,params);
// 	

    if (changepos) {
        socialrules.checkMovement();
//        socialrules.checkRobotmov();
        changepos = false;
	}

	if (!socialrules.pSymbolId.empty()) {
		checkHumanBlock();
	}

	first = false;

}


void SpecificWorker::checkHumanBlock()
{
    int robotID = -1;
    AGMModel::SPtr newModel(new AGMModel(worldModel));

    vector <int32_t> pId_blocking = pathfinder.getHumanBlocking();
	vector <int32_t> pId_softblocking = pathfinder.getHumanSoftBlocking();
    vector <int32_t> pId_affblocking = pathfinder.getAffordanceBlocking();

    if ((previous_blockinglist != pId_blocking) or (previous_softblockinglist != pId_softblocking) or (previous_affordanceslist != pId_affblocking))
    {
        try
        {
            robotID = worldModel->getIdentifierByType("robot");
        }

        catch(...)
        {
            std::cout<<__FUNCTION__<<" No robot found in AGM model"<<std::endl;
            return;
        }

        for (auto id1:previous_blockinglist)
        {
			try
			{
				newModel->removeEdgeByIdentifiers(id1, robotID, "block");
                qDebug ()<<"Se elimina el enlace block a la persona  " << id1;
			}

			catch(...)
			{
				std::cout<<__FUNCTION__<<"No existe el enlace"<<std::endl;

			}
        }

        for (auto id1:pId_blocking)
        {

			try
			{
				newModel->addEdgeByIdentifiers(id1, robotID, "block");
				qDebug ()<<"Se añade el enlace block a la persona  " << id1;
			}

			catch(...)
			{
				std::cout<<__FUNCTION__<<"No se puede añadir el enlace"<<std::endl;
			}

        }


		for (auto id1:previous_softblockinglist)
		{
			try
			{
				newModel->removeEdgeByIdentifiers(id1, robotID, "softBlock");
				qDebug ()<<"Se elimina el enlace softblock a la persona  " << id1;
			}

			catch(...)
			{
				std::cout<<__FUNCTION__<<"No existe el enlace"<<std::endl;

			}
		}

		for (auto id1:pId_softblocking)
		{

			try
			{
				newModel->addEdgeByIdentifiers(id1, robotID,  "softBlock");
				qDebug ()<<"Se añade el enlace softblock a la persona  " << id1;
			}

			catch(...)
			{
				std::cout<<__FUNCTION__<<"No se puede añadir el enlace"<<std::endl;
			}

		}

        for (auto id1:previous_affordanceslist)
        {
            try
            {
                newModel->removeEdgeByIdentifiers(id1, robotID, "affordanceBlock");
                qDebug ()<<"Se elimina el enlace affordanceBlock a la persona  " << id1;
            }

            catch(...)
            {
                std::cout<<__FUNCTION__<<"No existe el enlace"<<std::endl;

            }
        }

        for (auto id1:pId_affblocking)
        {

            try
            {
                newModel->addEdgeByIdentifiers(id1, robotID,  "affordanceBlock");
                qDebug ()<<"Se añade el enlace affordanceblock a la persona  " << id1;
            }

            catch(...)
            {
                std::cout<<__FUNCTION__<<"No se puede añadir el enlace"<<std::endl;
            }

        }

        try
        {
            sendModificationProposal(newModel,worldModel , "block");
        }
        catch(...)
        {
            std::cout<<"No se puede actualizar worldModel"<<std::endl;
        }
    }

    previous_blockinglist = pId_blocking;
	previous_softblockinglist = pId_softblocking;
	previous_affordanceslist = pId_affblocking;
}

float SpecificWorker::go(const TargetPose &target)
{
	if (target.doRotation)
	{
		pathfinder.go_rot(target.x, target.z, target.ry);
	}
	else
	{
		pathfinder.go(target.x, target.z);
	}
	return 0.0;
};

// *****************************************************************************************
// AGENT RELATED
// *****************************************************************************************

bool SpecificWorker::activateAgent(const ParameterMap& prs)
{
	bool activated = false;
	printf("<<activateAgent\n");
	if (setParametersAndPossibleActivation(prs, activated))
	{
		if (not activated)
		{
			printf("activateAgent 0 >>\n");
			return activate(p);
		}
	}
	else
	{
		printf("activateAgent 1 >>\n");
		return false;
	}
	printf("activateAgent 2 >>\n");
	return true;
}

bool SpecificWorker::deactivateAgent()
{
	return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
	StateStruct s;
	if (isActive())
	{
		s.state = Running;
	}
	else
	{
		s.state = Stopped;
	}
	s.info = p.action.name;
	return s;
}

ParameterMap SpecificWorker::getAgentParameters()
{
	return params;
}

bool SpecificWorker::setAgentParameters(const ParameterMap& prs)
{
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

void SpecificWorker::killAgent()
{
}

Ice::Int SpecificWorker::uptimeAgent()
{
	return 0;
}

bool SpecificWorker::reloadConfigAgent()
{
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World& modification)
{
	qDebug()<<"StructuralChange";
	QMutexLocker l(mutex);
	static bool first = true;
	
	AGMModelConverter::fromIceToInternal(modification, worldModel);
	InnerModel *inner = AGMInner::extractInnerModel(worldModel, "world", false);
	innerModel.reset(inner);
 

	if (!first)
	{
		socialrules.checkNewPersonInModel(worldModel);
		socialrules.innerModelChanged(innerModel);
	}
	else
		first = false;
	
	
	viewer->reloadInnerModel(innerModel);
	
	printf("FIN structuralChange>>\n");
}


void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node& modification)
{
	qDebug()<<"symbolUpdated";
	QMutexLocker l(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
	//qDebug()<<"symbolsUpdated";
	QMutexLocker l(mutex);

	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
}


void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
// 	qDebug()<<"edgesUpdated";
	changepos=true;

	QMutexLocker lockIM(mutex);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMModelEdge edge;
		AGMModelConverter::fromIceToInternal(modification, edge);
//TODO Guardar si el cambio se refiere a una persona?¿
		//Update InnerModel values
		if (edge->getLabel()=="RT" )
		{
			try{
				std::string songName= (worldModel->getSymbol( edge->getSymbolPair().second) )->getAttribute("imName");
				
				QVec vec = QVec::vec6();
				vec[0] = str2float(edge->getAttribute("tx"));
				vec[1] = str2float(edge->getAttribute("ty"));
				vec[2] = str2float(edge->getAttribute("tz"));
				vec[3] = str2float(edge->getAttribute("rx"));
				vec[4] = str2float(edge->getAttribute("ry"));
				vec[5] = str2float(edge->getAttribute("rz"));
				innerModel->updateTransformValues(QString::fromStdString(songName), vec);
				viewer->ts_updateTransformValues(QString::fromStdString(songName), vec);
			}
			catch (...)
			{
				qDebug()<<"EXCEPTION,RT label connect to a symbol without imName\n";
				std::cout<<(worldModel->getSymbol( edge->getSymbolPair().second))->toString(true);
			}
		}
	}
}

/**
 * \brief ACTUALIZACION DEL ENLACE EN INNERMODEL
 */ 
void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge& modification)
{
 	qDebug() << "edgeUpdated";
	changepos = true;
	QMutexLocker lockIM(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMModelEdge edge;
	AGMModelConverter::fromIceToInternal(modification, edge);
//TODO Guardar si el cambio se refiere a una persona?¿
	//Update InnerModel values
	if (edge->getLabel()=="RT" )
	{
		try{
			std::string songName= (worldModel->getSymbol( edge->getSymbolPair().second) )->getAttribute("imName");
			
			QVec vec = QVec::vec6();
			vec[0] = str2float(edge->getAttribute("tx"));
			vec[1] = str2float(edge->getAttribute("ty"));
			vec[2] = str2float(edge->getAttribute("tz"));
			vec[3] = str2float(edge->getAttribute("rx"));
			vec[4] = str2float(edge->getAttribute("ry"));
			vec[5] = str2float(edge->getAttribute("rz"));
			innerModel->updateTransformValues(QString::fromStdString(songName), vec);
			viewer->ts_updateTransformValues(QString::fromStdString(songName), vec);
		}
		catch (...)
		{
			qDebug()<<"EXCEPTION,RT label connect to a symbol without imName\n";
			std::cout<<(worldModel->getSymbol( edge->getSymbolPair().second))->toString(true);
		}
	}
}

bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		printf("param:%s   value:%s\n", it->first.c_str(), it->second.value.c_str());
		params[it->first] = it->second;
	}
	printf("----\n");

	try
	{
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);

		if (action == "graspobject")
		{
			active = true;
		}
		else
		{
			active = true;
		}
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\n");

	return true;
}


void SpecificWorker::sendModificationProposal(AGMModel::SPtr &newModel, AGMModel::SPtr &worldModel, string m)
{
	QMutexLocker locker(mutex);
	
	try
	{
		AGMMisc::publishModification(newModel, agmexecutive_proxy, std::string( "SocialnavigationAgent")+m);
	}
	catch(const RoboCompAGMExecutive::Locked &e)
	{
	}
	catch(const RoboCompAGMExecutive::OldModel &e)
	{
		printf("modelo viejo\n");
	}
	catch(const RoboCompAGMExecutive::InvalidChange &e)
	{
		printf("modelo invalido\n");
	}
	catch(const Ice::Exception& e)
	{
		exit(1);
	}
}
