/*
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
/** \mainpage RoboComp::genericComp
 *
 * \section intro_sec Introduction
 *
 * The genericComp component...
 *
 * \section interface_sec Interface
 *
 * genericComp interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * genericComp ...
 *
 * \subsection install2_ssec Compile and install
 * cd genericComp
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file genericComp/etc/specific_config and genericComp/etc/generic_config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/genericComp --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
// QT includes
#include <QtCore>
#include <QtGui>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>
#include <qlog/qlog.h>
// View the config.h file for config options like
// QtGui, etc...
#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"
#include <bodyinversekinematicsI.h>
#include <joystickadapterI.h>

// Includes for remote proxy example
// #include <Remote.h>
#include <ui_guiDlg.h>
#include <InnerModelManager.h>
#include <JointMotor.h>
#include <OmniRobot.h>


// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;
using namespace RoboCompBodyInverseKinematics;
using namespace RoboCompJoystickAdapter;
using namespace RoboCompInnerModelManager;
using namespace RoboCompJointMotor;
using namespace RoboCompOmniRobot;


class BIK : public RoboComp::Application
{
private:
	// User private data here

	void initialize();
	MapPrx mprx;

public:
	virtual int run(int, char*[]);
};

void BIK::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int BIK::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif
	int status=EXIT_SUCCESS;

	// Remote server proxy access example
	// RemoteComponentPrx remotecomponent_proxy;
	InnerModelManagerPrx innermodelmanager_proxy;
JointMotorPrx jointmotor0_proxy;
JointMotorPrx jointmotor1_proxy;
OmniRobotPrx omnirobot_proxy;


	string proxy;

	// User variables


	initialize();

	// Remote server proxy creation example
	// try
	// {
	// 	// Load the remote server proxy
	//	proxy = getProxyString("RemoteProxy");
	//	remotecomponent_proxy = RemotePrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	//	if( !remotecomponent_proxy )
	//	{
	//		rInfo(QString("Error loading proxy!"));
	//		return EXIT_FAILURE;
	//	}
	//catch(const Ice::Exception& ex)
	//{
	//	cout << "[" << PROGRAM_NAME << "]: Exception: " << ex << endl;
	//	return EXIT_FAILURE;
	//}
	//rInfo("RemoteProxy initialized Ok!");
	// 	// Now you can use remote server proxy (remotecomponent_proxy) as local object
	//Remote server proxy creation example
	try
	{
		innermodelmanager_proxy = InnerModelManagerPrx::uncheckedCast( communicator()->stringToProxy( getProxyString("InnerModelManagerProxy") ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("InnerModelManagerProxy initialized Ok!");
	mprx["InnerModelManagerProxy"] = (::IceProxy::Ice::Object*)(&innermodelmanager_proxy);//Remote server proxy creation example
	try
	{
		jointmotor0_proxy = JointMotorPrx::uncheckedCast( communicator()->stringToProxy( getProxyString("JointMotor0Proxy") ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("JointMotor0Proxy initialized Ok!");
	mprx["JointMotor0Proxy"] = (::IceProxy::Ice::Object*)(&jointmotor0_proxy);//Remote server proxy creation example
	try
	{
		jointmotor1_proxy = JointMotorPrx::uncheckedCast( communicator()->stringToProxy( getProxyString("JointMotor1Proxy") ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("JointMotor1Proxy initialized Ok!");
	mprx["JointMotor1Proxy"] = (::IceProxy::Ice::Object*)(&jointmotor1_proxy);//Remote server proxy creation example
	try
	{
		omnirobot_proxy = OmniRobotPrx::uncheckedCast( communicator()->stringToProxy( getProxyString("OmniRobotProxy") ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("OmniRobotProxy initialized Ok!");
	mprx["OmniRobotProxy"] = (::IceProxy::Ice::Object*)(&omnirobot_proxy);
	IceStorm::TopicManagerPrx topicManager = IceStorm::TopicManagerPrx::checkedCast(communicator()->propertyToProxy("TopicManager.Proxy"));
	
	
	GenericWorker *worker = new SpecificWorker(mprx);
	//Monitor thread
	GenericMonitor *monitor = new SpecificMonitor(worker,communicator());
	QObject::connect(monitor,SIGNAL(kill()),&a,SLOT(quit()));
	QObject::connect(worker,SIGNAL(kill()),&a,SLOT(quit()));
	monitor->start();
	
	if ( !monitor->isRunning() )
		return status;
	try
	{
		// Server adapter creation and publication
		Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapter("CommonBehavior");
		CommonBehaviorI *commonbehaviorI = new CommonBehaviorI(monitor );
		adapterCommonBehavior->add(commonbehaviorI, communicator()->stringToIdentity("commonbehavior"));
		adapterCommonBehavior->activate();
		// Server adapter creation and publication
    	Ice::ObjectAdapterPtr JoystickAdapter_adapter = communicator()->createObjectAdapter("JoystickAdapterTopic");
    	JoystickAdapterPtr joystickadapterI_ = new JoystickAdapterI(worker);
    	Ice::ObjectPrx joystickadapter_proxy = JoystickAdapter_adapter->addWithUUID(joystickadapterI_)->ice_oneway();
    	IceStorm::TopicPrx joystickadapter_topic;
    	if(!joystickadapter_topic){
	    	try {
	    		joystickadapter_topic = topicManager->create("JoystickAdapter");
	    	}
	    	catch (const IceStorm::TopicExists&) {
	    	  	//Another client created the topic
	    	  	try{
	       			joystickadapter_topic = topicManager->retrieve("JoystickAdapter");
	    	  	}catch(const IceStorm::NoSuchTopic&){
	    	  	  	//Error. Topic does not exist
				}
	    	}
	    	IceStorm::QoS qos;
	      	joystickadapter_topic->subscribeAndGetPublisher(qos, joystickadapter_proxy);
    	}
    	JoystickAdapter_adapter->activate();
    	// Server adapter creation and publication
		Ice::ObjectAdapterPtr adapterBodyInverseKinematics = communicator()->createObjectAdapter("BodyInverseKinematicsComp");
		BodyInverseKinematicsI *bodyinversekinematics = new BodyInverseKinematicsI(worker);
		adapterBodyInverseKinematics->add(bodyinversekinematics, communicator()->stringToIdentity("bodyinversekinematics"));

		adapterBodyInverseKinematics->activate();
		cout << SERVER_FULL_NAME " started" << endl;

		// User defined QtGui elements ( main window, dialogs, etc )

#ifdef USE_QTGUI
		//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
		a.setQuitOnLastWindowClosed( true );
#endif
		// Run QT Application Event Loop
		a.exec();
		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;

#ifdef USE_QTGUI
		a.quit();
#endif
		monitor->exit(0);
}

	return status;
}

int main(int argc, char* argv[])
{
	bool hasConfig = false;
	string arg;
	BIK app;

	// Search in argument list for --Ice.Config= argument
	for (int i = 1; i < argc; ++i)
	{
		arg = argv[i];
		if ( arg.find ( "--Ice.Config=", 0 ) != string::npos )
			hasConfig = true;
	}

	if ( hasConfig )
		return app.main( argc, argv );
	else
		return app.main(argc, argv, "../etc/generic_config"); // "config" is the default config file name
}