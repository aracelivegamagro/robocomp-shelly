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
#define REDUCTORA 246
#define ENCODER   1024

#define V_0_GRADOS_M1 1805
#define V_180_GRADOS_M1 8696
#define OFFSET_M1 -200			// AJUSTE DEL CENTRO

#define V_0_GRADOS_M2 8500
#define V_180_GRADOS_M2 1924
#define OFFSET_M2 -100			// AJUSTE DEL CENTRO

#define V__90_GRADOS_M3 1899
#define V_0_GRADOS_M3   4737
#define V_90_GRADOS_M3  8722
#define OFFSET_M3 330			// AJUSTE DEL CENTRO

#define V__90_GRADOS_M4 1692
#define V_0_GRADOS_M4   4951
#define V_90_GRADOS_M4  8253
#define OFFSET_M4 160.			// AJUSTE DEL CENTRO

#define V_0_GRADOS_M5 1557
#define V_150_GRADOS_M5 7168
#define OFFSET_M5 0			// AJUSTE DEL CENTRO

#define V_0_GRADOS_M11 8547
#define V_180_GRADOS_M11 1645
#define OFFSET_M11 0			// AJUSTE DEL CENTRO

#define V_0_GRADOS_M12 7793
#define V_180_GRADOS_M12 1038
#define OFFSET_M12 -100			// AJUSTE DEL CENTRO

#define V__90_GRADOS_M13 1677
#define V_0_GRADOS_M13   5016
#define V_90_GRADOS_M13  8571
#define OFFSET_M13 -100  		// AJUSTE DEL CENTRO

#define V__90_GRADOS_M14 1410
//#define V__90_GRADOS_M14 2725
#define V_0_GRADOS_M14   4725
#define V_90_GRADOS_M14  7593
//#define V_90_GRADOS_M14  6725
#define OFFSET_M14 200			// AJUSTE DEL CENTRO

#define V_0_GRADOS_M15 6500
#define V_150_GRADOS_M15 1469
#define OFFSET_M15 0			// AJUSTE DEL CENTRO

#define PASOS_90_GRADOS  62976
#define PASOS_180_GRADOS 125952
#define PASOS_150_GRADOS 105000


// #define FaulHaberDebug 

 #include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx)
{
	memory_mutex = new QMutex();
        write_memory_mutex = new QMutex();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::initializeMotors()
{
	///Create servos instances in a QMap indexed by name
	std::cout << "JointMotor::FaulHaber::FaulHaber - Motor Map creation with " << busParams.numMotors << " motors: " << std::endl;
	for (int i = 0; i < busParams.numMotors; i++)
	{
		QString name = QString::fromStdString(motorParamsList.operator[](i).name);
		motorsName[name] = new Servo( motorParamsList.operator[](i) );
		motorsId[motorParamsList.operator[](i).busId] = motorsName[name];
		
		//steps_range , max_degrees, steps_speed_range, max_speed_rads  ¿¿Deberían venir en los parámetros?
		motorsName[name]->setMotorRanges(motorParamsList.operator[](i).stepsRange, motorParamsList.operator[](i).maxDegrees, 0, 0.f);  
		qDebug()<<"\t" << name ; 
	}

	
	sleep(1);
	///Initialize motor params
	foreach( Servo *s, motorsName )
	{
		RoboCompJointMotor::MotorParams &params = s->params;
		std::cout << "JointMotor::FaulHaber::FaulHaber - Configuration data of motor " << params.name  << std::endl;
		qDebug() << "	" << "busId" << params.busId;
		
		faulhaber->Init_Node(params.busId);
		rInfo("Init node");
		///Change Mode		
		faulhaber->enableControlPositionMode(params.busId);
		rInfo("enable control");
		faulhaber->enableCommandMode(params.busId);
		rInfo("enable command");
		
		int ids[1];
		ids[0]=params.busId;
		int positions[1];
		
		cout<<"getPosition: "<<params.busId<<" "<<faulhaber->getPosition(params.busId)<<endl;
		cout<<"getPositionExternalEncoder: "<<params.busId<<" "<<faulhaber->getPositionExternalEncoder(params.busId)<<endl;
		faulhaber->syncGetPosition(1,&ids[0],&positions[0]);
		cout<<"syncGetPosition: "<<params.busId<<" "<<positions[0]<<endl;
		
		//Initialize motors internal encoders respect to external encoders
		long total=0;
		for(int i=0; i<40; i++)
		{
			total += faulhaber->getPositionExternalEncoder(params.busId);
		}
		int average = total/40;
		
		cout<<"Esta son las cuentas: "<<average<<" "<<External_to_internal_encoder(average,params.busId)<<endl;
		faulhaber->setInternalEncoderValue(params.busId, External_to_internal_encoder(average,params.busId));
		cout<<"getPosition after las cuentas: "<<params.busId<<" "<<faulhaber->getPosition(params.busId)<<endl;
		RoboCompJointMotor::MotorGoalPosition goalPosition;
		goalPosition.position=0;
		goalPosition.maxSpeed=1;
		goalPosition.name=params.name;
// 		setPosition(goalPosition);
	}
	

	
// 	
	
	
}

int SpecificWorker::External_to_internal_encoder(int valor_potenciometro,int ID)
{
  
  int resultado=0;
  
  switch (ID){
    
    case 1: {
        resultado=  -(((((valor_potenciometro-(V_0_GRADOS_M1 + OFFSET_M1))*180)/+((V_180_GRADOS_M1 + OFFSET_M1)-(V_0_GRADOS_M1 + OFFSET_M1)))*PASOS_180_GRADOS)/180);
      break;
    }
    case 2: {
	resultado=  (((((valor_potenciometro-(V_0_GRADOS_M2 + OFFSET_M2))*180)/+((V_180_GRADOS_M2 + OFFSET_M2) - (V_0_GRADOS_M2 + OFFSET_M2)))*PASOS_180_GRADOS)/180);
      break;
    }
    case 3: {
	resultado=  ((((((valor_potenciometro- (V__90_GRADOS_M3 + OFFSET_M3))*180)/+((V_90_GRADOS_M3 + OFFSET_M3)-(V__90_GRADOS_M3 + OFFSET_M3)))*(PASOS_180_GRADOS))/180)-PASOS_90_GRADOS);
      break;
    }
    case 4: {
	resultado=  ((((((valor_potenciometro- (V__90_GRADOS_M4 + OFFSET_M4))*180)/+((V_90_GRADOS_M4 + OFFSET_M4)-(V__90_GRADOS_M4 + OFFSET_M4)))*(PASOS_180_GRADOS))/180)-PASOS_90_GRADOS);      
      break;
    }
    case 5: {
	resultado= (((((valor_potenciometro - (V_0_GRADOS_M5 + OFFSET_M5))*150)/+((V_150_GRADOS_M5 + OFFSET_M5)-(V_0_GRADOS_M5 + OFFSET_M5)))*(PASOS_150_GRADOS)*(-1))/150);
      break;
    }
    case 11: {
	resultado= (((((valor_potenciometro-(V_0_GRADOS_M11 + OFFSET_M11))*180)/+((V_180_GRADOS_M11 + OFFSET_M11)-(V_0_GRADOS_M11 + OFFSET_M11)))*PASOS_180_GRADOS)/180);
      break;
    }
    case 12: {
	resultado= (((((valor_potenciometro-(V_0_GRADOS_M12 + OFFSET_M12))*180)/+((V_180_GRADOS_M12 + OFFSET_M12)-(V_0_GRADOS_M12 + OFFSET_M12)))*PASOS_180_GRADOS)/180);
      break;
    }
    case 13: {
	resultado= ((((((valor_potenciometro-(V__90_GRADOS_M13 + OFFSET_M13))*180)/+((V_90_GRADOS_M13 + OFFSET_M13)-(V__90_GRADOS_M13 + OFFSET_M13)))*(PASOS_180_GRADOS))/180)-PASOS_90_GRADOS);
      break;
    }
    case 14: {
	resultado=  ((((((valor_potenciometro- (V__90_GRADOS_M14 + OFFSET_M14))*180)/+((V_90_GRADOS_M14 + OFFSET_M14)-(V__90_GRADOS_M14 + OFFSET_M14)))*(PASOS_180_GRADOS))/180)-PASOS_90_GRADOS);      
      break;
    }
    case 15: {
	resultado= (((((valor_potenciometro-(V_0_GRADOS_M15 + OFFSET_M15))*150)/+((V_150_GRADOS_M15 + OFFSET_M15)-(V_0_GRADOS_M15+OFFSET_M15)))*PASOS_150_GRADOS)/150);
      break;
    }

  }
    return resultado; 
}
	

void SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	busParams.device = params["Faulhaber.Device"].value;
	busParams.numMotors = QString::fromStdString(params["Faulhaber.NumMotors"].value).toInt();
	busParams.baudRate = QString::fromStdString(params["Faulhaber.BaudRate"].value).toInt();
	busParams.basicPeriod = QString::fromStdString(params["Faulhaber.BasicPeriod"].value).toInt();
	Period = busParams.basicPeriod *1000;
	
	for (int i=0; i<busParams.numMotors; i++)
	{
		RoboCompJointMotor::MotorParams mpar;	
		std::string s= QString::number(i).toStdString();
		mpar.name = params["Faulhaber.Params_" + s +".name"].value;
		mpar.busId = QString::fromStdString(params["Faulhaber.Params_" + s +".busId"].value).toUShort();
		mpar.minPos = QString::fromStdString(params["Faulhaber.Params_" + s +".minPos"].value).toFloat();
		mpar.maxPos = QString::fromStdString(params["Faulhaber.Params_" + s +".maxPos"].value).toFloat();
		mpar.zeroPos = QString::fromStdString(params["Faulhaber.Params_" + s +".zeroPos"].value).toFloat();
		mpar.maxVelocity = QString::fromStdString(params["Faulhaber.Params_" + s +".maxVelocity"].value).toFloat();
		mpar.stepsRange = QString::fromStdString(params["Faulhaber.Params_" + s +".stepsRange"].value).toFloat();
		mpar.unitsRange = QString::fromStdString(params["Faulhaber.Params_" + s +".unitsRange"].value).toFloat();
		mpar.offset = QString::fromStdString(params["Faulhaber.Params_" + s +".offset"].value).toFloat();
		motorParamsList.push_back(mpar);
		mParams[QString::fromStdString(mpar.name)] = mpar;
		name2id[QString::fromStdString(mpar.name)] = mpar.busId;
	}
	cout<<"mparams: "<<motorParamsList[name2id["leftShoulder2"]].stepsRange<<endl;
	
	
	//Creacion de faulhaberApi
	qDebug() << "Opening CAN device" << QString::fromStdString(busParams.device);
	faulhaber = new FaulHaberApi(QString::fromStdString(busParams.device), busParams.baudRate);
	initializeMotors();
	qDebug()<<"basicperiod"<<Period;

	//uint value=0;
	//int id=12;
	//faulhaber->GADV(id,value);
	//int encoderPosition=faulhaber->Get_Pos_Encoder(id,value);
	//qDebug()<<id;
	//qDebug()<<"potenciometro:"<<value;
	//qDebug()<<"value to encoder"<<encoderPosition;
	//int ids[1];
	//ids[0]=12;
	//int positions[1];

	//faulhaber->setVelocity(12,10);
	//faulhaber->setPosition(12, 10000);
	
	
	//faulhaber->HO (id,encoderPosition);
	//qDebug()<<id;
	//qDebug()<<"potenciometro:"<<value;
	//qDebug()<<"value to encoder"<<encoderPosition;
	//qDebug()<<"position:"<<faulhaber->getPositionUrsus(id);
	//qDebug()<<".......";
	
 	//faulhaber->setPosition(id,faulhaber->getPositionUrsus(id)+10000);
	sleep(5);
	//faulhaber->GADV(id,value);	
	//encoderPosition=faulhaber->Get_Pos_Encoder(id,value);
	//qDebug()<<id<<"final";
	//qDebug()<<"potenciometro:"<<value;
	//qDebug()<<"value to encoder"<<encoderPosition;
	//qDebug()<<"position:"<<faulhaber->getPositionUrsus(id);
	//qDebug()<<".......";
 	start();
	
}

///
/// Servant methods
///

void SpecificWorker::run( )
{
// 	int id=13;
	//uint value;
	//qDebug()<<id<<"faulhaber->GADV"<<id<<faulhaber->GADV(id,value)<<value;
	//return;
	
	QVector <int> goal_ids;
	QVector <int> goal_pos;
	//int positions[busParams.numMotors];
	int ids[busParams.numMotors];
	QList <QString> names;
	float aux_rad;
	//int positions_d[busParams.numMotors];
	for(int i=0;i<busParams.numMotors;i++)
	{
		ids[i] = motorParamsList[i].busId;
		
		QString name = QString::fromStdString(motorParamsList[i].name);
		names.push_back(name);
	}
	
	forever {
#ifdef FaulHaberDebug
		cout<<"-----------------*************---------------"<<endl;
#endif		
                // read motor positions
		for(int i=0;i<busParams.numMotors;i++)
		{
                        mutex->lock();
			//qDebug()<<ids[i];
                            int32_t pos1 = faulhaber->getPosition(ids[i]);
                        mutex->unlock();
			if (pos1 == -1)
				continue;
			float pos = pos1;
			QString  name =names.at(i);
			///todo poner define para debug
#ifdef FaulHaberDebug 		
			qDebug()<<"getPosition: "<<names.at(i)<<"id: "<<ids[i]<<"pos"<<pos;
			qDebug()<<"steps2units: "<<faulhaber->steps2units(pos,mParams[name].stepsRange, mParams[name].unitsRange, mParams[name].offset);			
			qDebug()<<"----";
#endif
			aux_rad = faulhaber->steps2units(pos,mParams[name].stepsRange, mParams[name].unitsRange, mParams[name].offset);
			// check data reading error
			if (fabs(aux_rad) < 3.15){
				memory_mutex->lock(); 
					Servo::TMotorData &data = motorsId[ids[i]]->data;
					data.currentPosRads = aux_rad;
					data.isMoving = fabs(data.antPosRads - data.currentPosRads) > 0.01;
					data.antPosRads = data.currentPosRads;
				memory_mutex->unlock();
			}
		}
		
		// set motor positions
		write_memory_mutex->lock();
			
			while (!goalPosList_memory.isEmpty()){
				MotorGoalPosition goal = goalPosList_memory.pop();
				int motor_id = name2id[QString::fromStdString(goal.name)];
				if (not goal_ids.contains(motor_id)) // keep just last position for each motor
				{
					goal_ids.append(motor_id);
					goal_pos.append(goal.position);
				}
			}
			if (goal_ids.size())
			{
				mutex->lock();
#ifdef FaulHaberDebug
					qDebug()<<"setting real motor positions"<<goal_ids<<goal_pos;
#endif
					faulhaber->syncSetPosition(goal_ids.size(),&goal_ids.toStdVector()[0],&goal_pos.toStdVector()[0]);
					goal_ids.clear();
					goal_pos.clear();
				mutex->unlock();
			}
		write_memory_mutex->unlock();
                
	/*		
		faulhaber->syncGetPosition(busParams.numMotors,&ids[0],&positions[0]);
		qDebug() << "positions" << positions[0] << positions[1] << positions[2] << positions[3];
		
		for(int i=0;i < busParams.numMotors;i++){
			positions_d[i] = positions[i] + 1000;
			if(positions_d[i]>5000)
			  positions_d[i]=0;
		}
		//faulhaber->syncSetPosition(busParams.numMotors,&ids[0],&positions_d[0]);
		qDebug() << "positions_d" << positions_d[0] << positions_d[1] << positions_d[2] << positions_d[3];
		sleep(4);*/
		
		
		
		usleep(Period);
	}
}

void SpecificWorker::setPosition(const MotorGoalPosition& goalPosition)
{
	QString name = QString::fromStdString(goalPosition.name);
	qDebug()<<"Goal position:\n"<<"Motor:"<<name<<goalPosition.position;
	if( mParams.contains( name ) )
	{
		float position = truncatePosition(name,goalPosition.position);
		int pInt = faulhaber->units2steps(position, mParams[name].stepsRange, mParams[name].unitsRange, mParams[name].offset);

		//qDebug()<<"saving position"<<pInt<<"of motor"<<name<<busId;
		write_memory_mutex->lock();
			MotorGoalPosition goal;
			goal.name = goalPosition.name;
			goal.maxSpeed = 10;
			goal.position = pInt;
			goalPosList_memory.push(goal);
		write_memory_mutex->unlock();
	}
	else
	{
		uFailed.what = std::string("Exception: FaulhaberComp::setPosition:: Unknown motor name ") + goalPosition.name;
		throw uFailed;
	}
	//qDebug()<<"done:setting position of motor"<<name;
	
}


void SpecificWorker::setSyncPosition(const MotorGoalPositionList& goalPosList)
{
	bool correct = true;
	QStack<MotorGoalPosition> aux_q;
	qDebug()<<"Goal positions:";
	for(uint i=0;i<goalPosList.size();i++)
	{
		QString name = QString::fromStdString(goalPosList[i].name);
		qDebug()<<"Motor: "<<name<<goalPosList[i].position;
		if ( !motorsName.contains(name))
		{
			correct = false;
			break;
		}
		else
		{
			MotorGoalPosition goal;
			goal.name = goalPosList[i].name;
			goal.maxSpeed = 10;
			float position = truncatePosition(name,goalPosList[i].position);
			goal.position = faulhaber->units2steps(position, mParams[name].stepsRange, mParams[name].unitsRange, mParams[name].offset);
			aux_q.push(goal);
		}
	}
	
	if(correct)
	{
		write_memory_mutex->lock();
			goalPosList_memory += aux_q;
		write_memory_mutex->unlock();
	}
	else
	{
		uFailed.what = std::string("Exception: FaulhaberComp::setSyncPosition:: Unknown motor name");
		throw uFailed;
	}
	//qDebug()<<"done:setting sync position of motors";
}

MotorParams SpecificWorker::getMotorParams(const std::string& motor)
{
	RoboCompJointMotor::MotorParams mp;
	QString name = QString::fromStdString(motor);
	if ( mParams.contains( name ) )
	{
		mp = mParams[name];
	}
	else
	{
		uFailed.what = std::string("Exception: FaulhaberComp::getMotorParams:: Unknown motor name") + motor;
		throw uFailed;
	}
	return mp;
}
MotorState SpecificWorker::getMotorState(const std::string& motor)
{
	RoboCompJointMotor::MotorState state;
	QString name = QString::fromStdString(motor);
	if ( mParams.contains( name ) )
	{
		memory_mutex->lock();
		state.pos = motorsName[name]->data.currentPosRads;
		state.v = motorsName[name]->data.currentVelocityRads;
		state.p = motorsName[name]->data.currentPos;
		state.temperature = motorsName[name]->data.temperature;
		state.isMoving = motorsName[name]->data.isMoving;
		state.vel = motorsName[name]->data.currentVelocityRads;
		memory_mutex->unlock();
	}
	else
	{
		uFailed.what = std::string("Exception: FaulhaberComp::getMotorState:: Unknown motor name") + motor;
		throw uFailed;
	}
	return state;
}
void SpecificWorker::getAllMotorState(MotorStateMap& mstateMap)
{
	RoboCompJointMotor::MotorState state;
	memory_mutex->lock();
	foreach( Servo *s, motorsName)
	{
		state.pos = s->data.currentPosRads;
		
//		state.v = s->data.currentVelocityRads;
		state.p = s->data.currentPos;
		state.isMoving = s->data.isMoving;
//		state.temperature = s->data.temperature;
		mstateMap[s->params.name] = state;
	}
	memory_mutex->unlock();
}
MotorParamsList SpecificWorker::getAllMotorParams()
{
	return motorParamsList;
}
BusParams SpecificWorker::getBusParams()
{
	return busParams;
}

MotorStateMap SpecificWorker::getMotorStateMap(const MotorList& motorList)
{
	RoboCompJointMotor::MotorStateMap stateMap;
	RoboCompJointMotor::MotorState state;
	foreach(std::string motor, motorList)
	{
		
		QString name = QString::fromStdString(motor);
		if ( mParams.contains( name ) )
		{
			memory_mutex->lock();
			state.pos = motorsName[name]->data.currentPosRads;
//			state.v = motorsName[name]->data.currentVelocityRads;
			state.p = motorsName[name]->data.currentPos;
//			state.temperature = motorsName[name]->data.temperature;
			state.isMoving = motorsName[name]->data.isMoving;
//			state.vel = motorsName[name]->data.currentVelocityRads;
			stateMap[motor] = state;
			memory_mutex->unlock();

		}
		else
		{
			uFailed.what = std::string("Exception: FaulhaberComp::getMotorParams:: Unknown motor name") + motor;
			printf("##### %s\n", uFailed.what.c_str());
			throw uFailed;
		}
	}
	return stateMap;
}


void SpecificWorker::setZeroPos(const std::string& name)
{
	RoboCompJointMotor::UnknownMotorException ex;
	ex.what = std::string("Exception: Not implemented yet:");
	throw ex; 
}

void SpecificWorker::setSyncZeroPos()
{
	RoboCompJointMotor::UnknownMotorException ex;
	ex.what = std::string("Exception: Not implemented yet:");
	throw ex; 
}

void SpecificWorker::setVelocity(const MotorGoalVelocity& goalVelocity)
{
	RoboCompJointMotor::UnknownMotorException ex;
	ex.what = std::string("Exception: Not implemented yet:");
	throw ex; 
}
void SpecificWorker::setSyncVelocity(const MotorGoalVelocityList& listGoals)
{
	RoboCompJointMotor::UnknownMotorException ex;
	ex.what = std::string("Exception: Not implemented yet:");
	throw ex; 
}

//Comprueba la posicion pasada con los rangos máximos de movimiento y trunca el valor en caso necesario.
float SpecificWorker::truncatePosition(QString name,float position)
{
	float p = position;
	if(position > mParams[name].maxPos )
		p = mParams[name].maxPos;
	else if( position < mParams[name].minPos )
		p = mParams[name].minPos;
	return p;
}



///****************************************************************************************
///****************************************************************************************
	///relative to Muecas' Head
	//send all motor to home position (end of track)
// 	rInfo("Sending home");
// 	foreach( Servo *s, motorsName )
// 	{
// 		RoboCompJointMotor::MotorParams &params = s->params;
// 		//rInfo("Sending home");
// 		faulhaber->goHome(params.busId);
// 		if(params.busId != 3)
// 			faulhaber->goHome(params.busId);
// 	}
// 	rInfo("waiting");
	
//	sleep(3);
// 	rInfo("salgo");
	
	
// 	int positions[busParams.numMotors];
// 	int ids[busParams.numMotors];
// 	
// 	for(int i=0;i<busParams.numMotors;i++)
// 	  ids[i] = motorParamsList[i].busId;
	
//	int nohome,nohome_one;	
//	do{
//	  nohome = 0;
//	  for(int i=0;i<busParams.numMotors;i++){
//	    positions[i] = faulhaber->getPosition(ids[i]); 
// 	    if (positions[i]>3)
// 	      nohome_one = 1;
// 	    else
// 	      nohome_one = 0;
// 	    nohome = nohome + nohome_one;
// 	    usleep(Period*10);
// 	  }
// 	  qDebug() << "positions" << positions[0] << positions[1] << positions[2] << positions[3] << nohome;
// 	} while (nohome>0);

// 	  sleep(1);
// 	  //read current positions
// 	  for(int i=0;i<busParams.numMotors;i++)
// 	    positions[i] = faulhaber->getPosition(ids[i]); 
// 
// 	  int cuenta=0;
// 	  while  ((positions[0]< -1) || (positions[1]< -1) || (positions[2]< -1) || (positions[3]< -30) && (cuenta<30))
// 	  {
// 	    for(int i=0;i<busParams.numMotors;i++)
// 	      positions[i] = faulhaber->getPosition(ids[i]); 
// 	    usleep(500000);
// 	    cuenta++;
// 	    qDebug() << "positions" << positions[0] << positions[1] << positions[2] << positions[3];
// 	  }
// 	  qDebug() << "Cuenta:" << cuenta;
// 
// 	  if (cuenta>=30)
// 	    qDebug() << "ERROR: no he llegado al home";
// 	  //read all motor positions
// // 	  faulhaber->syncGetPosition(busParams.numMotors,&ids[0],&positions[0]);
// // 	  rInfo("syncGetPosition");
// // 	  qDebug() << "positions" << positions[0] << positions[1] << positions[2] << positions[3];
// 
//   	  //qDebug() << faulhaber->getPosition(21) << faulhaber->getPosition(22) << faulhaber->getPosition(23)  << faulhaber->getPosition(20) ;
// 
// 
// 
// 	  qDebug() << "positions" << positions[0] << positions[1] << positions[2] << positions[3];
// 	  sleep(1);
// 	
// 	
// 	rInfo("In home");
	
	  
// 	//send all motor to zero position
// 	for(int i=0;i < busParams.numMotors;i++)
// 		positions[i] = motorParamsList[i].zeroPos;
// 	faulhaber->syncSetPosition(busParams.numMotors,&ids[0],&positions[0]);
// 	sleep(4);
	
// 	foreach( Servo *s, motorsName )
// 	{
// 		RoboCompJointMotor::MotorParams &params = s->params;
// 		faulhaber->setZero(params.busId);
// 		params.zeroPos = 0.f;
// 	}
// 	qDebug()<<"fin initialize";

	//faulhaber->syncGetPosition(busParams.numMotors,&ids[0],&positions[0]);
        //rInfo("syncGetPosition");
// 	qDebug() << "positions" << positions[0] << positions[1] << positions[2] << positions[3];
