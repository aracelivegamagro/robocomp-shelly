
#include "socialrules.h"

#define PI M_PI

void SocialRules::initialize(SocialNavigationGaussianPrx socialnavigationgaussian_proxy_,
			     AGMExecutivePrx agmexecutive_proxy_,
			     QMutex *mutex_,
			     robocomp::pathfinder::PathFinder *pathfinder_,
			     AGMModel::SPtr worldModel_,
			     const std::shared_ptr<InnerModel> &innerModel_)

{
	qDebug()<<__FUNCTION__;
	
	socialnavigationgaussian_proxy = socialnavigationgaussian_proxy_;
	agmexecutive_proxy = agmexecutive_proxy_;
	mux = mutex_;
	pathfinder = pathfinder_;
	innerModel = innerModel_;
	worldModel = worldModel_;

	objectInteraction(false);
}

void SocialRules::innerModelChanged(const std::shared_ptr<InnerModel> &innerModel_)
{
	qDebug()<<__FUNCTION__;
	
	innerModel = innerModel_;
/*	printf("%s %d sr(%p) pf(%p) (%p)\n", __FILE__, __LINE__, this, pathfinder, innerModel.get());*/
	pathfinder->innerModelChanged(innerModel, totalpersons, intimate_seq, personal_seq, social_seq, object_seq, objectblock_seq);
}


void SocialRules::checkNewPersonInModel(AGMModel::SPtr worldModel_)
{	
	qDebug()<<__FUNCTION__;
	worldModel = worldModel_;
	idselected->clear();
	pSymbolId.clear();
	//Check if the person is in the model
 	for (uint i=1; i < 300; i++)
	{
		std::string name = "person" + std::to_string(i);
		int idx = 0;
		while ((personSymbolId = worldModel->getIdentifierByType("person", idx++)) != -1)
		{
			if (worldModel->getSymbolByIdentifier(personSymbolId)->getAttribute("imName") == name)
			{
				pSymbolId.push_back(personSymbolId);
                idselected->addItem(QString::number(personSymbolId));
				std::cout<<"Person found "<< name <<" "<<personSymbolId <<std::endl;
				break;
			}
		}
	}
	

	if (!pSymbolId.empty())
	{
		checkInteraction();
		checkMovement();
	}
	else
		qDebug()<<"No persons found";
}

void SocialRules::checkInteraction()
{	
	interactingId.clear();
	
	qDebug()<<__FUNCTION__;
	for (auto id : pSymbolId)
	{
		AGMModelSymbol::SPtr personAGM = worldModel->getSymbol(id);
		int32_t pairId = -1;
		for (auto edge = personAGM->edgesBegin(worldModel); edge != personAGM->edgesEnd(worldModel); edge++)
		{
			const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();
			if (edge->getLabel() == "interacting")
			{
				const string secondType = worldModel->getSymbol(symbolPair.second)->symbolType;
				if (symbolPair.first == id and secondType == "person")
				{
					pairId = symbolPair.second;
					qDebug()<<"INTERACTING" << symbolPair.first <<"AND " <<symbolPair.second;
					break;
				}
			}
		}

		vector <int32_t> Ids;
		bool p1found = false;
		bool p2found = false;

		for (auto i : interactingId)
		{
			for (auto v : i)
			{
				if (v == id) p1found = true;
				if (v == pairId) p2found = true;

			}
		}

		if (pairId != -1)
		{
			if (!p1found and !p2found)
			{
				Ids.push_back(id);
                Ids.push_back(pairId);
                interactingId.push_back(Ids);
			}

			else if (p1found and !p2found)
			{
				for (auto &i : interactingId)
				{
					for (auto v : i)
					{
						if (v == id) i.push_back(pairId);
					}
				}
			}

			else if (!p1found and p2found)
            {
                for (auto &i : interactingId)
                {
                    for (auto v : i)
                    {
                        if (v == pairId) i.push_back(id);
                    }
                }
            }
		}

		else if (!p1found)
        {
            Ids.push_back(id);
            interactingId.push_back(Ids);
        }
	}
}

void SocialRules::checkMovement()
{
//	qDebug()<<__FUNCTION__;
	AGMModel::SPtr newM(new AGMModel(worldModel));
	interactingpersons.clear();
	totalpersons.clear();
 	
	if (!interactingId.empty())
	{
		for (auto id_vect: interactingId)
		{
			SNGPersonSeq persons;
			for (auto id : id_vect)
			{
				AGMModelSymbol::SPtr personParent = newM->getParentByLink(id, "RT");
				AGMModelEdge &edgeRT = newM->getEdgeByIdentifiers(personParent->identifier, id, "RT");

                person.id = id;
                person.x = str2float(edgeRT.attributes["tx"])/1000;
                person.z = str2float(edgeRT.attributes["tz"])/1000;
                person.angle = str2float(edgeRT.attributes["ry"]);
                //person.vel=str2float(edgeRT.attributes["velocity"]);
                person.vel = 0;

//			    qDebug()<<"POSICION PERSONA "<< person.x <<" " <<person.z <<" " <<person.angle;
				persons.push_back(person);
				totalpersons.push_back(person);
				
			}
			
			interactingpersons.push_back(persons);
			
		}
		ApplySocialRules();
		

		
	}

}
SNGPolylineSeq SocialRules::ApplySocialRules()
{
//	qDebug()<<__FUNCTION__;

// 	movperson.clear();
// 	quietperson.clear();
// 	
// 	SNGPolylineSeq seq;	
// 	
// 	if (person.vel > 0)
// 		movperson.push_back(person);
// 	else
// 		quietperson.push_back(person);
// 
// 	if (!quietperson.empty())
// 	{
// 		SNGPolylineSeq secuencia = calculateGauss(false);
// 		
// 		for(auto s: secuencia)	
// 			seq.push_back(s);
// 			    
// 	}
// 	
// 	if (!movperson.empty())
// 	{
// 		SNGPolylineSeq secuencia2 = PassOnRight(false);
// 		for(auto s: secuencia2)
// 			seq.push_back(s);			
// 	}
	
	if(!interactingpersons.empty())
	{	
		try
		{
			social_seq.clear();
			personal_seq.clear();
			intimate_seq.clear();
			object_seq.clear();
			objectblock_seq.clear();
	
			for (auto per: interactingpersons)
			{
				if((per.size() == 1) and (porpulsed))
				{
					seq = socialnavigationgaussian_proxy->  getPassOnRight(per, 0.1, false);
					for (auto s:seq) {social_seq.push_back(s);}
					
					seq = socialnavigationgaussian_proxy-> getPassOnRight(per, 0.4, false);
					for (auto s:seq) {personal_seq.push_back(s);}

					seq = socialnavigationgaussian_proxy-> getPassOnRight(per, 0.8, false);
					for (auto s:seq) {intimate_seq.push_back(s);}

				}
				else
				{
					seq = socialnavigationgaussian_proxy-> getPersonalSpace(per, 0.1, false);
					for (auto s:seq) {social_seq.push_back(s);}

					seq = socialnavigationgaussian_proxy-> getPersonalSpace(per, 0.4, false);
					for (auto s:seq) {personal_seq.push_back(s);}

                    seq = socialnavigationgaussian_proxy-> getPersonalSpace(per, 0.8, false);
                    for (auto s:seq) {intimate_seq.push_back(s);}
				}
			}

			if (followpulsed) followPerson();
			if (accompanypulsed) accompanyPerson();
	
		}


		catch( const Ice::Exception &e)
		{
			std::cout << e << std::endl;
		}
	}
	
	if (!objects.empty())	
	{
        object_seq = socialnavigationgaussian_proxy->getObjectInteraction(totalpersons,objects,false,false);

		SNGPolylineSeq secuenciaobj = objectInteraction(false);
		for(auto s: secuenciaobj)
			objectblock_seq.push_back(s);

	}
	pathfinder->innerModelChanged(innerModel, totalpersons, intimate_seq, personal_seq, social_seq, object_seq,objectblock_seq);



	//SNGPolylineSeq seqpoints = socialnavigationgaussian_proxy->RemovePoints(seq);
	
	return seq;
}

/**
* \brief If the person is in the model it is added to a vector of persons wich is sent to the socialnavigationGaussian
* to model its personal space. 
* The function returns a sequence of polylines.
*/



SNGPolylineSeq SocialRules::calculateGauss(bool draw, float h)
{
	
//	qDebug()<<__FUNCTION__ << "con h = " << h;
	if (!interactingpersons.empty())
	{
		seq.clear();
		for (auto per: interactingpersons)
		{
			seq = socialnavigationgaussian_proxy-> getPersonalSpace(per, h, draw);
		}
		
	}
	return seq;
}


SNGPolylineSeq SocialRules::PassOnRight(bool draw)
{
//	qDebug()<<__FUNCTION__;
	if (!movperson.empty())
	{
		seq.clear();
		seq = socialnavigationgaussian_proxy-> getPassOnRight(movperson, h, draw);
	}
	
	return seq;	
}
 
SNGPolylineSeq SocialRules::objectInteraction(bool d)
{
//	qDebug()<<__FUNCTION__;
	
	RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
	AGMModelConverter::fromIceToInternal(w, worldModel);
	
	objects.clear();
	SNGPolylineSeq sequenceObj;
	try
	{
		int idx=0;
		while ((objectSymbolId = worldModel->getIdentifierByType("object_interaction", idx++)) != -1)
		{	
			
			AGMModelSymbol::SPtr objectP = worldModel->getParentByLink(objectSymbolId, "RT");
			AGMModelEdge &edgeRT  = worldModel->getEdgeByIdentifiers(objectP->identifier,objectSymbolId, "RT");
			SNGObject object;
			object.x = str2float(edgeRT.attributes["tx"])/1000;
			object.z = str2float(edgeRT.attributes["tz"])/1000;
			object.angle=str2float(edgeRT.attributes["ry"]);
			object.space=str2float(worldModel->getSymbolByIdentifier(objectSymbolId)->getAttribute("interaction"));
		
			objects.push_back(object);
			
//			qDebug()<<"Object"<<"Pose x"<<object.x<<"Pose z"<<object.z<<"Angle"<<object.angle<<"Space"<<object.space;
		}

		sequenceObj = socialnavigationgaussian_proxy->getObjectInteraction(totalpersons,objects,true,d);

	}
	catch(...){}
	
	return sequenceObj;
	
}

void SocialRules::checkstate()
{
//    qDebug()<<__FUNCTION__;

    if (follow->checkState() == Qt::CheckState(2) and followpulsed == false)
    	{
			followpulsed = true;
			accompanypulsed = false;
			accompany->setCheckState(Qt::CheckState(0));
            followPerson();

		}

    else if (accompany->checkState() == Qt::CheckState(2) and accompanypulsed == false)
	{
		accompanypulsed = true;
		followpulsed = false;
		follow->setCheckState(Qt::CheckState(0));
        accompanyPerson();
	}

	if (follow->checkState() == Qt::CheckState(0))
	    followpulsed = false;

    if (accompany->checkState() == Qt::CheckState(0))
        accompanypulsed = false;

    if (por->checkState() == Qt::CheckState(2))
        porpulsed = true;
    else
        porpulsed = false;
}

void SocialRules::followPerson()
{
    int32_t id = idselected->currentText().toInt();
    qDebug()<<"Person selected" << id;

    if (id != 0)
    {
        AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
        AGMModelEdge &edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");

        person.x = str2float(edgeRT.attributes["tx"]);
        person.z = str2float(edgeRT.attributes["tz"]);
        person.angle = str2float(edgeRT.attributes["ry"]);
        //person.vel=str2float(edgeRT.attributes["velocity"]);
        person.vel = 0;

        auto angle = M_PI/2 - person.angle;
        pathfinder->go(person.x - 900*cos(angle) , person.z -900*sin(angle));
    }


}


void SocialRules::accompanyPerson()
{
    qDebug() <<__FUNCTION__;
    int32_t id = idselected->currentText().toInt();

    if (id != 0)
    {
        AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
        AGMModelEdge &edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");

        person.x = str2float(edgeRT.attributes["tx"]);
        person.z = str2float(edgeRT.attributes["tz"]);
        person.angle = str2float(edgeRT.attributes["ry"]);
        //person.vel=str2float(edgeRT.attributes["velocity"]);
        person.vel = 0;

        qDebug()<<"Person selected "<< id<< "Pose (x,z) = ( "<<  person.x <<","<<person.z<<")";
        auto angle = M_PI/2 - person.angle;
        pathfinder->go(person.x + 900*sin(angle) , person.z - cos(angle));
    }

}

void SocialRules::goToPerson()
{
	qDebug()<<__FUNCTION__;
	int32_t id = idselected->currentText().toInt();
	qDebug()<<"Person selected" << id;

	if (id != 0)
	{
		AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
		AGMModelEdge &edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");

		person.x = str2float(edgeRT.attributes["tx"]);
		person.z = str2float(edgeRT.attributes["tz"]);
		person.angle = str2float(edgeRT.attributes["ry"]);
		//person.vel=str2float(edgeRT.attributes["velocity"]);
		person.vel = 0;

		auto angle = M_PI/2 - person.angle;
		auto rotation = person.angle + M_PI;
		pathfinder->go_rot(person.x + 1200*cos(angle) , person.z +1200*sin(angle), rotation);
	}

}


void SocialRules::checkRobotmov()
{
	robotSymbolId = worldModel->getIdentifierByType("robot");
	AGMModelSymbol::SPtr robotparent = worldModel->getParentByLink(robotSymbolId, "RT");
	AGMModelEdge &edgeRTrobot  = worldModel->getEdgeByIdentifiers(robotparent->identifier, robotSymbolId, "RT");
		
	robot.x=str2float(edgeRTrobot.attributes["tx"])/1000;
	robot.z=str2float(edgeRTrobot.attributes["tz"])/1000;
	robot.angle=str2float(edgeRTrobot.attributes["ry"]);

	point.x=robot.x;
	point.z=robot.z;
		 
	if (poserobot.size() == 0)
		poserobot.push_back(point);
	  
	else if ((poserobot[poserobot.size()-1].x != point.x) or (poserobot[poserobot.size()-1].z !=point.z))
	{  
		float  dist=sqrt((point.x - poserobot[poserobot.size()-1].x)*(point.x - poserobot[poserobot.size()-1].x)
				+(point.z - poserobot[poserobot.size()-1].z)*(point.z - poserobot[poserobot.size()-1].z));
		    
		totaldist=totaldist + dist;
		qDebug()<<"Distancia calculada" << dist << "Distancia total" <<totaldist;
	}

	poserobot.push_back(point);

}

/**
 * \brief The innerModel is extracted from the AGM and the polylines are inserted on it as a set of planes.
 */

void SocialRules::UpdateInnerModel(SNGPolylineSeq seq)
{
// 	QMutexLocker locker(mutex);
// 	qDebug() << "----------------------"<< __FUNCTION__ << "----------------------";
// 	
// 	// Extract innerModel
// 	InnerModel *inner  = AGMInner::extractInnerModel(worldModel, "world", false); f
// 
// 	int count = 0;
// 
// 	for (auto s:seq)
// 	{
// 		auto previousPoint = s[s.size()-1];
// 		for (auto currentPoint:s)
// 		{
// 			QString name = QString("polyline_obs_")+QString::number(count,10);
// 			qDebug() << __FUNCTION__ << "nombre"<<name;
// 			QVec ppoint = QVec::vec3(previousPoint.x*1000, 1000, previousPoint.z*1000);
// 			QVec cpoint = QVec::vec3(currentPoint.x*1000, 1000, currentPoint.z*1000);
// 			QVec center = (cpoint + ppoint).operator*(0.5);
// 
// 			QVec normal = (cpoint-ppoint);
// 			float dist = normal.norm2();	
// 			float temp = normal(2);
// 			normal(2) = normal(0);
// 			normal(0) = -temp;
// 
// 			if (inner->getNode(name))
// 			{
// 				try
// 				{
// 					inner->removeNode(name);
// 				}
// 
// 				catch(QString es){ qDebug() << "EXCEPCION" << es;}
// 			}
// 
// 			InnerModelNode *parent = inner->getNode(QString("world"));
// 			if (parent == NULL)
// 				printf("%s: parent does not exist\n", __FUNCTION__);
// 			else
// 			{			
// 				InnerModelPlane *plane;
// 				try
// 				{
// 					plane  = inner-> newPlane(name, parent, QString("#FFFF00"), dist, 2000, 90, 1, normal(0), normal(1), normal(2), center(0), center(1), center(2), true);
// 					parent->addChild(plane); 
// 				}
// 				catch(QString es)
// 				{ 
// 					qDebug() << "EXCEPCION" << es;}
// 			}
// 			count++;
// 			previousPoint=currentPoint;
// 		}
// 	}

// 	innerModel.reset(inner);
// 	pathfinder.innerModelChanged(innerModel, polyLineList);
// 	
// 	viewer->reloadInnerModel(innerModel);
}



void SocialRules::saveData()
{	
	qDebug("Saving in robotpose.txt the robot's pose");
	ofstream file("robotpose.txt", ofstream::out);
	for (auto p:poserobot)
	{
		file<< p.x << " " <<p.z<< endl;
	}
	file.close();

	qDebug("Saving in personpose.txt the human's poses");
	ofstream file2("personpose.txt", ofstream::out);
	for (auto person:totalpersons)
	{
		file2<< person.x << " " <<person.z<<" "<<person.angle<< endl;
	}
	file2.close();	
	poserobot.clear();


	qDebug("Saving intimate polyline");
	ofstream file3("polyline_intimate.txt", ofstream::out);
	for (auto s:intimate_seq)
	{
		for (auto p: s)
			file3<< p.x << " " <<p.z<<" "<< endl;
		file3 << " "<<endl ;
	}

	file3.close();
	
	qDebug("Saving personal polyline");
	ofstream file4("polyline_personal.txt", ofstream::out);
	for (auto s:personal_seq)
	{
		for (auto p: s)
			file4<< p.x << " " <<p.z<<" "<< endl;
        file4 << " "<<endl ;
	}
	file4.close();
	
	qDebug("Saving social polyline");
	ofstream file5("polyline_social.txt", ofstream::out);
	for (auto s:social_seq)
	{
		for (auto p: s)
			file5<< p.x << " " <<p.z<<" "<< endl;
		file5<<" "<<endl;
	}
	file5.close();

	qDebug()<<"Saving in dist.txt the total distance"<<totaldist;
	ofstream file6("dist.txt", ofstream::out);
	file6 <<" Distance " << totaldist << endl;
	totaldist = 0;
	file6.close();
}



bool SocialRules::checkHRI(SNGPerson p, int ind , InnerPtr &i, AGMModel::SPtr w)
{	
	worldModel = w;
	bool changes = false;
	/////////////////////Checking if the person is close and looking at the robot
	std::string type = "person";
	std::string name = "person" + std::to_string(ind);
		
	qDebug()<<QString::fromStdString(type)<<"-"<<QString::fromStdString(name);

	bool looking = false;
	bool close = false;

	mux->lock();
	QVec pose = i->transform(QString::fromStdString(name),"robot");
	mux->unlock();

	float dist = sqrt(pose.x()*pose.x()+pose.z()*pose.z());
	float angle = atan2(pose.x(),pose.z());
	
	qDebug()<<"pose x"<<pose.x()<<"pose z"<<pose.z();
	qDebug()<<"dist"<<dist<<"angle"<<abs(angle/0.0175);
	
	if (abs(angle)<20*0.0175)
		looking = true;
	
	if (dist<2000.0)
		close = true;
	
	///////////////////////Add edge interrupting////////////////////////

	int32_t Idperson = worldModel->getIdentifierByType(type);
	AGMModelSymbol::SPtr person = worldModel->getSymbolByIdentifier(Idperson);
	int32_t Idrobot = worldModel->getIdentifierByType("robot");
	AGMModelSymbol::SPtr robot = worldModel->getSymbolByIdentifier(Idrobot);

	if ((looking==true) and (close==true))
	{
		qDebug()<<"CERCA Y MIRANDO";
		try
		{		
			worldModel->addEdge(person,robot, "interrupting");
			qDebug()<<"SE AÑADE EL ENLACE";
			changes = true;
		}
		catch(...)
		{
			qDebug()<<"EXISTE EL ENLACE";
			changes = false;
		}		
	}	
	
	else 
	{	try
		{
			worldModel->removeEdge(person,robot,"interrupting");
			changes = true;
		}
		catch(...)
		{
			qDebug()<<"NO EXISTE EL ENLACE";
			changes = false;
		}
	}
	return changes;
}
