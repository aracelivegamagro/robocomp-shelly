/*
 *    Copyright (C)2018 by YOUR NAME HERE
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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	innerModel = new InnerModel();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    astra::terminate();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

        astra::initialize();

        reader = new astra::StreamReader(sensor.create_reader());
        listener = new BodyVisualizer();

        auto depthStream = reader->stream<astra::DepthStream>();
        depthStream.start();

        auto bodyStream = reader->stream<astra::BodyStream>();
        bodyStream.start();

        reader->add_listener(*listener);

    timer.start(Period);
    return true;
}


void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	if (first){

        try{
            qDebug()<<"Creating window";
            window.create(sf::VideoMode(1280, 960), "Simple Body Viewer");
        }

        catch(...)
        {
            qDebug()<<"Error al crear la ventana Simple BodyViewer";
        }
        first = false;

    }




    while (window.isOpen())
    {

        astra_update();

        sf::Event event;
        while (window.pollEvent(event))
        {
            switch (event.type)
            {
                case sf::Event::Closed:

                      window.close();
                    astra::terminate();
                    break;
                case sf::Event::KeyPressed:
                {
                    if ((event.key.code == sf::Keyboard::C and event.key.control) or (event.key.code == sf::Keyboard::Escape))
                    {

                        window.close();
                        astra::terminate();
                        break;
                    }
                    break;
                }
                default:
                    break;
            }
        }

        // clear the window with black color

        window.clear(sf::Color::Black);

        listener->draw_to(window);
        window.display();


    }



#ifdef USE_QTGUI
    if (innerModelViewer) innerModelViewer->update();
    osgView->frame();
#endif
}

