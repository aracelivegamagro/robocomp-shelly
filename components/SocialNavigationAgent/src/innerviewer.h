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

#ifndef INNERVIEWER_H
#define INNERVIEWER_H

#include <iostream>
#include <fstream>
#include <mutex>
#include <osgViewer/Viewer>
#include <innermodel/innermodelviewer.h>
#include <innermodel/innermodeldraw.h>
#include <innermodel/innermodelmgr.h>

/**
 * @brief Threaded InnerModelViewer to be used inside components that visualize the model
 * It gets a copy of the InnerModel so new elements can be drawn in it without interferring with the original one
 * The copy has to be resynchronized periodically to update the position of the robot, its body and any other changes occuring in it.
 */
class InnerViewer: public QThread
{
	public:
		typedef std::lock_guard<std::mutex> guard;
		InnerViewer(InnerModelMgr innerModel_, const std::string &name_ = "unknown", uint period = 100000, QObject *parent = 0);
		void run();
		inline void lock(){ mutex.lock(); };
		inline void unlock(){ mutex.unlock(); };
		mutable std::mutex mutex;
	
		void reloadInnerModel(InnerModelMgr other);
	
		/////////////////////////////
		// NOT thread safe interface
		/////////////////////////////
		void removeNode(const QString &item);
		void addTransform_ignoreExisting(const QString &item_, const QString &parent_, const QVec &pos = QVec::zeros(6));
		void addTransform(const QString &item_, const QString &parent_, const QVec &pos = QVec::zeros(6));
		void drawLine(const QString &item_, const QString &parent_, const QVec &center, const QVec &normal, float length, float width, const QString &texture = "#550000");
		void addPlane_ignoreExisting(const QString &item_, const QString &parent_, const QVec &center, const QVec &normal, const QString &texture, const QVec &size);
		void addPlane_notExisting(const QString &item_, const QString &parent_, const QVec &center, const QVec &normal, const QString &texture, const QVec &size);
		void updateTransformValues(const QString item_, const QVec &pos, const QString &parent = "");
	
		//////////////////////////
		/// thread safe interface
		//////////////////////////
		void ts_removeNode(const QString &item){guard gl(mutex); removeNode(item); };
		void ts_addTransform_ignoreExisting(const QString &item_, const QString &parent_, const QVec &pos = QVec::zeros(6)){guard gl(mutex);addTransform_ignoreExisting(item_, parent_,pos);};
		void ts_addTransform(const QString &item_, const QString &parent_, const QVec &pos = QVec::zeros(6)){guard gl(mutex);addTransform(item_, parent_, pos);};
		void ts_drawLine(const QString &item_, const QString &parent_, const QVec &center, const QVec &normal, float length, float width, const QString &texture = "#550000"){guard gl(mutex); drawLine(item_, parent_, center, normal, length, width, texture);};
		void ts_addPlane_ignoreExisting(const QString &item_, const QString &parent_, const QVec &center, const QVec &normal, const QString &texture, const QVec &size){guard gl(mutex); addPlane_ignoreExisting(item_, parent_, center, normal, texture, size);};
		void ts_addPlane_notExisting(const QString &item_, const QString &parent_, const QVec &center, const QVec &normal, const QString &texture, const QVec &size){guard gl(mutex);addPlane_notExisting(item_, parent_, center, normal, texture, size);};
		void ts_updateTransformValues(const QString item_, const QVec &pos, const QString &parent = "");
		
	private:
		InnerModelViewer *innerModelViewer;
		InnerModelMgr innerModel;
		osgViewer::Viewer viewer;
		void createWindow(osgViewer::Viewer& viewer, const std::string &name);
		QSettings *settings ;
		osgGA::TrackballManipulator *tb;
		uint period;
};

#endif
