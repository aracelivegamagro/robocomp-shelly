/*
 * Copyright 2014 <copyright holder> <email>
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

#ifndef CURRENTTARGET_H
#define CURRENTTARGET_H
#include <QMutex>
#include <QMutexLocker>
#include <qmat/QMatAll>
#include <mutex>
#include <thread>

/**
 * @brief Holds the current target since it is created until it is destroyed
 * 
 */
class CurrentTarget
{
	typedef std::lock_guard<std::mutex> guard;
	public:
		
 		CurrentTarget() = default;
 		CurrentTarget(CurrentTarget &&other);
// 		void reset(const QVec &t = QVec::zeros(3), const QVec &r = QVec::zeros(3), bool hasRotation = false); 						
 		QVec getTranslation() const;			
		void setTranslation(const QVec &t);
 		QVec getRotation() const;		
// 		void setRotation(const QVec &r);
// 		QVec getFullPose() const;
// 		bool isWithoutPlan() const ;
// 		void setWithoutPlan(bool w); 
// 		void print();
// 		ulong getElapsedTime() const;  //ms
 		bool hasRotation() const;
// 		bool isBlocked() const;
// 		void setHasRotation(bool a);
// 		enum class State { GOTO, SETHEADING, STOP, CHANGETARGET, GOBACKWARDS, IDLE, DISCONNECTED, ROBOT_COLLISION, TARGET_COLLISION, LEARNING, BLOCKED};
// 		State state;
// 		void setState(State st);
		inline bool isActive() const;
		void setActive(bool value);			
// 		
// 	private:
		mutable std::mutex mutex;
  		QVec targetTr = QVec::zeros(3);
 		QVec targetRot = QVec::zeros(3);
// 		bool withoutPlan;
// 		QTime reloj;
 		bool doRotation = false;;
  		bool active = false;
		
};

#endif // CURRENTTARGET_H
