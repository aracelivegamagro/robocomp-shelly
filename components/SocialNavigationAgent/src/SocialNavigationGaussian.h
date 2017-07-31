// **********************************************************************
//
// Copyright (c) 2003-2013 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.5.1
//
// <auto-generated>
//
// Generated from file `SocialNavigationGaussian.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef ____SocialNavigationGaussian_h__
#define ____SocialNavigationGaussian_h__

#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/StreamHelpers.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/Outgoing.h>
#include <Ice/OutgoingAsync.h>
#include <Ice/Incoming.h>
#include <Ice/Direct.h>
#include <IceUtil/ScopedArray.h>
#include <IceUtil/Optional.h>
#include <Ice/StreamF.h>
#include <Ice/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 305
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 1
#       error Ice patch level mismatch!
#   endif
#endif

namespace IceProxy
{

namespace RoboCompSocialNavigationGaussian
{

class SocialNavigationGaussian;
void __read(::IceInternal::BasicStream*, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompSocialNavigationGaussian::SocialNavigationGaussian>&);
::IceProxy::Ice::Object* upCast(::IceProxy::RoboCompSocialNavigationGaussian::SocialNavigationGaussian*);

}

}

namespace RoboCompSocialNavigationGaussian
{

class SocialNavigationGaussian;
bool operator==(const SocialNavigationGaussian&, const SocialNavigationGaussian&);
bool operator<(const SocialNavigationGaussian&, const SocialNavigationGaussian&);
::Ice::Object* upCast(::RoboCompSocialNavigationGaussian::SocialNavigationGaussian*);
typedef ::IceInternal::Handle< ::RoboCompSocialNavigationGaussian::SocialNavigationGaussian> SocialNavigationGaussianPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompSocialNavigationGaussian::SocialNavigationGaussian> SocialNavigationGaussianPrx;
void __patch(SocialNavigationGaussianPtr&, const ::Ice::ObjectPtr&);

}

namespace RoboCompSocialNavigationGaussian
{

struct Pose2D
{
    ::Ice::Float x;
    ::Ice::Float z;
    ::Ice::Float angle;
};

struct SNGPoint2D
{
    ::Ice::Float x;
    ::Ice::Float z;
};

struct SNGPerson
{
    ::Ice::Float x;
    ::Ice::Float z;
    ::Ice::Float angle;
    ::Ice::Float vel;
};

typedef ::std::vector< ::RoboCompSocialNavigationGaussian::SNGPerson> SNGPersonSeq;

typedef ::std::vector< ::RoboCompSocialNavigationGaussian::SNGPoint2D> SNGPolyline;

typedef ::std::vector< ::RoboCompSocialNavigationGaussian::SNGPolyline> SNGPolylineSeq;

}

namespace Ice
{
template<>
struct StreamableTraits< ::RoboCompSocialNavigationGaussian::Pose2D>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 12;
    static const bool fixedLength = true;
};

template<class S>
struct StreamWriter< ::RoboCompSocialNavigationGaussian::Pose2D, S>
{
    static void write(S* __os, const ::RoboCompSocialNavigationGaussian::Pose2D& v)
    {
        __os->write(v.x);
        __os->write(v.z);
        __os->write(v.angle);
    }
};

template<class S>
struct StreamReader< ::RoboCompSocialNavigationGaussian::Pose2D, S>
{
    static void read(S* __is, ::RoboCompSocialNavigationGaussian::Pose2D& v)
    {
        __is->read(v.x);
        __is->read(v.z);
        __is->read(v.angle);
    }
};

template<>
struct StreamableTraits< ::RoboCompSocialNavigationGaussian::SNGPoint2D>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 8;
    static const bool fixedLength = true;
};

template<class S>
struct StreamWriter< ::RoboCompSocialNavigationGaussian::SNGPoint2D, S>
{
    static void write(S* __os, const ::RoboCompSocialNavigationGaussian::SNGPoint2D& v)
    {
        __os->write(v.x);
        __os->write(v.z);
    }
};

template<class S>
struct StreamReader< ::RoboCompSocialNavigationGaussian::SNGPoint2D, S>
{
    static void read(S* __is, ::RoboCompSocialNavigationGaussian::SNGPoint2D& v)
    {
        __is->read(v.x);
        __is->read(v.z);
    }
};

template<>
struct StreamableTraits< ::RoboCompSocialNavigationGaussian::SNGPerson>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 16;
    static const bool fixedLength = true;
};

template<class S>
struct StreamWriter< ::RoboCompSocialNavigationGaussian::SNGPerson, S>
{
    static void write(S* __os, const ::RoboCompSocialNavigationGaussian::SNGPerson& v)
    {
        __os->write(v.x);
        __os->write(v.z);
        __os->write(v.angle);
        __os->write(v.vel);
    }
};

template<class S>
struct StreamReader< ::RoboCompSocialNavigationGaussian::SNGPerson, S>
{
    static void read(S* __is, ::RoboCompSocialNavigationGaussian::SNGPerson& v)
    {
        __is->read(v.x);
        __is->read(v.z);
        __is->read(v.angle);
        __is->read(v.vel);
    }
};

}

namespace RoboCompSocialNavigationGaussian
{

class Callback_SocialNavigationGaussian_getPersonalSpace_Base : virtual public ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_SocialNavigationGaussian_getPersonalSpace_Base> Callback_SocialNavigationGaussian_getPersonalSpacePtr;

class Callback_SocialNavigationGaussian_getPassOnRight_Base : virtual public ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_SocialNavigationGaussian_getPassOnRight_Base> Callback_SocialNavigationGaussian_getPassOnRightPtr;

}

namespace IceProxy
{

namespace RoboCompSocialNavigationGaussian
{

class SocialNavigationGaussian : virtual public ::IceProxy::Ice::Object
{
public:

    ::RoboCompSocialNavigationGaussian::SNGPolylineSeq getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d)
    {
        return getPersonalSpace(persons, v, d, 0);
    }
    ::RoboCompSocialNavigationGaussian::SNGPolylineSeq getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::Context& __ctx)
    {
        return getPersonalSpace(persons, v, d, &__ctx);
    }
#ifdef ICE_CPP11
    ::Ice::AsyncResultPtr
    begin_getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::IceInternal::Function<void (const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getPersonalSpace(persons, v, d, 0, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getPersonalSpace(persons, v, d, 0, ::Ice::newCallback(__completed, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getPersonalSpace(persons, v, d, &__ctx, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getPersonalSpace(persons, v, d, &__ctx, ::Ice::newCallback(__completed, __sent));
    }
    
private:

    ::Ice::AsyncResultPtr __begin_getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::Context* __ctx, const ::IceInternal::Function<void (const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception, const ::IceInternal::Function<void (bool)>& __sent)
    {
        class Cpp11CB : public ::IceInternal::Cpp11FnCallbackNC
        {
        public:

            Cpp11CB(const ::std::function<void (const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&)>& responseFunc, const ::std::function<void (const ::Ice::Exception&)>& exceptionFunc, const ::std::function<void (bool)>& sentFunc) :
                ::IceInternal::Cpp11FnCallbackNC(exceptionFunc, sentFunc),
                _response(responseFunc)
            {
                CallbackBase::checkCallback(true, responseFunc || exceptionFunc != nullptr);
            }

            virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
            {
                ::RoboCompSocialNavigationGaussian::SocialNavigationGaussianPrx __proxy = ::RoboCompSocialNavigationGaussian::SocialNavigationGaussianPrx::uncheckedCast(__result->getProxy());
                ::RoboCompSocialNavigationGaussian::SNGPolylineSeq __ret;
                try
                {
                    __ret = __proxy->end_getPersonalSpace(__result);
                }
                catch(::Ice::Exception& ex)
                {
                    Cpp11FnCallbackNC::__exception(__result, ex);
                    return;
                }
                if(_response != nullptr)
                {
                    _response(__ret);
                }
            }
        
        private:
            
            ::std::function<void (const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&)> _response;
        };
        return begin_getPersonalSpace(persons, v, d, __ctx, new Cpp11CB(__response, __exception, __sent));
    }
    
public:
#endif

    ::Ice::AsyncResultPtr begin_getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d)
    {
        return begin_getPersonalSpace(persons, v, d, 0, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::Context& __ctx)
    {
        return begin_getPersonalSpace(persons, v, d, &__ctx, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getPersonalSpace(persons, v, d, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::Context& __ctx, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getPersonalSpace(persons, v, d, &__ctx, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::RoboCompSocialNavigationGaussian::Callback_SocialNavigationGaussian_getPersonalSpacePtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getPersonalSpace(persons, v, d, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::Context& __ctx, const ::RoboCompSocialNavigationGaussian::Callback_SocialNavigationGaussian_getPersonalSpacePtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getPersonalSpace(persons, v, d, &__ctx, __del, __cookie);
    }

    ::RoboCompSocialNavigationGaussian::SNGPolylineSeq end_getPersonalSpace(const ::Ice::AsyncResultPtr&);
    
private:

    ::RoboCompSocialNavigationGaussian::SNGPolylineSeq getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq&, ::Ice::Float, bool, const ::Ice::Context*);
    ::Ice::AsyncResultPtr begin_getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq&, ::Ice::Float, bool, const ::Ice::Context*, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& __cookie = 0);
    
public:

    ::RoboCompSocialNavigationGaussian::SNGPolylineSeq getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d)
    {
        return getPassOnRight(persons, v, d, 0);
    }
    ::RoboCompSocialNavigationGaussian::SNGPolylineSeq getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::Context& __ctx)
    {
        return getPassOnRight(persons, v, d, &__ctx);
    }
#ifdef ICE_CPP11
    ::Ice::AsyncResultPtr
    begin_getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::IceInternal::Function<void (const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getPassOnRight(persons, v, d, 0, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getPassOnRight(persons, v, d, 0, ::Ice::newCallback(__completed, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getPassOnRight(persons, v, d, &__ctx, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getPassOnRight(persons, v, d, &__ctx, ::Ice::newCallback(__completed, __sent));
    }
    
private:

    ::Ice::AsyncResultPtr __begin_getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::Context* __ctx, const ::IceInternal::Function<void (const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception, const ::IceInternal::Function<void (bool)>& __sent)
    {
        class Cpp11CB : public ::IceInternal::Cpp11FnCallbackNC
        {
        public:

            Cpp11CB(const ::std::function<void (const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&)>& responseFunc, const ::std::function<void (const ::Ice::Exception&)>& exceptionFunc, const ::std::function<void (bool)>& sentFunc) :
                ::IceInternal::Cpp11FnCallbackNC(exceptionFunc, sentFunc),
                _response(responseFunc)
            {
                CallbackBase::checkCallback(true, responseFunc || exceptionFunc != nullptr);
            }

            virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
            {
                ::RoboCompSocialNavigationGaussian::SocialNavigationGaussianPrx __proxy = ::RoboCompSocialNavigationGaussian::SocialNavigationGaussianPrx::uncheckedCast(__result->getProxy());
                ::RoboCompSocialNavigationGaussian::SNGPolylineSeq __ret;
                try
                {
                    __ret = __proxy->end_getPassOnRight(__result);
                }
                catch(::Ice::Exception& ex)
                {
                    Cpp11FnCallbackNC::__exception(__result, ex);
                    return;
                }
                if(_response != nullptr)
                {
                    _response(__ret);
                }
            }
        
        private:
            
            ::std::function<void (const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&)> _response;
        };
        return begin_getPassOnRight(persons, v, d, __ctx, new Cpp11CB(__response, __exception, __sent));
    }
    
public:
#endif

    ::Ice::AsyncResultPtr begin_getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d)
    {
        return begin_getPassOnRight(persons, v, d, 0, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::Context& __ctx)
    {
        return begin_getPassOnRight(persons, v, d, &__ctx, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getPassOnRight(persons, v, d, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::Context& __ctx, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getPassOnRight(persons, v, d, &__ctx, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::RoboCompSocialNavigationGaussian::Callback_SocialNavigationGaussian_getPassOnRightPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getPassOnRight(persons, v, d, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq& persons, ::Ice::Float v, bool d, const ::Ice::Context& __ctx, const ::RoboCompSocialNavigationGaussian::Callback_SocialNavigationGaussian_getPassOnRightPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getPassOnRight(persons, v, d, &__ctx, __del, __cookie);
    }

    ::RoboCompSocialNavigationGaussian::SNGPolylineSeq end_getPassOnRight(const ::Ice::AsyncResultPtr&);
    
private:

    ::RoboCompSocialNavigationGaussian::SNGPolylineSeq getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq&, ::Ice::Float, bool, const ::Ice::Context*);
    ::Ice::AsyncResultPtr begin_getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq&, ::Ice::Float, bool, const ::Ice::Context*, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& __cookie = 0);
    
public:
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_adapterId(const ::std::string& __id) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_locatorCacheTimeout(int __timeout) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_connectionCached(bool __cached) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_secure(bool __secure) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_preferSecure(bool __preferSecure) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_collocationOptimized(bool __co) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_twoway() const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_twoway().get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_oneway() const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_oneway().get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_batchOneway() const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_datagram() const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_datagram().get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_batchDatagram() const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_compress(bool __compress) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_timeout(int __timeout) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_connectionId(const ::std::string& __id) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<SocialNavigationGaussian> ice_encodingVersion(const ::Ice::EncodingVersion& __v) const
    {
        return dynamic_cast<SocialNavigationGaussian*>(::IceProxy::Ice::Object::ice_encodingVersion(__v).get());
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

}

namespace IceDelegate
{

namespace RoboCompSocialNavigationGaussian
{

class SocialNavigationGaussian : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual ::RoboCompSocialNavigationGaussian::SNGPolylineSeq getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq&, ::Ice::Float, bool, const ::Ice::Context*, ::IceInternal::InvocationObserver&) = 0;

    virtual ::RoboCompSocialNavigationGaussian::SNGPolylineSeq getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq&, ::Ice::Float, bool, const ::Ice::Context*, ::IceInternal::InvocationObserver&) = 0;
};

}

}

namespace IceDelegateM
{

namespace RoboCompSocialNavigationGaussian
{

class SocialNavigationGaussian : virtual public ::IceDelegate::RoboCompSocialNavigationGaussian::SocialNavigationGaussian,
                                 virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual ::RoboCompSocialNavigationGaussian::SNGPolylineSeq getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq&, ::Ice::Float, bool, const ::Ice::Context*, ::IceInternal::InvocationObserver&);

    virtual ::RoboCompSocialNavigationGaussian::SNGPolylineSeq getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq&, ::Ice::Float, bool, const ::Ice::Context*, ::IceInternal::InvocationObserver&);
};

}

}

namespace IceDelegateD
{

namespace RoboCompSocialNavigationGaussian
{

class SocialNavigationGaussian : virtual public ::IceDelegate::RoboCompSocialNavigationGaussian::SocialNavigationGaussian,
                                 virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual ::RoboCompSocialNavigationGaussian::SNGPolylineSeq getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq&, ::Ice::Float, bool, const ::Ice::Context*, ::IceInternal::InvocationObserver&);

    virtual ::RoboCompSocialNavigationGaussian::SNGPolylineSeq getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq&, ::Ice::Float, bool, const ::Ice::Context*, ::IceInternal::InvocationObserver&);
};

}

}

namespace RoboCompSocialNavigationGaussian
{

class SocialNavigationGaussian : virtual public ::Ice::Object
{
public:

    typedef SocialNavigationGaussianPrx ProxyType;
    typedef SocialNavigationGaussianPtr PointerType;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::RoboCompSocialNavigationGaussian::SNGPolylineSeq getPersonalSpace(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq&, ::Ice::Float, bool, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getPersonalSpace(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::RoboCompSocialNavigationGaussian::SNGPolylineSeq getPassOnRight(const ::RoboCompSocialNavigationGaussian::SNGPersonSeq&, ::Ice::Float, bool, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getPassOnRight(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

protected:
    virtual void __writeImpl(::IceInternal::BasicStream*) const;
    virtual void __readImpl(::IceInternal::BasicStream*);
    #ifdef __SUNPRO_CC
    using ::Ice::Object::__writeImpl;
    using ::Ice::Object::__readImpl;
    #endif
};

inline bool operator==(const SocialNavigationGaussian& l, const SocialNavigationGaussian& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

inline bool operator<(const SocialNavigationGaussian& l, const SocialNavigationGaussian& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

}

namespace RoboCompSocialNavigationGaussian
{

template<class T>
class CallbackNC_SocialNavigationGaussian_getPersonalSpace : public Callback_SocialNavigationGaussian_getPersonalSpace_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&);

    CallbackNC_SocialNavigationGaussian_getPersonalSpace(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), response(cb)
    {
    }

    virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompSocialNavigationGaussian::SocialNavigationGaussianPrx __proxy = ::RoboCompSocialNavigationGaussian::SocialNavigationGaussianPrx::uncheckedCast(__result->getProxy());
        ::RoboCompSocialNavigationGaussian::SNGPolylineSeq __ret;
        try
        {
            __ret = __proxy->end_getPersonalSpace(__result);
        }
        catch(::Ice::Exception& ex)
        {
            ::IceInternal::CallbackNC<T>::__exception(__result, ex);
            return;
        }
        if(response)
        {
            (::IceInternal::CallbackNC<T>::callback.get()->*response)(__ret);
        }
    }

    Response response;
};

template<class T> Callback_SocialNavigationGaussian_getPersonalSpacePtr
newCallback_SocialNavigationGaussian_getPersonalSpace(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_SocialNavigationGaussian_getPersonalSpace<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_SocialNavigationGaussian_getPersonalSpacePtr
newCallback_SocialNavigationGaussian_getPersonalSpace(T* instance, void (T::*cb)(const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_SocialNavigationGaussian_getPersonalSpace<T>(instance, cb, excb, sentcb);
}

template<class T, typename CT>
class Callback_SocialNavigationGaussian_getPersonalSpace : public Callback_SocialNavigationGaussian_getPersonalSpace_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&, const CT&);

    Callback_SocialNavigationGaussian_getPersonalSpace(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), response(cb)
    {
    }

    virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompSocialNavigationGaussian::SocialNavigationGaussianPrx __proxy = ::RoboCompSocialNavigationGaussian::SocialNavigationGaussianPrx::uncheckedCast(__result->getProxy());
        ::RoboCompSocialNavigationGaussian::SNGPolylineSeq __ret;
        try
        {
            __ret = __proxy->end_getPersonalSpace(__result);
        }
        catch(::Ice::Exception& ex)
        {
            ::IceInternal::Callback<T, CT>::__exception(__result, ex);
            return;
        }
        if(response)
        {
            (::IceInternal::Callback<T, CT>::callback.get()->*response)(__ret, CT::dynamicCast(__result->getCookie()));
        }
    }

    Response response;
};

template<class T, typename CT> Callback_SocialNavigationGaussian_getPersonalSpacePtr
newCallback_SocialNavigationGaussian_getPersonalSpace(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_SocialNavigationGaussian_getPersonalSpace<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_SocialNavigationGaussian_getPersonalSpacePtr
newCallback_SocialNavigationGaussian_getPersonalSpace(T* instance, void (T::*cb)(const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_SocialNavigationGaussian_getPersonalSpace<T, CT>(instance, cb, excb, sentcb);
}

template<class T>
class CallbackNC_SocialNavigationGaussian_getPassOnRight : public Callback_SocialNavigationGaussian_getPassOnRight_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&);

    CallbackNC_SocialNavigationGaussian_getPassOnRight(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), response(cb)
    {
    }

    virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompSocialNavigationGaussian::SocialNavigationGaussianPrx __proxy = ::RoboCompSocialNavigationGaussian::SocialNavigationGaussianPrx::uncheckedCast(__result->getProxy());
        ::RoboCompSocialNavigationGaussian::SNGPolylineSeq __ret;
        try
        {
            __ret = __proxy->end_getPassOnRight(__result);
        }
        catch(::Ice::Exception& ex)
        {
            ::IceInternal::CallbackNC<T>::__exception(__result, ex);
            return;
        }
        if(response)
        {
            (::IceInternal::CallbackNC<T>::callback.get()->*response)(__ret);
        }
    }

    Response response;
};

template<class T> Callback_SocialNavigationGaussian_getPassOnRightPtr
newCallback_SocialNavigationGaussian_getPassOnRight(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_SocialNavigationGaussian_getPassOnRight<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_SocialNavigationGaussian_getPassOnRightPtr
newCallback_SocialNavigationGaussian_getPassOnRight(T* instance, void (T::*cb)(const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_SocialNavigationGaussian_getPassOnRight<T>(instance, cb, excb, sentcb);
}

template<class T, typename CT>
class Callback_SocialNavigationGaussian_getPassOnRight : public Callback_SocialNavigationGaussian_getPassOnRight_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&, const CT&);

    Callback_SocialNavigationGaussian_getPassOnRight(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), response(cb)
    {
    }

    virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompSocialNavigationGaussian::SocialNavigationGaussianPrx __proxy = ::RoboCompSocialNavigationGaussian::SocialNavigationGaussianPrx::uncheckedCast(__result->getProxy());
        ::RoboCompSocialNavigationGaussian::SNGPolylineSeq __ret;
        try
        {
            __ret = __proxy->end_getPassOnRight(__result);
        }
        catch(::Ice::Exception& ex)
        {
            ::IceInternal::Callback<T, CT>::__exception(__result, ex);
            return;
        }
        if(response)
        {
            (::IceInternal::Callback<T, CT>::callback.get()->*response)(__ret, CT::dynamicCast(__result->getCookie()));
        }
    }

    Response response;
};

template<class T, typename CT> Callback_SocialNavigationGaussian_getPassOnRightPtr
newCallback_SocialNavigationGaussian_getPassOnRight(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_SocialNavigationGaussian_getPassOnRight<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_SocialNavigationGaussian_getPassOnRightPtr
newCallback_SocialNavigationGaussian_getPassOnRight(T* instance, void (T::*cb)(const ::RoboCompSocialNavigationGaussian::SNGPolylineSeq&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_SocialNavigationGaussian_getPassOnRight<T, CT>(instance, cb, excb, sentcb);
}

}

#endif
