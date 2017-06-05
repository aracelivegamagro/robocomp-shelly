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
// Generated from file `AprilBasedLocalization.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#include <AprilBasedLocalization.h>
#include <Ice/LocalException.h>
#include <Ice/ObjectFactory.h>
#include <Ice/BasicStream.h>
#include <Ice/Object.h>
#include <IceUtil/Iterator.h>

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

namespace
{

const ::std::string __RoboCompAprilBasedLocalization__AprilBasedLocalization__newAprilBasedPose_name = "newAprilBasedPose";

}
::IceProxy::Ice::Object* ::IceProxy::RoboCompAprilBasedLocalization::upCast(::IceProxy::RoboCompAprilBasedLocalization::AprilBasedLocalization* p) { return p; }

void
::IceProxy::RoboCompAprilBasedLocalization::__read(::IceInternal::BasicStream* __is, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompAprilBasedLocalization::AprilBasedLocalization>& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RoboCompAprilBasedLocalization::AprilBasedLocalization;
        v->__copyFrom(proxy);
    }
}

void
IceProxy::RoboCompAprilBasedLocalization::AprilBasedLocalization::newAprilBasedPose(::Ice::Float x, ::Ice::Float z, ::Ice::Float alpha, const ::Ice::Context* __ctx)
{
    ::IceInternal::InvocationObserver __observer(this, __RoboCompAprilBasedLocalization__AprilBasedLocalization__newAprilBasedPose_name, __ctx);
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::RoboCompAprilBasedLocalization::AprilBasedLocalization* __del = dynamic_cast< ::IceDelegate::RoboCompAprilBasedLocalization::AprilBasedLocalization*>(__delBase.get());
            __del->newAprilBasedPose(x, z, alpha, __ctx, __observer);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, __observer);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, true, __cnt, __observer);
        }
    }
}

::Ice::AsyncResultPtr
IceProxy::RoboCompAprilBasedLocalization::AprilBasedLocalization::begin_newAprilBasedPose(::Ice::Float x, ::Ice::Float z, ::Ice::Float alpha, const ::Ice::Context* __ctx, const ::IceInternal::CallbackBasePtr& __del, const ::Ice::LocalObjectPtr& __cookie)
{
    ::IceInternal::OutgoingAsyncPtr __result = new ::IceInternal::OutgoingAsync(this, __RoboCompAprilBasedLocalization__AprilBasedLocalization__newAprilBasedPose_name, __del, __cookie);
    try
    {
        __result->__prepare(__RoboCompAprilBasedLocalization__AprilBasedLocalization__newAprilBasedPose_name, ::Ice::Normal, __ctx);
        ::IceInternal::BasicStream* __os = __result->__startWriteParams(::Ice::DefaultFormat);
        __os->write(x);
        __os->write(z);
        __os->write(alpha);
        __result->__endWriteParams();
        __result->__send(true);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __result->__exceptionAsync(__ex);
    }
    return __result;
}

void
IceProxy::RoboCompAprilBasedLocalization::AprilBasedLocalization::end_newAprilBasedPose(const ::Ice::AsyncResultPtr& __result)
{
    __end(__result, __RoboCompAprilBasedLocalization__AprilBasedLocalization__newAprilBasedPose_name);
}

const ::std::string&
IceProxy::RoboCompAprilBasedLocalization::AprilBasedLocalization::ice_staticId()
{
    return ::RoboCompAprilBasedLocalization::AprilBasedLocalization::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RoboCompAprilBasedLocalization::AprilBasedLocalization::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RoboCompAprilBasedLocalization::AprilBasedLocalization);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RoboCompAprilBasedLocalization::AprilBasedLocalization::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RoboCompAprilBasedLocalization::AprilBasedLocalization);
}

::IceProxy::Ice::Object*
IceProxy::RoboCompAprilBasedLocalization::AprilBasedLocalization::__newInstance() const
{
    return new AprilBasedLocalization;
}

void
IceDelegateM::RoboCompAprilBasedLocalization::AprilBasedLocalization::newAprilBasedPose(::Ice::Float x, ::Ice::Float z, ::Ice::Float alpha, const ::Ice::Context* __context, ::IceInternal::InvocationObserver& __observer)
{
    ::IceInternal::Outgoing __og(__handler.get(), __RoboCompAprilBasedLocalization__AprilBasedLocalization__newAprilBasedPose_name, ::Ice::Normal, __context, __observer);
    try
    {
        ::IceInternal::BasicStream* __os = __og.startWriteParams(::Ice::DefaultFormat);
        __os->write(x);
        __os->write(z);
        __os->write(alpha);
        __og.endWriteParams();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(__og.hasResponse())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.readEmptyParams();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateD::RoboCompAprilBasedLocalization::AprilBasedLocalization::newAprilBasedPose(::Ice::Float x, ::Ice::Float z, ::Ice::Float alpha, const ::Ice::Context* __context, ::IceInternal::InvocationObserver&)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Ice::Float __p_x, ::Ice::Float __p_z, ::Ice::Float __p_alpha, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_x(__p_x),
            _m_z(__p_z),
            _m_alpha(__p_alpha)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::RoboCompAprilBasedLocalization::AprilBasedLocalization* servant = dynamic_cast< ::RoboCompAprilBasedLocalization::AprilBasedLocalization*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->newAprilBasedPose(_m_x, _m_z, _m_alpha, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::Ice::Float _m_x;
        ::Ice::Float _m_z;
        ::Ice::Float _m_alpha;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __RoboCompAprilBasedLocalization__AprilBasedLocalization__newAprilBasedPose_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(x, z, alpha, __current);
        try
        {
            __direct.getServant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

::Ice::Object* RoboCompAprilBasedLocalization::upCast(::RoboCompAprilBasedLocalization::AprilBasedLocalization* p) { return p; }

namespace
{
const ::std::string __RoboCompAprilBasedLocalization__AprilBasedLocalization_ids[2] =
{
    "::Ice::Object",
    "::RoboCompAprilBasedLocalization::AprilBasedLocalization"
};

}

bool
RoboCompAprilBasedLocalization::AprilBasedLocalization::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RoboCompAprilBasedLocalization__AprilBasedLocalization_ids, __RoboCompAprilBasedLocalization__AprilBasedLocalization_ids + 2, _s);
}

::std::vector< ::std::string>
RoboCompAprilBasedLocalization::AprilBasedLocalization::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RoboCompAprilBasedLocalization__AprilBasedLocalization_ids[0], &__RoboCompAprilBasedLocalization__AprilBasedLocalization_ids[2]);
}

const ::std::string&
RoboCompAprilBasedLocalization::AprilBasedLocalization::ice_id(const ::Ice::Current&) const
{
    return __RoboCompAprilBasedLocalization__AprilBasedLocalization_ids[1];
}

const ::std::string&
RoboCompAprilBasedLocalization::AprilBasedLocalization::ice_staticId()
{
    return __RoboCompAprilBasedLocalization__AprilBasedLocalization_ids[1];
}

::Ice::DispatchStatus
RoboCompAprilBasedLocalization::AprilBasedLocalization::___newAprilBasedPose(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.startReadParams();
    ::Ice::Float x;
    ::Ice::Float z;
    ::Ice::Float alpha;
    __is->read(x);
    __is->read(z);
    __is->read(alpha);
    __inS.endReadParams();
    newAprilBasedPose(x, z, alpha, __current);
    __inS.__writeEmptyParams();
    return ::Ice::DispatchOK;
}

namespace
{
const ::std::string __RoboCompAprilBasedLocalization__AprilBasedLocalization_all[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "newAprilBasedPose"
};

}

::Ice::DispatchStatus
RoboCompAprilBasedLocalization::AprilBasedLocalization::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< const ::std::string*, const ::std::string*> r = ::std::equal_range(__RoboCompAprilBasedLocalization__AprilBasedLocalization_all, __RoboCompAprilBasedLocalization__AprilBasedLocalization_all + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __RoboCompAprilBasedLocalization__AprilBasedLocalization_all)
    {
        case 0:
        {
            return ___ice_id(in, current);
        }
        case 1:
        {
            return ___ice_ids(in, current);
        }
        case 2:
        {
            return ___ice_isA(in, current);
        }
        case 3:
        {
            return ___ice_ping(in, current);
        }
        case 4:
        {
            return ___newAprilBasedPose(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
RoboCompAprilBasedLocalization::AprilBasedLocalization::__writeImpl(::IceInternal::BasicStream* __os) const
{
    __os->startWriteSlice(ice_staticId(), -1, true);
    __os->endWriteSlice();
}

void
RoboCompAprilBasedLocalization::AprilBasedLocalization::__readImpl(::IceInternal::BasicStream* __is)
{
    __is->startReadSlice();
    __is->endReadSlice();
}

void 
RoboCompAprilBasedLocalization::__patch(AprilBasedLocalizationPtr& handle, const ::Ice::ObjectPtr& v)
{
    handle = ::RoboCompAprilBasedLocalization::AprilBasedLocalizationPtr::dynamicCast(v);
    if(v && !handle)
    {
        IceInternal::Ex::throwUOE(::RoboCompAprilBasedLocalization::AprilBasedLocalization::ice_staticId(), v);
    }
}
