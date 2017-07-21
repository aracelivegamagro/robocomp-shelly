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
// Generated from file `Logger.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef ____Logger_h__
#define ____Logger_h__

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

namespace RoboCompLogger
{

class Logger;
void __read(::IceInternal::BasicStream*, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompLogger::Logger>&);
::IceProxy::Ice::Object* upCast(::IceProxy::RoboCompLogger::Logger*);

}

}

namespace RoboCompLogger
{

class Logger;
bool operator==(const Logger&, const Logger&);
bool operator<(const Logger&, const Logger&);
::Ice::Object* upCast(::RoboCompLogger::Logger*);
typedef ::IceInternal::Handle< ::RoboCompLogger::Logger> LoggerPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompLogger::Logger> LoggerPrx;
void __patch(LoggerPtr&, const ::Ice::ObjectPtr&);

}

namespace RoboCompLogger
{

struct LogMessage
{
    ::std::string sender;
    ::std::string method;
    ::std::string file;
    ::Ice::Int line;
    ::std::string timeStamp;
    ::std::string message;
    ::std::string type;
    ::std::string fullpath;

    bool operator==(const LogMessage& __rhs) const
    {
        if(this == &__rhs)
        {
            return true;
        }
        if(sender != __rhs.sender)
        {
            return false;
        }
        if(method != __rhs.method)
        {
            return false;
        }
        if(file != __rhs.file)
        {
            return false;
        }
        if(line != __rhs.line)
        {
            return false;
        }
        if(timeStamp != __rhs.timeStamp)
        {
            return false;
        }
        if(message != __rhs.message)
        {
            return false;
        }
        if(type != __rhs.type)
        {
            return false;
        }
        if(fullpath != __rhs.fullpath)
        {
            return false;
        }
        return true;
    }

    bool operator<(const LogMessage& __rhs) const
    {
        if(this == &__rhs)
        {
            return false;
        }
        if(sender < __rhs.sender)
        {
            return true;
        }
        else if(__rhs.sender < sender)
        {
            return false;
        }
        if(method < __rhs.method)
        {
            return true;
        }
        else if(__rhs.method < method)
        {
            return false;
        }
        if(file < __rhs.file)
        {
            return true;
        }
        else if(__rhs.file < file)
        {
            return false;
        }
        if(line < __rhs.line)
        {
            return true;
        }
        else if(__rhs.line < line)
        {
            return false;
        }
        if(timeStamp < __rhs.timeStamp)
        {
            return true;
        }
        else if(__rhs.timeStamp < timeStamp)
        {
            return false;
        }
        if(message < __rhs.message)
        {
            return true;
        }
        else if(__rhs.message < message)
        {
            return false;
        }
        if(type < __rhs.type)
        {
            return true;
        }
        else if(__rhs.type < type)
        {
            return false;
        }
        if(fullpath < __rhs.fullpath)
        {
            return true;
        }
        else if(__rhs.fullpath < fullpath)
        {
            return false;
        }
        return false;
    }

    bool operator!=(const LogMessage& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const LogMessage& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const LogMessage& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const LogMessage& __rhs) const
    {
        return !operator<(__rhs);
    }
};

}

namespace Ice
{
template<>
struct StreamableTraits< ::RoboCompLogger::LogMessage>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 11;
    static const bool fixedLength = false;
};

template<class S>
struct StreamWriter< ::RoboCompLogger::LogMessage, S>
{
    static void write(S* __os, const ::RoboCompLogger::LogMessage& v)
    {
        __os->write(v.sender);
        __os->write(v.method);
        __os->write(v.file);
        __os->write(v.line);
        __os->write(v.timeStamp);
        __os->write(v.message);
        __os->write(v.type);
        __os->write(v.fullpath);
    }
};

template<class S>
struct StreamReader< ::RoboCompLogger::LogMessage, S>
{
    static void read(S* __is, ::RoboCompLogger::LogMessage& v)
    {
        __is->read(v.sender);
        __is->read(v.method);
        __is->read(v.file);
        __is->read(v.line);
        __is->read(v.timeStamp);
        __is->read(v.message);
        __is->read(v.type);
        __is->read(v.fullpath);
    }
};

}

namespace RoboCompLogger
{

class Callback_Logger_sendMessage_Base : virtual public ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_Logger_sendMessage_Base> Callback_Logger_sendMessagePtr;

}

namespace IceProxy
{

namespace RoboCompLogger
{

class Logger : virtual public ::IceProxy::Ice::Object
{
public:

    void sendMessage(const ::RoboCompLogger::LogMessage& m)
    {
        sendMessage(m, 0);
    }
    void sendMessage(const ::RoboCompLogger::LogMessage& m, const ::Ice::Context& __ctx)
    {
        sendMessage(m, &__ctx);
    }
#ifdef ICE_CPP11
    ::Ice::AsyncResultPtr
    begin_sendMessage(const ::RoboCompLogger::LogMessage& m, const ::IceInternal::Function<void ()>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return begin_sendMessage(m, 0, new ::IceInternal::Cpp11FnOnewayCallbackNC(__response, __exception, __sent));
    }
    ::Ice::AsyncResultPtr
    begin_sendMessage(const ::RoboCompLogger::LogMessage& m, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_sendMessage(m, 0, ::Ice::newCallback(__completed, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_sendMessage(const ::RoboCompLogger::LogMessage& m, const ::Ice::Context& __ctx, const ::IceInternal::Function<void ()>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return begin_sendMessage(m, &__ctx, new ::IceInternal::Cpp11FnOnewayCallbackNC(__response, __exception, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_sendMessage(const ::RoboCompLogger::LogMessage& m, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_sendMessage(m, &__ctx, ::Ice::newCallback(__completed, __sent));
    }
#endif

    ::Ice::AsyncResultPtr begin_sendMessage(const ::RoboCompLogger::LogMessage& m)
    {
        return begin_sendMessage(m, 0, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_sendMessage(const ::RoboCompLogger::LogMessage& m, const ::Ice::Context& __ctx)
    {
        return begin_sendMessage(m, &__ctx, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_sendMessage(const ::RoboCompLogger::LogMessage& m, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_sendMessage(m, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_sendMessage(const ::RoboCompLogger::LogMessage& m, const ::Ice::Context& __ctx, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_sendMessage(m, &__ctx, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_sendMessage(const ::RoboCompLogger::LogMessage& m, const ::RoboCompLogger::Callback_Logger_sendMessagePtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_sendMessage(m, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_sendMessage(const ::RoboCompLogger::LogMessage& m, const ::Ice::Context& __ctx, const ::RoboCompLogger::Callback_Logger_sendMessagePtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_sendMessage(m, &__ctx, __del, __cookie);
    }

    void end_sendMessage(const ::Ice::AsyncResultPtr&);
    
private:

    void sendMessage(const ::RoboCompLogger::LogMessage&, const ::Ice::Context*);
    ::Ice::AsyncResultPtr begin_sendMessage(const ::RoboCompLogger::LogMessage&, const ::Ice::Context*, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& __cookie = 0);
    
public:
    
    ::IceInternal::ProxyHandle<Logger> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_adapterId(const ::std::string& __id) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_locatorCacheTimeout(int __timeout) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_connectionCached(bool __cached) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_secure(bool __secure) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_preferSecure(bool __preferSecure) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_collocationOptimized(bool __co) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_twoway() const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_twoway().get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_oneway() const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_oneway().get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_batchOneway() const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_datagram() const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_datagram().get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_batchDatagram() const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_compress(bool __compress) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_timeout(int __timeout) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_connectionId(const ::std::string& __id) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<Logger> ice_encodingVersion(const ::Ice::EncodingVersion& __v) const
    {
        return dynamic_cast<Logger*>(::IceProxy::Ice::Object::ice_encodingVersion(__v).get());
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

namespace RoboCompLogger
{

class Logger : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void sendMessage(const ::RoboCompLogger::LogMessage&, const ::Ice::Context*, ::IceInternal::InvocationObserver&) = 0;
};

}

}

namespace IceDelegateM
{

namespace RoboCompLogger
{

class Logger : virtual public ::IceDelegate::RoboCompLogger::Logger,
               virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void sendMessage(const ::RoboCompLogger::LogMessage&, const ::Ice::Context*, ::IceInternal::InvocationObserver&);
};

}

}

namespace IceDelegateD
{

namespace RoboCompLogger
{

class Logger : virtual public ::IceDelegate::RoboCompLogger::Logger,
               virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void sendMessage(const ::RoboCompLogger::LogMessage&, const ::Ice::Context*, ::IceInternal::InvocationObserver&);
};

}

}

namespace RoboCompLogger
{

class Logger : virtual public ::Ice::Object
{
public:

    typedef LoggerPrx ProxyType;
    typedef LoggerPtr PointerType;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void sendMessage(const ::RoboCompLogger::LogMessage&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___sendMessage(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

protected:
    virtual void __writeImpl(::IceInternal::BasicStream*) const;
    virtual void __readImpl(::IceInternal::BasicStream*);
    #ifdef __SUNPRO_CC
    using ::Ice::Object::__writeImpl;
    using ::Ice::Object::__readImpl;
    #endif
};

inline bool operator==(const Logger& l, const Logger& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

inline bool operator<(const Logger& l, const Logger& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

}

namespace RoboCompLogger
{

template<class T>
class CallbackNC_Logger_sendMessage : public Callback_Logger_sendMessage_Base, public ::IceInternal::OnewayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)();

    CallbackNC_Logger_sendMessage(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::OnewayCallbackNC<T>(obj, cb, excb, sentcb)
    {
    }
};

template<class T> Callback_Logger_sendMessagePtr
newCallback_Logger_sendMessage(const IceUtil::Handle<T>& instance, void (T::*cb)(), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Logger_sendMessage<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_Logger_sendMessagePtr
newCallback_Logger_sendMessage(const IceUtil::Handle<T>& instance, void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Logger_sendMessage<T>(instance, 0, excb, sentcb);
}

template<class T> Callback_Logger_sendMessagePtr
newCallback_Logger_sendMessage(T* instance, void (T::*cb)(), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Logger_sendMessage<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_Logger_sendMessagePtr
newCallback_Logger_sendMessage(T* instance, void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_Logger_sendMessage<T>(instance, 0, excb, sentcb);
}

template<class T, typename CT>
class Callback_Logger_sendMessage : public Callback_Logger_sendMessage_Base, public ::IceInternal::OnewayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const CT&);

    Callback_Logger_sendMessage(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::OnewayCallback<T, CT>(obj, cb, excb, sentcb)
    {
    }
};

template<class T, typename CT> Callback_Logger_sendMessagePtr
newCallback_Logger_sendMessage(const IceUtil::Handle<T>& instance, void (T::*cb)(const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Logger_sendMessage<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_Logger_sendMessagePtr
newCallback_Logger_sendMessage(const IceUtil::Handle<T>& instance, void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Logger_sendMessage<T, CT>(instance, 0, excb, sentcb);
}

template<class T, typename CT> Callback_Logger_sendMessagePtr
newCallback_Logger_sendMessage(T* instance, void (T::*cb)(const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Logger_sendMessage<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_Logger_sendMessagePtr
newCallback_Logger_sendMessage(T* instance, void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_Logger_sendMessage<T, CT>(instance, 0, excb, sentcb);
}

}

#endif