/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include "Thread.h"

namespace xmate {
class Network;
class UdpProxy;
}  // namespace xmate
class IEvent;

class NetThread : public ThreadBase {
   public:
    virtual ~NetThread(){};
    NetThread(const std::string& name, std::shared_ptr<xmate::Network> sp)
        : ThreadBase(name), sp_net(std::move(sp)), mb_recv(false){};

   public:
    virtual void init(){};
    void registProxy(std::shared_ptr<xmate::UdpProxy> proxy) { sp_proxy = proxy; }
    void registReceivers(std::vector<std::string> recvs) { v_receivers = recvs; }
    void addReceiver(const std::string& rev) { v_receivers.push_back(rev); }
    void registEvent(std::shared_ptr<IEvent> evt) { sp_event = evt; }
    std::shared_ptr<IEvent> getEvent() { return sp_event; }
    void setRecevieState(const bool flag) { mb_recv.store(flag); }

   private:
    virtual void routine() override = 0;

   protected:
    std::shared_ptr<xmate::Network> sp_net;
    std::shared_ptr<xmate::UdpProxy> sp_proxy;
    std::vector<std::string> v_receivers;
    std::shared_ptr<IEvent> sp_event;
    std::atomic_bool mb_recv;
};

class UDPThread final : public NetThread {
   public:
    virtual ~UDPThread(){};
    UDPThread(const std::string& name, std::shared_ptr<xmate::Network> sp) : NetThread(name, sp){};

   private:
    virtual void routine() override;
};

class TCPThread final : public NetThread {
   public:
    virtual ~TCPThread(){};
    TCPThread(const std::string& name, std::shared_ptr<xmate::Network> sp) : NetThread(name, sp){};

   private:
    virtual void routine() override;
};
