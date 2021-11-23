
/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include <netinet/in.h>  //sockaddr_in
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "rci_data/command_traits.h"
#include "xmate_exception.h"

using namespace std::chrono_literals;

namespace xmate {

class Network {
   public:
    Network(const std::string &ip_addr, uint16_t port);
    ~Network();

   public:
    int connectRobot();
    void resetConnet() { m_connect_state = false; }

    template <typename T>
    void udpSend(T data) {
        m_udp_send++;
        try {
            std::lock_guard<std::mutex> lock(udp_lock);
            data.HostToNet();
            auto send_num = sendto(sock_udp, &data, sizeof(T), 0, (struct sockaddr *)&addr_serv, sizeof(addr_serv));

            if (send_num != sizeof(T)) {
                throw UdpException("udp发送数据长度与目标对象不匹配！");
            }
        } catch (const std::exception &e) {
            throw UdpException("udp发送时发生错误：" + std::string(e.what()));
        } catch (...) {
            throw UdpException("udp发送时发生未知错误！");
        }
    }

    template <typename T>
    bool udpReceive(T &data) {
        m_udp_rec++;
        last_time_ = current_time_;
        clock_gettime(CLOCK_REALTIME, &current_time_);
        long long time_inter = current_time_.tv_nsec - last_time_.tv_nsec;
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000;
        if (setsockopt(sock_udp, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
            perror("udp timeout");
        }
        try {
            std::lock_guard<std::mutex> lock(udp_lock);
            auto recv_num = recvfrom(sock_udp, &data, sizeof(T), 0, (struct sockaddr *)&udp_recv,
                                     static_cast<socklen_t *>(&link_len));
            robot_state_now_ = data;
            if(robot_state_last_.message_id == robot_state_now_.message_id){
                std::cout<<"这一周期没有新的数据"<<std::endl;
            }
            if (recv_num == sizeof(T)) {
                data.NetToHost();
                robot_state_last_ = robot_state_now_;
                m_udp_received++;
                return true;
            }else{
                std::cout<<"没收到这一周期数据"<<std::endl;
            }

            if (recv_num == -1) {
                data = robot_state_last_;
                data.NetToHost();
                std::cout<<"这一周期与上一周期重合"<<std::endl;
                return false;
            }
            if (recv_num != sizeof(T)) {
                throw UdpException("udp接收数据长度与目标对象不匹配！");
            }
        } catch (const std::exception &e) {
            throw UdpException("udp接收时发生错误：" + std::string(e.what()));
        } catch (...) {
            throw UdpException("udp接收时发生未知错误！");
        }

        robot_state_last_ = robot_state_now_;

        return true;
    }

    template <typename T /*, typename... TArgs*/>
    uint32_t tcpSendRequest(T &data /*TArgs&&... args*/) {
        if (!m_connect_state) throw TcpException("tcp连接未建立，不能发送数据");
        try {
            std::lock_guard<std::mutex> lock(tcp_lock);
            auto commu_msg = RCI::robot::TcpMsg<T>(data);
            commu_msg.len = sizeof(commu_msg);
            commu_msg.command_id = ++_cmd_id;  //++command_id_;
            commu_msg.HostToNet();
            send(sock_tcp, &commu_msg, sizeof(commu_msg), 0);
            std::cout <<std::string(RCI::robot::CommandTraits<T>::kName) << std::endl;
            return _cmd_id;
        } catch (const std::exception &e) {
            throw TcpException("tcp发送时发生错误：" + std::string(e.what()));
        } catch (...) {
            throw TcpException("tcp发送时发生未知错误！");
        }
    }

    template <typename R>
    R tcpBlockingReceiveResponse(uint32_t cmd_id) {
        // using std::literals::chrono_literals::operator""ms;
        std::unique_lock<std::mutex> lock(tcp_lock, std::defer_lock);
        decltype(tcp_respons_map)::const_iterator it;
        do {
            lock.lock();
            tcpReadFromBuffer<R>(10ms);
            it = tcp_respons_map.find(cmd_id);
            lock.unlock();
            std::this_thread::yield();
        } while (it == tcp_respons_map.end());
        it->second.data();
        R reply = *reinterpret_cast<const R *>(it->second.data());
        tcp_respons_map.erase(it);
        std::cout<<std::string(RCI::robot::CommandTraits<R>::kName)<<std::endl;
        return reply;
    }

    template <typename R>
    void tcpReadFromBuffer(std::chrono::microseconds timeout) {
        try {
            using PKG = RCI::robot::TcpMsg<R>;
            auto len = read(sock_tcp, respons_bytes.data(), sizeof(PKG));
            if (len < 10) {
                throw TcpException("tcp接收命令响应长度错误");
            }
            PKG *pbuf = reinterpret_cast<decltype(pbuf)>(respons_bytes.data());
            pbuf->NetToHost();
            auto _id = pbuf->command_id;
            memcpy(respons_bytes.data(), respons_bytes.data() + sizeof(PKG) - sizeof(R), sizeof(R));
            tcp_respons_map.emplace(_id, respons_bytes);
        } catch (const std::exception &e) {
            throw TcpException("tcp接收命令响应时发生错误：" + std::string(e.what()));
        } catch (...) {
            throw TcpException("tcp接收命令响应时发生未知错误！");
        }
    }

    template <typename R, typename T>
    R tcpCmdBlockHandshake(T &data) {
        auto id = tcpSendRequest(data);
        return tcpBlockingReceiveResponse<R>(id);
    }

   private:
    int sock_udp;
    int sock_tcp;

    std::mutex udp_lock;
    std::mutex tcp_lock;

    timespec last_time_;
    timespec current_time_;

    struct sockaddr_in addr_serv;
    struct sockaddr_in udp_recv;

    socklen_t link_len;
    bool m_connect_state;
    uint32_t _cmd_id;
    std::unordered_map<uint32_t, std::array<char, 1024>> tcp_respons_map;
    std::array<char, 1024> respons_bytes;

    RCI::robot::RobotState robot_state_last_, robot_state_now_;
    double m_udp_rec;
    double m_udp_received;
    double m_udp_send;
};

}  // namespace xmate
