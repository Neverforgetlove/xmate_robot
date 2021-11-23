/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#pragma once

#include <pthread.h>

#include <memory>
#include <string>

class IEvent {
   public:
    /*
     动态方式初始化互斥锁,初始化状态变量m_cond
    `bAutoReset  true   人工重置
                 false  自动重置
    */
    IEvent(const std::string& str, const bool bman);

    /*
     注销互斥锁,注销状态变量m_cond
    */
    ~IEvent();

    /*
     将当前事件对象设置为有信号状态
     若自动重置，则等待该事件对象的所有线程只有一个可被调度
     若人工重置，则等待该事件对象的所有线程变为可被调度
    */
    void SetEvent();

    /*
    将当前事件对象设置为无信号状态
    */
    void ResetEvent();

    /*
     以当前事件对象，阻塞线程，将其永远挂起
     直到事件对象被设置为有信号状态
    */
    bool Wait();

    /*
     以当前事件对象，阻塞线程，将其挂起指定时间间隔
     之后线程自动恢复可调度
    */
    bool Wait(unsigned long milliseconds);

   private:
    IEvent(const IEvent& cls) = delete;
    IEvent& operator=(const IEvent& cls) = delete;

   private:
    std::string m_EvName;
    bool m_bManual;
    volatile bool m_bState;
    pthread_mutex_t m_mutex;
    pthread_cond_t m_cond;
};
