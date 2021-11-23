/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once
#include <pthread.h>

#include <string>

class ThreadBase {
   public:
    ThreadBase() : tid(0), m_name(""){};
    explicit ThreadBase(const std::string& name) : tid(0), m_name(name){};
    virtual ~ThreadBase(){};

   public:
    //获取线程id，便于对线程行为进行控制
    pthread_t GetTid();
    //默认的工作流
    void runflow();

   private:
    //启动线程
    virtual int start();
    //线程的运行函数，由派生类实现
    virtual void routine() = 0;
    //比较线程号是否相等
    virtual int equal(pthread_t t);
    //分离线程
    virtual int detach();
    //连接线程
    virtual int join(pthread_t t) noexcept;
    //线程退出
    virtual void exit() noexcept;
    //取消线程
    virtual int cancel(pthread_t t);
    //销毁线程
    virtual int destroy();

   protected:
    const std::string& getThreadName() const { return m_name; }

   private:
    //线程清理函数
    static void cleaner(void* pHandle);
    //线程回调函数
    static void* work(void* pHandle);

   private:
    ThreadBase(const ThreadBase& cls) = delete;
    ThreadBase& operator=(const ThreadBase& cls) = delete;

   private:
    //线程属性
    pthread_attr_t t_attr;
    //线程号
    pthread_t tid;
    //线程名称
    std::string m_name;
};
