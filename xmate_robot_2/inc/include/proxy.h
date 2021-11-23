/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace xmate {
class Goblin {
   public:
    using func = std::function<void(char*)>;
    Goblin() : m_name("") { init(); }
    explicit Goblin(const std::string& str) : m_name(str) { init(); }
    virtual ~Goblin() {}

   public:
    virtual void Notify(const std::vector<std::string>& task, char* p) = 0;
    const std::string getName() const { return m_name; }
    void Register(std::string func_key, func f) { m_taskmap.emplace(func_key, f); }
    void Erase(std::string func_key) { m_taskmap.erase(m_taskmap.find(func_key)); }

   private:
    void init() { m_taskmap.clear(); };

   protected:
    std::unordered_map<std::string, func> m_taskmap;

   private:
    std::string m_name;
};

class UdpProxy : public Goblin {
   public:
    explicit UdpProxy(const std::string& str) : Goblin(str){};
    virtual ~UdpProxy(){};

   public:
    virtual void Notify(const std::vector<std::string>& task, char* p) override;
};

} /* namespace xmate */
