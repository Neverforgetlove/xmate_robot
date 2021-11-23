/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd. 
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file json_helper.hpp
 * @brief 提供Json数据转换的帮助函数
 */

#pragma once

#include <iostream>
#include <string>
#include <json/json.h>

namespace json {
/**
 * @brief 检查数据的有效性及获取json的key的内容
 */
inline bool GetDouble(const Json::Value& value, const std::string& key, double& d) {
    if(value.isNull()) return false;
    //Int可转为double
    if(!value[key].isDouble() && !value[key].isInt()) return false;
    d = value[key].asDouble();
    return true;
}

inline bool GetUInt(const Json::Value& value, const std::string& key, unsigned int& i) {
    if(value.isNull()) return false;
    if(!value[key].isUInt()) return false;
    i = value[key].asUInt();
    return true;
}

inline bool GetInt(const Json::Value& value, const std::string& key, int& i) {
    if(value.isNull()) return false;
    if(!value[key].isInt()) return false;
    i = value[key].asInt();
    return true;
}

inline bool GetBool(const Json::Value& value, const std::string& key, bool& b) {
    if(value.isNull()) return false;
    if(!value[key].isBool()) return false;
    b = value[key].asBool();
    return true;
}

inline bool GetFloat(const Json::Value& value, const std::string& key, float& b) {
    if(value.isNull()) return false;
    if(!value[key].isDouble()) return false;
    b = value[key].asFloat();
    return true;
}

inline bool GetString(const Json::Value& value, const std::string& key, std::string& s) {
    if(value.isNull()) return false;
    if(!value[key].isString()) return false;
    s = value[key].asString();
    return true;
}

inline bool GetValue(const Json::Value& value, const std::string& key, Json::Value& obj) {
    if(value.isNull()) return false;
    if(!value[key].isObject()) return false;
    obj = value[key];
    return true;
}

inline bool GetArray(const Json::Value& value, const std::string& key, Json::Value& arr) {
    if(value.isNull()) return false;
    if(!value[key].isArray()) return false;
    arr = value[key];
    return true;
}

inline bool GetDoubleArray(const Json::Value& value, const std::string& key, std::vector<double>& vec) {
    if(value.isNull()) return false;

    if(!value[key].isArray()) return false;

    const auto &array = value[key];
    for(Json::ArrayIndex i = 0; i < array.size(); i++){
        if(!array[i].isDouble()) return false;
        vec.push_back(array[i].asDouble());
    }
    return true;
}

inline bool GetStringVec(const Json::Value& value, const std::string& key, std::vector<std::string>& vec) {
    if (value.isNull()) return false;

    if (!value[key].isArray()) return false;
   
    if (!int(value[key].size())) return false;

    for (unsigned int i = 0; i < value[key].size(); i++) {
        if (!value[key][i].isString()) return false;
        vec.push_back(value[key][i].asString());
    }
    return true;
}

inline void SetArray(Json::Value& root, const std::array<double,7> &data) {
    //仅支持写入到不存在的key值或者数组类型
    if(!root.empty() && !root.isArray()){
        throw std::runtime_error("Cannot write to a existing object!");
    }
    for (std::size_t i = 0; i < data.size(); ++i) {
        root[static_cast<int> (i)] = data[i];
    }
}

inline bool ParseJson(const std::string &str, Json::Value &value){
    JSONCPP_STRING errs;
    Json::CharReaderBuilder readerBuilder;

    std::unique_ptr<Json::CharReader> const jsonReader(readerBuilder.newCharReader());

    if (!jsonReader->parse(str.c_str(), str.c_str() + str.length(), &value, &errs) || !errs.empty()){
        std::cout << "parseJson err. " << errs<<std::endl;
        return false;
    }
    return true;
} 

inline std::string WriteJson(const Json::Value& value){
    //序列化json
    std::ostringstream str;
    Json::StreamWriterBuilder writerBuilder;
    std::unique_ptr<Json::StreamWriter> jsonWriter(writerBuilder.newStreamWriter());
    jsonWriter->write(value, &str);
    return str.str();
}

} // namespace json