#include "ias_drawer_executive/Keywords.h"
#include <string>
#include <ros/ros.h>
#include <tf/tf.h>

Keywords::Keywords(std::string name, double val)
{
    parameters_d[name] = val;
};

Keywords &Keywords::operator()(std::string name, double val)
{
    parameters_d[name] = val;
    return *this;
};

Keywords::Keywords(std::string name, std::string val)
{
    parameters_s[name] = val;
};

Keywords &Keywords::operator()(std::string name, std::string val)
{
    parameters_s[name] = val;
    return *this;
};

Keywords::Keywords(std::string name, tf::Vector3 val)
{
    parameters_v[name] = val;
};

Keywords &Keywords::operator()(std::string name, tf::Vector3 val)
{
    parameters_v[name] = val;
    return *this;
};


Keywords &Keywords::contains()
{
    std::cout << "Contains :" << std::endl;
    {
        std::map<std::string,double>::iterator iter;
        for (iter = parameters_d.begin(); iter != parameters_d.end(); iter++ )
            std::cout << iter->first << ":" << iter->second << std::endl;
    }
    {
        std::map<std::string,std::string>::iterator iter;
        for (iter = parameters_s.begin(); iter != parameters_s.end(); iter++ )
            std::cout << iter->first << ":" << iter->second << std::endl;
    }
    {
        std::map<std::string,tf::Vector3>::iterator iter;
        for (iter = parameters_v.begin(); iter != parameters_v.end(); iter++ )
            std::cout << iter->first << ":" << iter->second.x() << " "  << iter->second.y() << " " << iter->second.z() << std::endl;
    }


    return *this;
};


bool Keywords::has_d(const std::string &key) const
{
    if (parameters_d.find(key) != parameters_d.end())
        return true;
    else
        return false;
};

bool Keywords::has_s(const std::string &key) const
{
    if (parameters_s.find(key) != parameters_s.end())
        return true;
    else
        return false;
};

bool Keywords::has_v(const std::string &key) const
{
    if (parameters_v.find(key) != parameters_v.end())
        return true;
    else
        return false;
};

double Keywords::lookup_d(const std::string &key) const
{
    std::map<std::string,double>::const_iterator it = parameters_d.find(key);
    if (it != parameters_d.end())
        return it->second; // we cant use map[key] since its a modifier
    else return 0;
    // else throw ...
};

std::string Keywords::lookup_s(const std::string &key) const
{
    std::map<std::string,std::string>::const_iterator it = parameters_s.find(key);
    if (it != parameters_s.end())
        return it->second;
    else return "";
    // else throw ...
};

tf::Vector3 Keywords::lookup_v(const std::string &key) const
{
    std::map<std::string,tf::Vector3>::const_iterator it = parameters_v.find(key);
    if (it != parameters_v.end())
        return it->second;
    else return tf::Vector3(0,0,0);
    // else throw ...
};

KeyValPair Keywords::lookup(const std::string &key) const
{
    KeyValPair kvp;
    {
        std::map<std::string,tf::Vector3>::const_iterator it = parameters_v.find(key);
        if (it != parameters_v.end())
            kvp.v = it->second;
    }
    {
        std::map<std::string,std::string>::const_iterator it = parameters_s.find(key);
        if (it != parameters_s.end())
            kvp.s = it->second;
    }
    {
        std::map<std::string,double>::const_iterator it = parameters_d.find(key);
        if (it != parameters_d.end())
            kvp.d = it->second;
    }
    return kvp;
}



