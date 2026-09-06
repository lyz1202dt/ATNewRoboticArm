#pragma once

#include "port.hpp"
#include  <vector>

class Component{
public:
    Component() = default;
    virtual ~Component() = default;
    virtual bool init(){ return true; }
    virtual void task(){}

    template <typename T>
    void add_in_port(std::string name, InputPort<T>* port){
        port->name = name;
        in_ports.push_back(port);
    }
    template <typename T>
    void add_out_port(std::string name,T* value_ptr=nullptr){
        out_ports.emplace_back(new OutputPort<T>(name, value_ptr));
    };
    std::string name;
    std::vector<InputPortBase*> in_ports;
    std::vector<OutputPortBase*> out_ports;
};
