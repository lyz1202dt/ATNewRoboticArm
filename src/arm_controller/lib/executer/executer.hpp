#pragma once

#include "port.hpp"
#include "component.hpp"

#include <vector>
#include <unordered_map>
#include <queue>
#include <functional>

class Executer {
public:
    Executer()=default;
    void register_component(Component* component){
        components.push_back(component);
    };
    bool build(){

        std::unordered_map<Component*, int> indeg;
        std::unordered_map<Component*,std::vector<Component*>> components_childs;

        //执行init，让组件声明依赖关系
        for(auto component : components){
            if(!component->init()){
                return false;
            }
            indeg.emplace(component,0);
            components_childs.emplace(component,std::vector<Component*>());
        }
 
        //遍历每一个节点，连接outport和inport，通过去重构建父子关系，同时通过对输入输出端口去重，得到唯一父子关系
        for(auto parent : components){
            for(auto& out_port : parent->out_ports){
                for(auto child : components){
                    for(auto& in_port :child->in_ports){
                        if(out_port->name == in_port->name && out_port->port_type_id() == in_port->port_type_id()){
                            in_port->bind(out_port->get_value_raw_ptr());

                            //根据端口关系，去重后构建节点间父子关系
                            bool added_child=false;
                            for(auto c: components_childs[parent])
                            {
                                if(c==child)   //该子节点已经被加入过
                                    added_child=true;
                            }
                            if(!added_child)   //没有加入过才需要增加
                            {
                                components_childs[parent].push_back(child);
                                indeg[child]++; //更新子节点的入度
                            }
                        }
                    }
                }
            }
        }

        execution_order.clear();
        std::queue<Component*> current_queue;
    
        // 初始化：所有入度为0的节点
        for (auto& pair : indeg) {
            if (pair.second == 0) {
                current_queue.push(pair.first);
            }
        }
    
        // 分层处理
        while (!current_queue.empty()) {
            std::vector<Component*> current_layer;
            std::queue<Component*> next_queue;
        
            // 收集当前层的所有节点
            while (!current_queue.empty()) {
                current_layer.push_back( current_queue.front());
                current_queue.pop();
            }
        
            // 将当前层加入执行顺序
            execution_order.push_back(current_layer);
        
            // 处理当前层的所有子节点
            for (auto node : current_layer) {
                for (auto child : components_childs[node]) {
                    indeg[child]--;
                    if (indeg[child] == 0) {
                        next_queue.push(child);
                    }
                }
            }
            
            current_queue = std::move(next_queue);
        }
    
        // 检查是否有环
        size_t total_components = 0;
        for (const auto& layer : execution_order) {
            total_components += layer.size();
        }
        
        if (total_components != components.size()) {
            execution_order.clear();
            return false;  // 存在循环依赖
        }

        ready_ = true;
        return true;
    };

    bool run(){
        if(!ready_)
            return false;
        
        for(auto components : execution_order)
        {
            for(auto component : components)
            {
                component->task();
            }
        }
        
        return true;
    };

    bool run_mult_threading(){
        if(!ready_ || !thread_func_)
            return false;
        
        for(auto& layer : execution_order)          //可以同时运行的任务，多线程派发，同时执行
        {
            if(layer.empty()) continue;
            for(size_t i=1;i<layer.size();i++)
                new_task_start_func_(layer[i]);
            layer[0]->task();
            for(size_t i=1;i<layer.size();i++)
                new_task_join_func_(layer[i]);
        }
        
        return true;
    };

    void register_new_task_func(std::function<void(Component*)> start_func,std::function<void(Component*)> join_func){    //注册快速启动子线程的API（推荐线程池实现）
        new_task_start_func_ =start_func;
        new_task_join_func_=join_func;
        thread_func_=true;
    }

private:
    bool ready_{false};
    bool thread_func_{false};
    std::function<void(Component*)> new_task_start_func_;
    std::function<void(Component*)> new_task_join_func_;
    std::vector<Component*> components;
    std::vector<std::vector<Component*>> execution_order;
};