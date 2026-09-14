#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>

#include "fsm.hpp"

class FSMFactory {
public:
    bool register_fsm(FSM* fsm) {
        if (fsm == nullptr) {
            return false;
        }
        fsm_map.emplace(fsm->get_name(), fsm);
        return true;
    }
    bool set_init_state(const std::string& name) {
        current_fsm   = fsm_map[name];
        return true;
    }
    bool run() {
        if(first_run)
        {
            first_run=false;
            current_fsm->enter("");
            return true;
        }

        if (state_switch) {
            bool switch_success=current_fsm->exit(next_fsm->get_name());
            if(!switch_success)
                return false;
            switch_success=next_fsm->enter(current_fsm->get_name());
            if(!switch_success)
                return false;
            current_fsm=next_fsm;
            state_switch=false;
        } else {
            bool success = current_fsm->run();
            if (!success)
                return false;
            const std::string next_state = current_fsm->check_switch();
            if(next_state!=current_fsm->get_name())
            {
                next_fsm  = fsm_map[next_state];
                state_switch=true;
            } 
        }

        return true;
    }

private:
    bool first_run{true};
    bool state_switch{false};
    FSM* next_fsm{nullptr};
    FSM* current_fsm{nullptr};
    std::string last_state_name;
    std::unordered_map<std::string, FSM*> fsm_map;
};