#pragma once

#include <string>
#include <any>

class FSMFactory;

class FSM {
public:
    FSM(const std::string& fsm_name, std::any ctx)
        : fsm_name_(fsm_name)
        , ctx_(ctx) {}

    virtual ~FSM() = default;

    const std::string& get_name() const { return fsm_name_; }

    virtual bool enter(const std::string& last_state) {
        (void)last_state;
        return true;
    }

    virtual bool exit(const std::string& next_state) {
        (void)next_state;
        return true;
    }

    virtual std::string check_switch() const {
        return fsm_name_;
    }

    virtual bool run() {
        return true;
    }

protected:
    std::string fsm_name_;
    std::any ctx_;

private:
    
};