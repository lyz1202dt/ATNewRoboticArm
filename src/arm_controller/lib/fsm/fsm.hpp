#pragma once

#include <string>

class FSMFactory;

class FSM {
public:
    FSM(const std::string& fsm_name, FSMFactory* factory)
        : fsm_name_(fsm_name)
        , factory_(factory) {}

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
    FSMFactory* factory() const { return factory_; }

private:
    std::string fsm_name_;
    FSMFactory* factory_;
};