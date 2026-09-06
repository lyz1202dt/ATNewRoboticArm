#pragma once

#include <string>


template <typename T>
inline constexpr char _tid_var = 0;

template <typename T>
constexpr const void* type_id() noexcept {
    return &_tid_var<T>;                 // 变量地址即类型身份
}

class PortBase{
public:
    PortBase(std::string name):name(name){};
    virtual ~PortBase() = default;
    virtual const void* port_type_id() const = 0;
    std::string name;
};

class InputPortBase : public virtual PortBase{
public:
    InputPortBase(std::string name):PortBase(name){};
    virtual void bind(void *p)=0;
};

class OutputPortBase : public virtual PortBase{
public:
    OutputPortBase(std::string name):PortBase(name){};
    virtual void * get_value_raw_ptr() const = 0;
};

template <class T>
class Typed :  public virtual PortBase {
public:
    explicit Typed(const std::string& name): PortBase(name){}
    const void* port_type_id() const noexcept override { return type_id<T>(); }
};


template <typename T>
class OutputPort : public virtual Typed<T>, public virtual OutputPortBase{
public:
    explicit OutputPort(const std::string &name, T* value_ptr = nullptr)
        : PortBase(name), Typed<T>(name), OutputPortBase(name), p_value(value_ptr){};
    void *  get_value_raw_ptr() const override { return static_cast<void*>(p_value); }
    T* p_value{nullptr};
};

template <typename T>
class InputPort : public virtual Typed<T>,public virtual InputPortBase{
public:
    explicit InputPort(const std::string &name)
        : PortBase(name), Typed<T>(name), InputPortBase(name){};
    void bind(void *p) override {p_value=p;};
    T * get_value_ptr() const { return static_cast<T*>(p_value); }
    void *p_value{nullptr};
};

