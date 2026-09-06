#include <iostream>
#include <thread>
#include <map>
#include <mutex>
#include <vector>
#include <string>

#include "executer.hpp"

// 测试组件：init 中声明输入/输出端口，task 中打印自身编号
class TestComponent : public Component {
public:
    TestComponent(std::string name, int id,
                  std::vector<std::string> in_names = {},
                  std::vector<std::string> out_names = {})
        : id_(id), in_names_(std::move(in_names)), out_names_(std::move(out_names)) {
        this->name = std::move(name);
    }

    bool init() override {
        for (auto& n : out_names_) {
            add_out_port<std::string>(n);
        }
        // 预留容量，避免 vector 扩容导致 InPort 指针失效
        in_storage_.reserve(in_names_.size());
        for (auto& n : in_names_) {
            in_storage_.emplace_back("in");
            add_in_port(n, &in_storage_.back());
        }
        return true;
    }

    void task() override {
        std::cout << id_ << " ";
    }

private:
    int id_;
    std::vector<std::string> in_names_;
    std::vector<std::string> out_names_;
    std::vector<InputPort<std::string>> in_storage_;  // 持有 InPort 实例，保证指针有效
};

// std::thread 简单包装：start 启动子线程执行 task，join 等待其完成
class ThreadWrapper {
public:
    void start(Component* c) {
        std::lock_guard<std::mutex> lk(mtx_);
        pool_.emplace(c, std::thread([c] { c->task(); }));
    }

    void join(Component* c) {
        std::thread t;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            auto it = pool_.find(c);
            if (it != pool_.end()) {
                t = std::move(it->second);
                pool_.erase(it);
            }
        }
        if (t.joinable()) {
            t.join();
        }
    }

private:
    std::map<Component*, std::thread> pool_;
    std::mutex mtx_;
};

int main() {
    // 构建一个有向无环图 (DAG)：
    //   A(1) --a--> C(3) --c--> E(5)
    //   B(2) --b--> D(4) --d--> E(5)
    // 拓扑分层为：[A, B] -> [C, D] -> [E]
    TestComponent A("A", 1, {}, {"a"});
    TestComponent B("B", 2, {}, {"b"});
    TestComponent C("C", 3, {"a"}, {"c"});
    TestComponent D("D", 4, {"b"}, {"d"});
    TestComponent E("E", 5, {"c", "d"}, {});

    Executer exe;
    exe.register_component(&A);
    exe.register_component(&B);
    exe.register_component(&C);
    exe.register_component(&D);
    exe.register_component(&E);

    // 测试 1：构建（应成功）
    std::cout << "build: " << (exe.build() ? "ok" : "fail") << std::endl;

    // 测试 2：单线程按拓扑序执行
    // 层间严格有序（5 一定最后，3 在 1 后、4 在 2 后）；
    // 层内顺序由 unordered_map 遍历顺序决定，1/2、3/4 可能互换
    std::cout << "run(single): ";
    exe.run();
    std::cout << std::endl;

    // 测试 3：多线程执行（层内并行，层间保序；层内 1/2、3/4 顺序可能互换）
    ThreadWrapper tw;
    exe.register_new_task_func(
        [&tw](Component* c) { tw.start(c); },
        [&tw](Component* c) { tw.join(c); }
    );
    std::cout << "run(multi): ";
    exe.run_mult_threading();
    std::cout << std::endl;

    // 测试 4：循环依赖检测（应 fail）
    TestComponent F("F", 6, {"y"}, {"x"});
    TestComponent G("G", 7, {"x"}, {"y"});
    Executer exe_cycle;
    exe_cycle.register_component(&F);
    exe_cycle.register_component(&G);
    std::cout << "build(cycle): " << (exe_cycle.build() ? "ok" : "fail") << std::endl;

    return 0;
}