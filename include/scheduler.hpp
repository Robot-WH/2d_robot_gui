#pragma once 
#include <memory>
#include "orbit_network.hpp"
#include "ipc/DataDispatcher.hpp"
namespace schedule {
class Scheduler {
public:
    Scheduler() {} 
    void Restart() {
      task_ptr_ = 0;
      Run();
    }
    /**
     * @brief 遍历任务
     * 
     */
    void Run() {
        std::cout << "执行任务 --- " << task_ptr_ << "\n";
        const auto& node =  orbit_net_ptr_->ReadNode(task_ptr_);
        if (task_ptr_ == orbit_net_ptr_->GetNodeSize() - 1) {
            if (node.path_.empty()) {
                return;
            }
            task_ptr_ = 0;
        } else {
          task_ptr_++;
        }
        const auto& next_node =  orbit_net_ptr_->ReadNode(task_ptr_);
        // 发布这个节点的轨迹数据和最终的朝向数据
        ipc::DataDispatcher::GetInstance().Publish("TaskPathMsg", std::make_pair(node, next_node.state_yaw_));
        return; 
    }

    void SetOrbitNetwork(const std::shared_ptr<OrbitNetwork>& orbit_net_ptr) {orbit_net_ptr_ = orbit_net_ptr;}
private:
    int task_ptr_ = 0;
    std::shared_ptr<OrbitNetwork> orbit_net_ptr_;  
};
}
