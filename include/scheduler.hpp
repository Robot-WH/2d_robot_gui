#pragma once 
#include <memory>
#include "orbit_network.hpp"
#include "ipc/DataDispatcher.hpp"

namespace schedule {
class Scheduler {
public:
    Scheduler() {} 
    /**
     * @brief 遍历任务
     * 
     */
    void Run() {
        static int task_ptr = 0; 
        const auto& node =  orbit_net_ptr_->ReadNode(task_ptr);
        if (task_ptr == orbit_net_ptr_->GetNodeSize() - 1) {
            if (node.path_.empty()) {
                return;
            }
            task_ptr = 0;
        } else {
          task_ptr++;
        }
        std::cout << "Publish TaskPathMsg" << "\n";
        // 发布这个节点的轨迹数据和最终的朝向数据
        ipc::DataDispatcher::GetInstance().Publish("TaskPathMsg", node);
        return; 
    }

    void SetOrbitNetwork(const std::shared_ptr<OrbitNetwork>& orbit_net_ptr) {orbit_net_ptr_ = orbit_net_ptr;}
private:
    std::shared_ptr<OrbitNetwork> orbit_net_ptr_;  
};
}