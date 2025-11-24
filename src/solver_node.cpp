#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>
#include <iostream>
#include <thread>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/reset.hpp"

using namespace std::chrono_literals;

class SolverNode : public rclcpp::Node
{
public:
  SolverNode() : Node("solver_node")
  {
    map_client_ = this->create_client<cg_interfaces::srv::GetMap>("get_map");
    move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("move_command");
    reset_client_ = this->create_client<cg_interfaces::srv::Reset>("reset");
  }

  void solve()
  {
    // Wait for services
    while (!map_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) return;
    }
    
    while (!move_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) return;
    }

    while (!reset_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) return;
    }

    auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
    auto result_future = map_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      return;
    }

    auto result = result_future.get();
    int rows = result->occupancy_grid_shape[0];
    int cols = result->occupancy_grid_shape[1];
    const auto& flat_grid = result->occupancy_grid_flattened;

    std::vector<std::vector<std::string>> grid(rows, std::vector<std::string>(cols));
    Point start{-1, -1};
    Point target{-1, -1};

    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        grid[i][j] = flat_grid[i * cols + j];
        if (grid[i][j] == "r") {
          start = {j, i}; // x, y
        } else if (grid[i][j] == "t") {
          target = {j, i};
        }
      }
    }

    if (start.x == -1 || target.x == -1) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Map received. Start: (%d, %d), Target: (%d, %d)", start.x, start.y, target.x, target.y);

    // 1. Calculate Shortest Path using BFS
    std::queue<Point> q;
    q.push(start);
    std::map<Point, Point> came_from;
    std::map<Point, bool> visited_bfs;
    visited_bfs[start] = true;
    
    bool found = false;
    int dx[] = {0, 0, -1, 1};
    int dy[] = {-1, 1, 0, 0}; 
    
    while (!q.empty()) {
      Point current = q.front();
      q.pop();

      if (current == target) {
        found = true;
        break;
      }

      for (int i = 0; i < 4; ++i) {
        Point next = {current.x + dx[i], current.y + dy[i]};
        
        if (next.x >= 0 && next.x < cols && next.y >= 0 && next.y < rows) {
          std::string cell = grid[next.y][next.x];
          if (!visited_bfs[next] && cell != "b") {
            visited_bfs[next] = true;
            came_from[next] = current;
            q.push(next);
          }
        }
      }
    }

    std::vector<std::string> optimal_path_cmds;
    std::vector<Point> optimal_path_points;

    if (found) {
      Point curr = target;
      optimal_path_points.push_back(curr);
      while (!(curr == start)) {
        Point prev = came_from[curr];
        if (curr.x == prev.x && curr.y == prev.y - 1) optimal_path_cmds.push_back("up");
        else if (curr.x == prev.x && curr.y == prev.y + 1) optimal_path_cmds.push_back("down");
        else if (curr.x == prev.x - 1 && curr.y == prev.y) optimal_path_cmds.push_back("left");
        else if (curr.x == prev.x + 1 && curr.y == prev.y) optimal_path_cmds.push_back("right");
        curr = prev;
        optimal_path_points.push_back(curr);
      }
      std::reverse(optimal_path_cmds.begin(), optimal_path_cmds.end());
      std::reverse(optimal_path_points.begin(), optimal_path_points.end());

      RCLCPP_INFO(this->get_logger(), "First task (optimal)");
      
      // Run the optimal path first
      for (const auto& cmd : optimal_path_cmds) {
          move_robot(cmd);
      }
      
      std::cout << "Path taken: ";
      for (const auto& p : optimal_path_points) {
          std::cout << "(" << p.x << "," << p.y << ") ";
      }
      std::cout << std::endl;
      
      // Reset for mapping phase
      auto reset_req = std::make_shared<cg_interfaces::srv::Reset::Request>();
      reset_req->is_random = false;
      auto reset_future = reset_client_->async_send_request(reset_req);
      
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), reset_future) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
          RCLCPP_ERROR(this->get_logger(), "Failed to call reset service");
          return;
      }
      std::this_thread::sleep_for(1s);

    } else {
      return;
    }

    // 2. Execute Mapping using DFS (Physical)
    // This explores the entire maze to "map" it.
    std::vector<std::vector<bool>> visited_dfs(rows, std::vector<bool>(cols, false));
    dfs_explore(start, grid, visited_dfs);
    
    RCLCPP_INFO(this->get_logger(), "Mapping complete!");
    
    // Verification: Check if the mapped area covers the optimal path
    bool path_possible = true;
    
    for (const auto& p : optimal_path_points) {
        if (!visited_dfs[p.y][p.x]) {
            path_possible = false;
        }
    }
    // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str()); // Uncomment to see full path coords

    if (path_possible) {
        RCLCPP_INFO(this->get_logger(), "SUCCESS: The mapped area contains the entire optimal path.");
    } else {
        RCLCPP_INFO(this->get_logger(), "FAILURE: The mapped area does NOT contain the entire optimal path.");
    }
  }

private:
  rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr map_client_;
  rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
  rclcpp::Client<cg_interfaces::srv::Reset>::SharedPtr reset_client_;

  struct Point {
    int x;
    int y;
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
    bool operator<(const Point& other) const { return x < other.x || (x == other.x && y < other.y); }
  };

  bool move_robot(const std::string& direction) {
    auto move_req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    move_req->direction = direction;
    auto move_future = move_client_->async_send_request(move_req);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), move_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call move_command service");
      return false;
    }
    
    auto move_res = move_future.get();
    if (!move_res->success) {
        // RCLCPP_WARN(this->get_logger(), "Failed to move %s", direction.c_str());
        return false;
    }
    
    // Small delay for visualization
    std::this_thread::sleep_for(50ms);
    return true;
  }

  void dfs_explore(Point curr, const std::vector<std::vector<std::string>>& grid, std::vector<std::vector<bool>>& visited) {
    visited[curr.y][curr.x] = true;
    
    // Directions: Up, Down, Left, Right
    // Order matters for traversal shape, but any is valid.
    struct Dir { std::string cmd; int dx; int dy; std::string opp; };
    std::vector<Dir> dirs = {
        {"up", 0, -1, "down"},
        {"down", 0, 1, "up"},
        {"left", -1, 0, "right"},
        {"right", 1, 0, "left"}
    };

    for (const auto& d : dirs) {
        Point next = {curr.x + d.dx, curr.y + d.dy};
        
        // Check bounds
        if (next.y >= 0 && next.y < (int)grid.size() && next.x >= 0 && next.x < (int)grid[0].size()) {
            // Check if valid move (not wall) and not visited
            if (grid[next.y][next.x] != "b" && !visited[next.y][next.x]) {
                // Move forward
                if (move_robot(d.cmd)) {
                    // Recurse
                    dfs_explore(next, grid, visited);
                    // Backtrack
                    move_robot(d.opp);
                }
            }
        }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SolverNode>();
  node->solve();
  rclcpp::shutdown();
  return 0;
}
