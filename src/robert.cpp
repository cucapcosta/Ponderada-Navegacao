// Ros pra cpp + comandos basicos do jogo
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/reset.hpp"
#include <algorithm>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
using namespace std;
string direcoes[4] = {"up", "down", "left", "right"};

struct mapa
{
    vector<vector<string>> grid;
    pair<int, int> posicao_inicial;
    pair<int, int> posicao_objetivo;
    // Vou fazer essa matriz flattened mesmo, preguiça ter que lidar, e provavelmente só vou usar as 4 direções principais
    vector<string> arredores;
    bool valido = false;
};

vector<pair<int, int>> adicionarArredores(const pair<int, int> &pos, const mapa &map)
{
    vector<pair<int, int>> arredores;

    const int row = pos.first;
    const int col = pos.second;
    const int rows = static_cast<int>(map.grid.size());
    const int cols = static_cast<int>(map.grid[0].size());

    // Up
    if (row > 0 && map.grid[row - 1][col] != "b")
    {
        arredores.push_back({row - 1, col});
    }
    // Down
    if (row < rows - 1 && map.grid[row + 1][col] != "b")
    {
        arredores.push_back({row + 1, col});
    }
    // Left
    if (col > 0 && map.grid[row][col - 1] != "b")
    {
        arredores.push_back({row, col - 1});
    }
    // Right
    if (col < cols - 1 && map.grid[row][col + 1] != "b")
    {
        arredores.push_back({row, col + 1});
    }

    return arredores;
}

string direcaoEntre(const pair<int, int> &origem, const pair<int, int> &destino)
{
    if (destino.first < origem.first)
    {
        return "up";
    }
    if (destino.first > origem.first)
    {
        return "down";
    }
    if (destino.second < origem.second)
    {
        return "left";
    }
    if (destino.second > origem.second)
    {
        return "right";
    }
    return "";
}

bool executarMovimentos(const rclcpp::Node::SharedPtr &node, const vector<string> &comandos)
{
    if (comandos.empty())
    {
        RCLCPP_INFO(node->get_logger(), "Nenhum movimento necessário: já estamos no objetivo");
        return true;
    }

    auto client = node->create_client<cg_interfaces::srv::MoveCmd>("move_command");
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(node->get_logger(), "Servico move_command indisponivel");
        return false;
    }

    for (const auto &direcao : comandos)
    {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direcao;
        auto future = client->async_send_request(request);
        const auto status = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(2));
        if (status != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Falha ao enviar comando de movimento: %s", direcao.c_str());
            return false;
        }
        auto response = future.get();
        if (!response)
        {
            RCLCPP_ERROR(node->get_logger(), "Resposta nula do move_command");
            return false;
        }
        if (!response->success)
        {
            RCLCPP_WARN(node->get_logger(), "Movimento %s rejeitado pelo jogo", direcao.c_str());
            return false;
        }
        const auto robo_x = static_cast<int>(response->robot_pos[0]);
        const auto robo_y = static_cast<int>(response->robot_pos[1]);
        RCLCPP_INFO(node->get_logger(), "Movido %s para (%d, %d)", direcao.c_str(), robo_x, robo_y);
        if (response->robot_pos[0] == response->target_pos[0] && response->robot_pos[1] == response->target_pos[1])
        {
            RCLCPP_INFO(node->get_logger(), "Objetivo alcançado durante execucao dos comandos!");
            return true;
        }
        //Deve-se incluir essa linha se for um requerimento ENXERGAR o robô se mexendo
        //Caso contrario, o jogo é resolvido quase instantaneamente
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return true;
}

void resetGame(const rclcpp::Node::SharedPtr &node)
{
    auto client = node->create_client<cg_interfaces::srv::Reset>("reset_game");
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(node->get_logger(), "Servico reset_game indisponivel");
        return;
    }

    auto request = std::make_shared<cg_interfaces::srv::Reset::Request>();
    request->is_random = false;
    request->map_name.clear();
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Falha ao chamar o servico reset_game");
        return;
    }

    auto response = future.get();
    if (!response)
    {
        RCLCPP_ERROR(node->get_logger(), "Resposta nula do reset_game");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Jogo reiniciado com sucesso");
}

mapa getMap(const rclcpp::Node::SharedPtr &node)
{
    mapa resultado;
    auto client = node->create_client<cg_interfaces::srv::GetMap>("get_map");
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(node->get_logger(), "get_map service not available");
        return resultado;
    }

    auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future, std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call get_map service");
        return resultado;
    }

    auto response = result_future.get();
    if (!response)
    {
        RCLCPP_ERROR(node->get_logger(), "get_map returned null response");
        return resultado;
    }

    if (response->occupancy_grid_shape.size() < 2)
    {
        RCLCPP_ERROR(node->get_logger(), "invalid occupancy_grid_shape");
        return resultado;
    }

    const size_t linhas = response->occupancy_grid_shape[0];
    const size_t colunas = response->occupancy_grid_shape[1];
    vector<vector<string>> grid(linhas, vector<string>(colunas));

    const size_t total_celulas = linhas * colunas;
    // Nota para si mesmo: Usa min para não preencher a mensagem alem do tamanho do vetor
    const size_t preenchimento = min(total_celulas, response->occupancy_grid_flattened.size());
    for (size_t idx = 0; idx < preenchimento; ++idx)
    {
        const size_t linha = idx / colunas;
        const size_t coluna = idx % colunas;
        grid[linha][coluna] = response->occupancy_grid_flattened[idx];
    }
    resultado.grid = grid;
    pair<int, int> pos_robo = {-1, -1};
    pair<int, int> pos_objetivo = {-1, -1};
    for (size_t linha = 0; linha < linhas; ++linha)
    {
        for (size_t coluna = 0; coluna < colunas; ++coluna)
        {
            const string &celula = resultado.grid[linha][coluna];
            if (celula == "r")
            {
                pos_robo = {static_cast<int>(linha), static_cast<int>(coluna)};
            }
            else if (celula == "t")
            {
                pos_objetivo = {static_cast<int>(linha), static_cast<int>(coluna)};
            }
            if (pos_robo.first != -1 && pos_objetivo.first != -1)
            {
                break;
            }
        }
        if (pos_robo.first != -1 && pos_objetivo.first != -1)
        {
            break;
        }
    }
    resultado.posicao_inicial = pos_robo;
    resultado.posicao_objetivo = pos_objetivo;
    resultado.valido = true;
    return resultado;
}

void fullMapSolver(const rclcpp::Node::SharedPtr &node)
{
    // Vamos, no primeiro desafio, encontrar o melhor caminho até o robô
    // Tentar implementar isso por BFS, vamos ver se dá certo
    // Lembra q é fifo desgraça
    auto mapa_completo = getMap(node);
    if (!mapa_completo.valido)
    {
        RCLCPP_ERROR(node->get_logger(), "Falha ao obter mapa completo");
        return;
    }
    const auto pos_robo = mapa_completo.posicao_inicial;
    const auto pos_objetivo = mapa_completo.posicao_objetivo;

    const int linhas = static_cast<int>(mapa_completo.grid.size());
    const int colunas = static_cast<int>(mapa_completo.grid[0].size());
    printf("Posição do robô: (%d, %d)\n", pos_robo.first, pos_robo.second);
    printf("Posição do objetivo: (%d, %d)\n", pos_objetivo.first, pos_objetivo.second);
    printf("Mapa recebido:\n");
    for (const auto &linha : mapa_completo.grid)
    {
        for (const auto &celula : linha)
        {
            printf("%s ", celula.c_str());
        }
        printf("\n");
    }
    vector<vector<pair<int, int>>> parent(linhas, vector<pair<int, int>>(colunas, {-1, -1}));
    vector<vector<bool>> visitados(linhas, vector<bool>(colunas, false));
    vector<pair<int, int>> fila;
    size_t head = 0;
    fila.push_back(pos_robo);
    visitados[pos_robo.first][pos_robo.second] = true;
    bool encontrou = false;
    while (head < fila.size())
    {
        auto atual = fila[head++];
        if (atual == pos_objetivo)
        {
            encontrou = true;
            break;
        }
        for (const auto &vizinho : adicionarArredores(atual, mapa_completo))
        {
            if (!visitados[vizinho.first][vizinho.second])
            {
                visitados[vizinho.first][vizinho.second] = true;
                parent[vizinho.first][vizinho.second] = atual;
                fila.push_back(vizinho);
            }
        }
    }
    if (!encontrou)
    {
        RCLCPP_WARN(node->get_logger(), "Objetivo inalcançável a partir da posição atual");
        return;
    }

    vector<pair<int, int>> caminho;
    for (auto passo = pos_objetivo; passo.first != -1 && passo.second != -1; passo = parent[passo.first][passo.second])
    {
        caminho.push_back(passo);
        if (passo == pos_robo)
        {
            break;
        }
    }
    reverse(caminho.begin(), caminho.end());
    printf("Caminho encontrado com %zu passos:\n", caminho.size());
    pair<int, int> passo_anterior = caminho[0];
    vector<string> comandos;
    for (const auto &passo : caminho)
    {
        printf("(%d, %d) ", passo.first, passo.second);
        if (passo != passo_anterior)
        {
            const auto direcao = direcaoEntre(passo_anterior, passo);
            if (direcao.empty())
            {
                RCLCPP_ERROR(node->get_logger(), "Passo inválido detectado durante reconstrução do caminho");
                return;
            }
            printf("%s", direcao.c_str());
            comandos.push_back(direcao);
            passo_anterior = passo;
        }
        
    }
    printf("\n");
    if (!executarMovimentos(node, comandos))
    {
        RCLCPP_WARN(node->get_logger(), "Nem todos os comandos foram executados com sucesso");
    }
    resetGame(node);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("solver_node");
    fullMapSolver(node);
    return 0;
}
