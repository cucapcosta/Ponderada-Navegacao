// Ros pra cpp + comandos basicos do jogo
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/reset.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
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

bool verificarVizinhos(pair<int, int> pos_atual, const cg_interfaces::msg::RobotSensors &arredores, const vector<vector<bool>> &visitados, vector<pair<int, int>> &pilha)
{
    int livres = 0;
    int r = pos_atual.first;
    int c = pos_atual.second;
    int rows = static_cast<int>(visitados.size());
    int cols = static_cast<int>(visitados[0].size());

    // Right
    if (c < cols - 1 && arredores.right != "b" && !visitados[r][c + 1])
    {
        pilha.push_back({r, c + 1});
        livres++;
    }
    // Down
    if (r < rows - 1 && arredores.down != "b" && !visitados[r + 1][c])
    {
        pilha.push_back({r + 1, c});
        livres++;
    }
    // Up
    if (r > 0 && arredores.up != "b" && !visitados[r - 1][c])
    {
        pilha.push_back({r - 1, c});
        livres++;
    }
    // Left
    if (c > 0 && arredores.left != "b" && !visitados[r][c - 1])
    {
        pilha.push_back({r, c - 1});
        livres++;
    }

    if (livres == 0) {
        printf("Sem vizinhos livres em (%d, %d). Sensor: U=%s D=%s L=%s R=%s. Visitados: U=%d D=%d L=%d R=%d\n",
               r, c, 
               arredores.up.c_str(), arredores.down.c_str(), arredores.left.c_str(), arredores.right.c_str(),
               (r>0 ? (int)visitados[r-1][c] : -1),
               (r<rows-1 ? (int)visitados[r+1][c] : -1),
               (c>0 ? (int)visitados[r][c-1] : -1),
               (c<cols-1 ? (int)visitados[r][c+1] : -1)
               );
    }

    return livres > 1;
}

string direcaoEntre(const pair<int, int> &origem, const pair<int, int> &destino)
{
    if (destino.first == origem.first - 1 && destino.second == origem.second)
    {
        return "up";
    }
    if (destino.first == origem.first + 1 && destino.second == origem.second)
    {
        return "down";
    }
    if (destino.first == origem.first && destino.second == origem.second - 1)
    {
        return "left";
    }
    if (destino.first == origem.first && destino.second == origem.second + 1)
    {
        return "right";
    }
    return "";
}

pair<int, int> executarMovimentos(const rclcpp::Node::SharedPtr &node, const vector<string> &comandos)
{
    if (comandos.empty())
    {
        RCLCPP_INFO(node->get_logger(), "Nenhum movimento necessário: já estamos no objetivo");
        return {-2, -2}; // Código para "sem movimento" mas sucesso
    }

    auto client = node->create_client<cg_interfaces::srv::MoveCmd>("move_command");
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(node->get_logger(), "Servico move_command indisponivel");
        return {-1, -1};
    }

    pair<int, int> ultima_pos = {-1, -1};

    for (const auto &direcao : comandos)
    {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direcao;
        auto future = client->async_send_request(request);
        const auto status = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(2));
        if (status != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Falha ao enviar comando de movimento: %s", direcao.c_str());
            return {-1, -1};
        }
        auto response = future.get();
        if (!response)
        {
            RCLCPP_ERROR(node->get_logger(), "Resposta nula do move_command");
            return {-1, -1};
        }
        if (!response->success)
        {
            RCLCPP_WARN(node->get_logger(), "Movimento %s rejeitado pelo jogo", direcao.c_str());
            return {-1, -1};
        }
        const auto robo_x = static_cast<int>(response->robot_pos[0]);
        const auto robo_y = static_cast<int>(response->robot_pos[1]);
        ultima_pos = {robo_x, robo_y};
        RCLCPP_INFO(node->get_logger(), "Movido %s para (%d, %d)", direcao.c_str(), robo_x, robo_y);
        
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return ultima_pos;
}

void resetGame(const rclcpp::Node::SharedPtr &node)
{
    auto client = node->create_client<cg_interfaces::srv::Reset>("/reset");
    if (!client->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(node->get_logger(), "Servico /reset indisponivel");
        return;
    }

    auto request = std::make_shared<cg_interfaces::srv::Reset::Request>();
    request->is_random = false;
    request->map_name.clear();
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Falha ao chamar o servico /reset");
        return;
    }

    auto response = future.get();
    if (!response)
    {
        RCLCPP_ERROR(node->get_logger(), "Resposta nula do /reset");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Jogo reiniciado com sucesso");
    std::this_thread::sleep_for(std::chrono::seconds(2));
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

cg_interfaces::msg::RobotSensors lerSensorAtual(const rclcpp::Node::SharedPtr &node)
{
    cg_interfaces::msg::RobotSensors::SharedPtr msg;
    auto sub = node->create_subscription<cg_interfaces::msg::RobotSensors>(
        "/culling_games/robot_sensors", rclcpp::SensorDataQoS(),
        [&msg](const cg_interfaces::msg::RobotSensors::SharedPtr m) { msg = m; });
    
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !msg)
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(2)) {
             RCLCPP_WARN(node->get_logger(), "Timeout esperando sensor");
             return cg_interfaces::msg::RobotSensors();
        }
    }
    return *msg;
}

pair<int, int> moverRobo(const rclcpp::Node::SharedPtr &node, pair<int, int> de, pair<int, int> para)
{
    string dir = direcaoEntre(de, para);
    if (dir.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Tentativa de movimento invalido (nao adjacente): (%d, %d) -> (%d, %d)", 
                     de.first, de.second, para.first, para.second);
        return {-1, -1};
    }
    return executarMovimentos(node, {dir});
}

bool explorarMapa(const rclcpp::Node::SharedPtr &node, pair<int,int> posicao_inicial, pair<int,int> posicao_objetivo, int rows, int cols, vector<pair<int, int>> caminho_optimo = {})
{
    //Setup das coisinhas
    vector<pair<int, int>> caminho_total;
    pair<int, int> pos_robo = posicao_inicial;
    
    //Variáveis para fazer o DFS
    vector<pair<int, int>> pilha;
    pilha.push_back(pos_robo);
    
    vector<vector<bool>> visitados(rows, vector<bool>(cols, false));
    
    vector<vector<bool>> temDivergencia(rows, vector<bool>(cols, false));
    vector<vector<pair<int, int>>> parent(rows, vector<pair<int, int>>(cols, {-1, -1}));
    
    while(!pilha.empty())
    {
        auto target = pilha.back();
        pilha.pop_back();

        // Se já visitamos este nó (por outro caminho), ignoramos
        if (visitados[target.first][target.second])
        {
            continue;
        }

        printf("Processando alvo (%d, %d). Posicao atual: (%d, %d)\n", 
               target.first, target.second, pos_robo.first, pos_robo.second);

        // Lógica de Navegação 
        if (target != pos_robo) 
        {
             auto p_target = parent[target.first][target.second];
             
             // Backtrack até o pai do target
             while (pos_robo != p_target)
             {
                 if (p_target.first == -1 && pos_robo == posicao_inicial) break;
                 
                 auto p = parent[pos_robo.first][pos_robo.second];
                 if (p.first == -1) break; //Ainda no inicio
                 
                 printf("Backtracking: (%d, %d) -> (%d, %d)\n", pos_robo.first, pos_robo.second, p.first, p.second);
                 auto nova_pos = moverRobo(node, pos_robo, p);
                 if (nova_pos.first == -1) 
                 {
                     RCLCPP_ERROR(node->get_logger(), "Falha crítica no backtracking de (%d,%d) para (%d,%d)", 
                                  pos_robo.first, pos_robo.second, p.first, p.second);
                     printf("Falha crítica no backtracking de (%d,%d) para (%d,%d)\n", 
                            pos_robo.first, pos_robo.second, p.first, p.second);
                     return false;
                 }
                 pos_robo = nova_pos;
             }
             
             // Mover do pai para o target
             if (pos_robo != target) {
                 printf("Avancando: (%d, %d) -> (%d, %d)\n", pos_robo.first, pos_robo.second, target.first, target.second);
                 auto nova_pos = moverRobo(node, pos_robo, target);
                 if (nova_pos.first == -1) 
                 {
                     RCLCPP_WARN(node->get_logger(), "Falha ao mover para target (%d,%d), assumindo bloqueado", 
                                 target.first, target.second);
                     printf("Falha ao mover para target (%d,%d), assumindo bloqueado\n", 
                            target.first, target.second);
                     // Se falhar, marcamos como visitado (para não tentar de novo) e continuamos
                     visitados[target.first][target.second] = true;
                     continue;
                 }
                 pos_robo = nova_pos;
             }
        }

        // Processar o nó
        visitados[target.first][target.second] = true;
        caminho_total.push_back(target);

        if (target == posicao_objetivo)
        {
            printf("Objetivo (%d, %d) alcancado! Verificando completude do caminho...\n", target.first, target.second);
            bool contem_todos = true;
            for (const auto &ponto_optimo : caminho_optimo)
            {
                bool encontrado = false;
                for (const auto &ponto_total : caminho_total)
                {
                    if (ponto_optimo == ponto_total)
                    {
                        encontrado = true;
                        break;
                    }
                }
                if (!encontrado)
                {
                    contem_todos = false;
                    printf("Ponto do caminho otimo (%d, %d) NAO foi visitado.\n", ponto_optimo.first, ponto_optimo.second);
                }
            }
            
            if (contem_todos) {
                printf("SUCESSO: Todos os pontos do caminho otimo foram visitados!\n");
                RCLCPP_INFO(node->get_logger(), "SUCESSO: Todos os pontos do caminho otimo foram visitados!");
                return true;
            } else {
                printf("FALHA: Nem todos os pontos do caminho otimo foram visitados.\n");
                RCLCPP_WARN(node->get_logger(), "FALHA: Nem todos os pontos do caminho otimo foram visitados.");
                return false;
            }
        }

        // Ler sensores reais na posição atual
        auto arredores = lerSensorAtual(node);
        if (arredores.up.empty()) {
             printf("Aviso: Leitura de sensor vazia/timeout em (%d, %d)\n", pos_robo.first, pos_robo.second);
        }

        // Expandir vizinhos
        size_t old_size = pilha.size();
        if (verificarVizinhos(target, arredores, visitados, pilha))
        {
            temDivergencia[target.first][target.second] = true;
        }
        
        // Debug pilha
        printf("Vizinhos adicionados: %zu. Tamanho da pilha: %zu\n", pilha.size() - old_size, pilha.size());

        // Registrar paternidade para os novos nós adicionados à pilha
        for (size_t i = old_size; i < pilha.size(); ++i)
        {
            auto child = pilha[i];
            parent[child.first][child.second] = target;
        }
    }
    printf("Exploracao DFS finalizada.\n");
    return contem_todos;
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
    auto pos_robo = mapa_completo.posicao_inicial;
    auto pos_objetivo = mapa_completo.posicao_objetivo;

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
    if (executarMovimentos(node, comandos).first == -1)
    {
        RCLCPP_WARN(node->get_logger(), "Nem todos os comandos foram executados com sucesso");
    }
    resetGame(node);
    static rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub;
    sensor_sub = node->create_subscription<cg_interfaces::msg::RobotSensors>(
        "/culling_games/robot_sensors", rclcpp::SensorDataQoS(),
        [node](const cg_interfaces::msg::RobotSensors::SharedPtr msg)
        {
            if (!msg)
            {
                return;
            }
            RCLCPP_DEBUG(node->get_logger(), "Sensor up:%s down:%s left:%s right:%s",
                         msg->up.c_str(), msg->down.c_str(), msg->left.c_str(), msg->right.c_str());
        });
    // Após resetar, refazemos o mapeamento explorando via DFS (FILO)
    mapa_completo = getMap(node);
    if (!mapa_completo.valido)
    {
        RCLCPP_ERROR(node->get_logger(), "Falha ao obter mapa completo");
        return;
    }
    pos_robo = mapa_completo.posicao_inicial;
    pos_objetivo = mapa_completo.posicao_objetivo;


    explorarMapa(node, pos_robo, pos_objetivo, linhas, colunas, caminho);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("solver_node");
    fullMapSolver(node);
    return 0;
}
