# Navegação: Lucas Periquito

## Instruções

1. Clone o repositório.
2. Rode ```colcon build``` na raiz do repositório.
3. Rode ```source install/setup.bash``` para configurar o ambiente.
4. Inicie o jogo com ```ros2 run cg maze```
5. Em outro terminal, rode ```ros2 run robert solver_node``` para rodar o sistema.


## Como funciona

### Desafio 1: Navegação otimizada com conhecimento do mapa

&emsp;Primeiramente, configuramos a main para iniciar o serviço ROS e criando um Node (ponto de entrada) chamado "solver_node", e passamos esse node para a função fullMapExplorer, que é responsável por realizar a primeira parte do desafio e chamar a segunda em seguida.

&emsp;Dentro do fullMapSolver, criamos uma matriz do mapa com todos os pontos do mapa, utilizando a chamada da interface GetMap. Em seguida, marcamos a posição inicial do robô e do objetivo, marcamos a primeira como visitada, e prosseguimos com a criação de mais três variáveis que serão de interesse para nosso algorítmo:
- Uma fila para armazenar os pontos a serem visitados (iniciada com a posição inicial do robô);
- Um vetor para armazenar se o ponto já foi visitado ou não (iniciado com todos os pontos como não visitados, exceto a posição inicial do robô);
- Vetor de parente, em que armazenamos a posição de onde o algorítmo veio antes de chegar nesse ponto. (-1,-1) significa que é o ponto inicial.

&emsp;No loop principal, marcamos o ponto atual como o primeiro da fila que criamos, aumentamos o contador e fazemos algumas verificações:

-Se o ponto atual for o objetivo, marcamos que encontramos o objetivo e saímos do loop.
-Caso contrário, verificamos os arredores, utilizando a função adicionarArredores, e verificamos se os pontos ao redor são caminhos novos válidos (não ocupados e não visitados). Se passarem nesse teste, adicionamos esses vizinhos na fila, marcamos como visitados e atualizamos o parent de cada um como o ponto atual.
&emsp;Quando passarmos por esses pontos na fila, repetimos o processo até encontrarmos o objetivo ou descobrirmos que não existe um caminho viável.
&emsp;Em seguida, se encontramos o objetivo, nós refazemos o caminho inteiro de trás pra frente, começando do objetivo até o ponto inicial com base no vetor de parentes. Por fim, invertemos o vetor para ter o caminho no sentido correto, criamos uma lista de movimentos para a comunicação com o robô, e chamamos a função executarMovimentos para enviar a lista de movimentos para o robô.
&emsp;ExecutarMovimentos cria um cliente que se utiliza do cg_interfaces/srv/Move para enviar os movimentos para o robô, e itera sobre a lista de movimentos, para completar o labirínto.

### Desafio 2: Navegação com exploração do mapa

&emsp;Vamos gravar esse caminho, por enquanto. Iremos usar mais tarde durante o mapping para verificar se é possível utilizar o caminho otimizado no mapa descoberto.
&emsp;Primeiro, recomeçamos o mapa com o mesmo layout, novamente utilizando os serviços cg já criados anteriormente no projeto. Depois, criamos um novo cliente ros para subscrever o tópico das posições, de forma a conseguirmos saber a posição atual e os arredores do robô a cada instante. Novamente chamamos a função de mapa completo para saber as posições iniciais do robô e do objetivo, e seguimos para o explorarMapa, passando o node ROS, as posições do robô e do objetivo, o formato do mapa (linhas x colunas) e o caminho otimizado da primeira parte.
&emsp;Novamente, setamos algumas variáveis para utilizarmos o DFS (Depth-First Search) para explorar o mapa:
- Uma pilha para armazenar os pontos a serem visitados (iniciada com a posição inicial do robô);
- Um vetor para armazenar se o ponto já foi visitado
- Um vetor do caminho total
- Um vetor condicional se um ponto tem mais de um caminho não explorável viável (divergência, porque não sei que palavra usar pelo amor)
- Vetor de parente, em que célula estavamos quando descobrimos esse ponto.
&emsp;Enquanto a pilha não estiver vazia, pegamos o ponto do topo da pilha como o ponto atual, e fazemos algumas verificações:
- Se o ponto já tiver sido visitado, continuamos para a próxima iteração do loop.

&emsp;Passado isso, temos nosso loop principal de navegação: Começamos verificando se o robô já está no seu target de movimento. Se não estiver, vamos movendo o robô seguindo os parentes até chegarmos no parent do alvo. Sainda nesse loop, fazemos uma verificação de segurança para garantir que o robô não está no target ao tentar se mover, se houver alguma interferência externa.
&emsp;Depois, marcamos o ponto atual como visitado, adicionamos ao caminho total, e verificamos se o alvo atual é o objetivo. Se for, falamos que alcançamos o objetivo e fazemos a verificação se o caminho otimizado pode ser utilizado no mapa descoberto.
&emsp;Os sensores são chamados no final do loop para mante-los atualizados após a movimentação, e as células adjacentes são adicionados a pilha.