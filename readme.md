# Navegação: Lucas Periquito

## Instruções

1. Crie uma pasta dentro de `src`, do culling_games, com o nome que desejar
2. Copie os conteúdos deste repositório para a pasta criada
3. Execute o comando ``` colcon build --packages-select [nome_da_pasta] ``` para compilar apenas o pacote novo
4. Em um terminal na raiz de "culling_games", execute o comando ``` source install/setup.bash ``` e, em seguida, ```ros2 run cg maze``` para iniciar a simulação do labirinto
5. Em outro terminal, execute o comando ``` source install/setup.bash ``` e, em seguida, ```ros2 run [nome_da_pasta] solver_node``` para iniciar o código

&emsp; Neste sistema, o robô primeiramente procurará o objetivo com informalções conhecidas do mapa, fazendo a busca por amplitude (BFS). Após encontrar o objetivo, o caminho dos pontos que o robô passou fica salvo e, em seguida, o robô reinicia o trajeto, agora fazendo uma busca por profundidade para chegar novamente no objetivo, gerando o mapeamento do labirinto. Esse último tem seu resultado comparado com o primeiro para ver se, dado as informações de navegação da segunda run, seria possível gerar o mesmo caminho usando BFS.

## Notas

&emsp; Para total transparência, o código teve partes geradas por IA, principalmente com relação à comunicação ROS e aplicação dos conceitos. Creio que consigo entender e explicar o que está acontecendo com o sistema, mas, devido a limitações de tempo, foi necessário recorrer a essa ferramenta para acelerar o desenvolvimento.