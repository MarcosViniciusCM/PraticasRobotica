# Narração dos clipes — Prática 05.2

Texto de cada cena (mesmo da legenda do vídeo), para ler em voz alta.

## 00_abertura

Prática 05.2, SLAM. Montei o pacote my_robot_slam: o robô da prática anterior com um Lidar 2D, uma sala simples no Gazebo, e o slam_toolbox. Vou mostrar o ruído no Lidar, a configuração do SLAM, a comparação com e sem ruído, e o robô sequestrado.

## 01_ex1_1_ruido_lidar

O ruído é do próprio sensor do Gazebo: cada distância medida ganha um valor gaussiano de média zero. O desvio padrão é o argumento lidar_noise_std; zero é sensor perfeito, 0.03 são 3 centímetros. O Lidar tem 360 feixes, 10 hertz e alcance de 8 metros, e fica a 15 centímetros de altura, acima do topo das rodas.

Mesma volta, mesmo mapa, só muda o ruído. Sem ruído, a parede vira uma linha fina de células ocupadas. Com 3 centímetros de desvio, a mesma parede fica espalhada: de 780 para 2030 células ocupadas.

## 02_ex2_1_slam_toolbox

Configurei o slam_toolbox em modo online assíncrono. Ele precisa do tópico do scan, do frame do robô e do TF de odom para base_link. O resto são os parâmetros de scan matching e de fechamento de laço, com o padrão do pacote; só reduzi a distância mínima entre scans de 0.5 para 0.15 metro.

A árvore de TF: o slam_toolbox publica map para odom; o meu nó odom_tf_node publica odom para base_link, com a pose real do Gazebo ou com a odometria ruidosa; o robot_state_publisher publica o Lidar.

Resultado com tudo ground truth: o robô percorre a sala e o SLAM constrói o mapa. A trajetória estimada, em azul, cobre a real, em verde: erro de posição de 1,3 centímetro.

## 03_ex2_2_comparacao

Três situações, mesmo percurso de duas voltas: tudo ground truth; ruído no Lidar com odometria perfeita; e ruído no Lidar e na odometria. O script run_slam_experiment salva mapa, CSV e métricas contra o ground truth.

Ruído no Lidar e na odometria. A linha vermelha é só a odometria: ela deriva até 4,8 metros, o caminho nem lembra a sala. O SLAM, em azul, fica em 6 centímetros, com picos de até 28. Quem remove a deriva é o scan matching contra o mapa e o fechamento de laço na segunda volta.

Os três mapas lado a lado: com ruído no Lidar as paredes engrossam, mas o mapa continua fechado, mesmo com a odometria ruidosa.

Resumindo: o ruído no Lidar piora pouco, de 1 para 4 centímetros; a odometria ruidosa sozinha seria desastrosa, 2,2 metros de RMSE, mas com o SLAM o erro fica em 6 centímetros.

## 04_ex3_1_robo_sequestrado

Com o mapa já construído, o slam_toolbox em modo localização, com ruído no Lidar e na odometria. Os passos: pausar o Gazebo, mover o robô em modo de translação para o outro lado do mapa, e retomar a simulação.

O robô de verdade, em verde, foi levado de lá para cá. A estimativa do SLAM, em azul, continua na origem. O erro vai de 1 centímetro para 3,6 metros e não corrige: um minuto depois são 4,1 metros.

Dando uma pose nova pelo 2D Pose Estimate do RViz, que publica em initialpose, a estimativa pula para o lugar certo: o erro cai para 4 centímetros.

A localização não se recupera sozinha. O slam_toolbox só compara o scan com o mapa perto da pose que ele já acredita estar, e a odometria não percebeu o salto. Alargar a busca ou deixar o robô andando não resolveu. É preciso uma pose inicial nova, ou um localizador global como o AMCL.

