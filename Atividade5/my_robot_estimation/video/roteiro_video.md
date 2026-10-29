# Roteiro de narração — Prática 05.1

Texto de cada cena, com o instante em que ela começa no vídeo `pratica_05_1.mp4` (duração total 198 s).

## [00:00] titulo

Nesta prática eu uso o filtro de Kalman do pacote robot_localization para estimar a pose do robô, e depois refaço o seguimento da trajetória em 8, agora controlando com o estado estimado e com os sensores bem mais ruidosos.

## [00:06] arq

Exercício 1.1. O Gazebo fornece a odometria das rodas, o giroscópio da IMU e o ground truth. Um nó acrescenta ruído gaussiano controlado por um parâmetro, o noise_scale. O ekf_filter_node recebe as velocidades da odometria e do giroscópio e uma medida de posição absoluta, e publica a pose estimada, x, y e theta, em /odometry/filtered.

## [00:30] cfg

No ekf.yaml o filtro roda em modo 2D a 50 hertz. A odometria entra só com as velocidades, porque a pose dela é uma integral ruidosa e deriva. O giroscópio reforça a velocidade angular e o sensor de posição absoluta ancora x, y e yaw. A matriz Q diz quanto confiar no modelo, e cada sensor traz a sua covariância R.

## [00:50] term

Aqui está a saída real dos comandos. O ekf_filter_node está no ar junto com os sensores ruidosos, o tópico /odometry/filtered publica a cerca de 50 hertz, e a mensagem traz posição, orientação e velocidades estimadas.

## [01:10] anim1

Esta é a simulação com ruído padrão. Em verde está o estado real; em laranja, o sensor de posição ruidoso; em vermelho, a odometria bruta, que deriva para longe; e em azul, a estimativa do filtro, que fica colada no real. O RMSE do Kalman é menor que o do melhor sensor sozinho.

## [01:41] ctrl

Exercício 2.1. Refiz o experimento do 8 trocando apenas o tópico que o controlador lê: em vez da odometria, ele usa /odometry/filtered. Para o robô realmente seguir a trajetória foi preciso calibrar a separação das rodas e usar ômega igual a 0.25, porque com ômega 0.5 o robô não segue o 8 nem com o estado perfeito.

## [02:05] anim2

Agora aumentei o ruído cinco vezes: o sensor de posição passa a ter desvio de 25 centímetros e a nuvem laranja espalha tudo. O filtro ainda reduz o erro de 0.36 para 0.12 metros, e o robô, controlado pela estimativa, continua desenhando o 8, com erro real de 11 centímetros.

## [02:36] cmp

Comparando com o controle usando a odometria bruta: o controle acha que está a poucos centímetros da trajetória, mas o robô real fica a 32 centímetros com ruído padrão e 58 centímetros com ruído cinco vezes maior. Com o Kalman, o erro real fica em 7 e 11 centímetros.

## [03:02] fim

Resumindo: o filtro de Kalman fundiu os sensores, publicou a pose em /odometry/filtered, e controlar com o estado estimado mantém o robô na trajetória mesmo com muito ruído. O controle é tão bom quanto o estado que ele recebe.

