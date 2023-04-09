#!/bin/bash



# inicia o simulador Gazebo e cria uma entidade de lata de refrigerante com o nome "coke1" na posição x=0.45, y=0.32 e z=0.55.

ros2 run gazebo_ros spawn_entity.py -database 'coke_can' -entity coke1 -x 0.45 -y 0.32 -z 0.55



# inicia o simulador Gazebo e cria uma entidade de lata de refrigerante com o nome "coke2" na posição x=-0.10, y=0.65 e z=0.55.

#ros2 run gazebo_ros spawn_entity.py -database 'coke_can' -entity coke2 -x -0.10 -y 0.65 -z 0.55



# inicia o simulador Gazebo e cria uma entidade de lata de refrigerante com o nome "coke3" na posição x=0.40, y=0.10 e z=0.55.

#ros2 run gazebo_ros spawn_entity.py -database 'coke_can' -entity coke3 -x 0.40 -y 0.1 -z 0.55



# comando publica mensagens no tópico /coke_position com a posição de cada lata, a orientação da lata é fixada como 45 graus em relação ao eixo x. O parâmetro -1 indica que a mensagem deve ser publicada apenas uma vez e o nó deve ser encerrado após a publicação da mensagem.

ros2 topic pub /coke_position geometry_msgs/PoseStamped "header:

  stamp: {sec: 0, nanosec: 0}

  frame_id: 'world'

pose:

  position: {x: 0.45, y: 0.32, z: 0.55}

  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1}" -1
