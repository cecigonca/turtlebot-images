# Ponderada Turtlebot Teleoperado - Parte 2
Incrementar o sistema do turtlebot teleoperado para incluir conceitos de streaming de imagens.

## Vídeo da Atividade
Para visualizar na prática, assista esse [vídeo](https://drive.google.com/file/d/1Q-YPd4ftMF2uKfFxZBMDGQciblsHT9WI/view?usp=drive_link)

## Instruções para Execução
### Clone o Repositório
Coloque esse comando no terminal

```git clone ```

### Ative o Ambiente Virtual (terminal Linux)
Navegue até o diretorio

```/../turtlebot-images/```

Rode os comandos seguintes

```python3 -m venv venv```

```source ./venv/bin/activate```

### Instalar GStreamer
```sudo apt-get install libgstreamer1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly```

### Instalar Numpy
```pip3 install numpy```

### Instalar OpenCV
```pip3 install opencv-python-headless```

### Importando as Dependências
```python3 -m pip install -r requirements.txt```

### Comandos para rodar a atividade 
Abra quatro terminais diferentes e rode um comando em cada um 
1. ```ros2 launch webots_ros2_turtlebot robot_launch.py``` : Inicia o Wedots, onde pode-se ver a movimentação do robô;
2. ```ros2 launch rosbridge_server rosbridge_websocket_launch.xml``` : Com esse comando você irá conseguir rodar o programa para poder obter a visualização da câmera, seja do robô ou seja da webcam do seu notebook.
Em paralelo a esse cmd, temos que rodar outro para que aconteça as publicações nos tópicos através do WebSockets;
3. ```python3 backend/mover.py``` : Roda o script responsável pela movimentação do robô;
4. ```python3 backend/sender.py``` : Roda o script responsável pelo funcionamento da câmera.

## Explicação dos Botôes do Frontend
- Botão "Frente": Movimenta o robô para frente;
- Botão "Trás": Movimenta o robô para trás;
- Botão "Esquerda": Vira o robô para esquerda;
- Botão "Direita": Vira o robô para direita;
- Botão "Matar nó de movimentação": Mata o nó de movimentação;


