![banner](https://user-images.githubusercontent.com/60598779/138715577-8c875e6d-62c0-491b-90b8-e4181d99bf8c.png)

## Description
This project is commissioned by Hogeschool Utrecht. This project is originated with the purpose to excite potential new students about the HBO-ICT study. The main goal is to automatically make RoboMaster s1's lasergame with eachother.

## Installation
The project consists of multiple processes running on multiple machines, when installing you need to specify which machine you want to install.  
For deployment, you need one "GameLeader" and multiple "robots",  
so on the machine you want to assign as the GameLeader, you run this to install:
```bash
pip install .[game_leader]
```
And on each of the machines that control the robots, you run:
```bash
pip install .[robot]
```

For training, you'll also need an "AI-trainer", installed with:
```bash
pip install .[ai_trainer]
```
To run the experiments install `[experiments]`, if you want everything, use: `[all]` and developers should use `[dev]`.

## Usage
If you have the api and webapp running you can simply press one of the buttons in the webapp to run a command.
![react webapp](https://user-images.githubusercontent.com/60598779/138708913-d1ec42e1-289e-45fa-bb00-7375f80d0656.png)

Start the api: 
```bash
cd RobotWarsApi && python3 api.py
```

Start the react webapp
```bash
cd robotwarsreact && yarn install && yarn start
```

## Contributors
**Artificial Intelligence**
- Richard  [github](https://github.com/RichardDev01)
- Max      [github](https://github.com/Max2411)
- Lucas    [github](https://github.com/Lucas-vdr-Horst)
- Gerrit   [github](https://github.com/SwagLag)

**Technical Informatics**
- Matthies [github](https://github.com/MatthiesBrouwer)

**Software Developer**
- Ruben    [github](https://github.com/Rubenvdbrink)

**Business informatics**
- Bas      [github](https://github.com/The-Bimmer)
