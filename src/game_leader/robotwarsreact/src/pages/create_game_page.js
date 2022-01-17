import React from 'react';
import Header from "../components/header"

export default function CreateGamePage() {
  var robot1 = React.createRef();
  var robot2 = React.createRef();
  var robot3 = React.createRef();

  var state = {gameName: '',
              gameMode: 'Free for all',
              robot1Team: '',
              robot2Team: '',
              robot3Team: '',
              gameDescription: '',
              gameHP: '',
              gameTime: ''}

  function handleSubmit() {
    fetch("http://127.0.0.1:5000/api/v1/new_game", {
    method: "POST",
    headers: {
      'Content-Type' : 'application/json',
      },
    body: JSON.stringify({
      "name": state.gameName,
      "description": state.gameDescription,
      "gameHP": state.gameHP,
      "gameMode": state.gameMode,
      "robot1Team": state.robot1Team,
      "robot2Team": state.robot2Team,
      "robot3Team": state.robot3Team,
    })
  }).then(response => response.json())
  .then(data => console.log(data))
  }

  function handleChange(event) {
    var value = event.target.value;
    var target = event.target.name;
    switch (target) {
      case 'gameName':
        state.gameName = value;
        break;
      case 'gameMode':
        state.gameMode = value;
        if(value === "Free for all") {
          state.robot1Team = ''
          state.robot2Team = ''
          state.robot3Team = ''
          robot1.current.disabled = true
          robot2.current.disabled = true
          robot3.current.disabled = true
        }
        if(value === "Team Deathmatch") {
          state.robot1Team = 'Team 1'
          state.robot2Team = 'Team 1'
          state.robot3Team = 'Team 1'
          robot1.current.disabled = false
          robot2.current.disabled = false
          robot3.current.disabled = false
        }
        break;
      case 'robot1Team':
        state.robot1Team = value
        break;
      case 'robot2Team':
        state.robot2Team = value
        break;
      case 'robot3Team':
        state.robot3Team = value
        break;
      case 'gameDescription':
        state.gameDescription = value;
        break;
      case 'gameHP':
        state.gameHP = value;
        break;
      case 'gameTime':
        state.gameTime = value;
        break;
      default:
        break;
    }
    console.log(state)
  }

  return (
    <div style={styles.container}>
      <div style={styles.header}>
        <Header/>
      </div>
      <div style={styles.content}>
        <div style={styles.form}>
          <label style={styles.label}>
            Name of the game:
            <input type="text" name="gameName" onChange={handleChange} />
          </label>
          <label style={styles.label}>
            Gamemode:
            <select style={styles.selectBox} name="gameMode" onChange={handleChange}>
              <option>Free for all</option>
              <option>Team Deathmatch</option>
            </select>
          </label>
          <label style={styles.label}>
            Robot 1 Team:
            <select ref={robot1} disabled={true} style={styles.selectBox} name="robot1Team" onChange={handleChange}>
              <option>Team 1</option>
              <option>Team 2</option>
            </select>
          </label>
          <label style={styles.label}>
            Robot 2 Team:
            <select ref={robot2} disabled={true} style={styles.selectBox} name="robot2Team" onChange={handleChange}>
              <option>Team 1</option>
              <option>Team 2</option>
            </select>
          </label>
          <label style={styles.label}>
            Robot 3 Team:
            <select ref={robot3} disabled={true} style={styles.selectBox} name="robot3Team" onChange={handleChange}>
              <option>Team 1</option>
              <option>Team 2</option>
            </select>
          </label>
          <label style={styles.label}>
            Description:
            <input type="text" name="gameDescription" onChange={handleChange} />
          </label>
          <label style={styles.label}>
            Amount of HP per robot:
            <input type="number" name="gameHP" onChange={handleChange} />
          </label>
          <label style={styles.label}>
            Amount of time in seconds:
            <input type="number" name="gameTime" onChange={handleChange} />
          </label>
          <button style={styles.submit} onClick={handleSubmit}>Submit</button>
        </div>
      </div>
    </div>
  );
}

const styles = {
  container: {
    width: "100%",
    height: "100%",
  },
  form: {
    width: "70%",
    height: "500px",
    backgroundColor: "#323232",
    borderRadius: "10px",
    display: "flex",
    flexDirection: "column",
    color: "white",
    alignItems: "center",
    padding: "10px"
  },
  label: {
    marginTop: "10px",
    display: "flex",
    justifyContent: "space-between",
    minWidth: "400px"
  },
  selectBox: {
    width: "170px"
  },
  submit: {
    width: "150px",
    height: "30px",
    backgroundColor: "#0D7377",
    borderRadius: "10px",
    borderStyle: "hidden",
    color: "white",
    alignSelf: "center",
    marginTop: "auto",
  },
  content: {
    height: "50%",
    margin: "20px",
    display: "flex",
    justifyContent: "space-evenly",
  },
}