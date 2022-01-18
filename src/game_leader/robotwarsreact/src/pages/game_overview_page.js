import React, { useEffect, useState } from 'react';
import Header from "../components/header"


export default function GameOverviewPage() {
    const [games, setGames] = useState([]);
    let [selectedGame, setSelectedGame] = useState(null);

    useEffect(() => {
      fetch('http://127.0.0.1:5000/api/v1/active_games')
      .then(response => response.json())
      .then(
        (data) => {
        setGames(data[0])
        setSelectedGame(data[0][0])
      },
      (error) => {
        window.alert(error)
      })
    }, [])
    
    function handleChange(event) {
        setSelectedGame(games.find(game => game.name === event.target.value))
    }

    return (
        <div style={styles.container}>
            <Header/>
            <div style={styles.content}>
                <div style={styles.overview}>
                    <select onChange={handleChange} style={styles.select}>
                        {games?.map((item)=> {
                            return <option value={item.name}>{item.name}</option>
                        })}
                    </select>
                    <text style={styles.field}>{"Name: " + selectedGame?.name}</text>
                    <text style={styles.field}>{"Description: " + selectedGame?.description}</text>
                    <text style={styles.field}>{"Gamemode: " + selectedGame?.gameMode}</text>
                    <text style={styles.field}>{"Robot 1 Team: " + selectedGame?.robot1Team}</text>
                    <text style={styles.field}>{"Robot 2 Team: " + selectedGame?.robot2Team}</text>
                    <text style={styles.field}>{"Robot 3 Team: " + selectedGame?.robot3Team}</text>
                    <text style={styles.field}>{"Maximum hp of robots: " + selectedGame?.maxHp}</text>
                    <text style={styles.field}>{"Game Time: " + selectedGame?.gameTime}</text>
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
  content: {
    height: "50%",
    margin: "20px",
    display: "flex",
    justifyContent: "space-evenly",
  },
  overview: {
    padding: "10px",
    width: "70%",
    height: "500px",
    backgroundColor: "#323232",
    borderRadius: "10px",
    display: "flex",
    flexDirection: "column",
    alignItems: "center"
  },
  select: {
    width: "200px",
    height: "20px",
    borderRadius: "10px"
  },
  field: {
    color: "white",
    marginTop: "5px",
    width: "100%",
    display: "flex"
  }
}