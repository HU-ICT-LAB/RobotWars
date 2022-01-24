import React, {useState, useEffect} from "react";
import SelectBox from "./SelectBox";

export default function GameOverview() {
  const [games, setGames] = useState([]);
  let [selectedGame, setSelectedGame] = useState(null);

  useEffect(() => {
    fetch("http://127.0.0.1:5000/api/v1/active_games")
      .then((response) => response.json())
      .then(
        (data) => {
          setGames(data[0]);
          setSelectedGame(data[0][0]);
        },
        (error) => {
          window.alert(error);
        }
      );
  }, []);

  function handleChange(event) {
    setSelectedGame(games.find((game) => game.name === event.target.value));
  }
  return (
    <div style={styles.overview}>
    <h2 style={styles.title}>Game information</h2>
    <SelectBox items={games} onChange={handleChange}></SelectBox>
    <text style={styles.field}>
      {"Name: " + (selectedGame?.name === null ? "" : selectedGame?.name)}
    </text>
    <text style={styles.field}>
      {"Description: " +
        (selectedGame?.description === null
          ? ""
          : selectedGame?.description)}
    </text>
    <text style={styles.field}>
      {"Gamemode: " +
        (selectedGame?.gameMode === null ? "" : selectedGame?.gameMode)}
    </text>
    <text style={styles.field}>
      {"Robot 1 Team: " +
        (selectedGame?.robot1Team === null
          ? ""
          : selectedGame?.robot1Team)}
    </text>
    <text style={styles.field}>
      {"Robot 2 Team: " +
        (selectedGame?.robot2Team === null
          ? ""
          : selectedGame?.robot2Team)}
    </text>
    <text style={styles.field}>
      {"Robot 3 Team: " +
        (selectedGame?.robot3Team === null
          ? ""
          : selectedGame?.robot3Team)}
    </text>
    <text style={styles.field}>
      {"Maximum hp of robots: " +
        (selectedGame?.maxHp === null ? "" : selectedGame?.maxHp)}
    </text>
    <text style={styles.field}>
      {"Game Time: " +
        (selectedGame?.gameTime === null ? "" : selectedGame?.gameTime)}
    </text>
  </div>
  );
}

const styles = {
  overview: {
    paddingLeft: "30px",
    backgroundColor: "#323232",
    height: "500px",
    maxWidth: "40%",
    minWidth: "500px",
    display: "flex",
    flexDirection: "column",
    borderRadius: "10px",
  },
  field: {
    color: "white",
    marginTop: "10px",
    width: "100%",
    display: "flex",
  },
}