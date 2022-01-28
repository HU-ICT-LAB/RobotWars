import React, {useState, useEffect} from "react";
import SelectBox from "./SelectBox";
import TextLine from "./TextLine";

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
      <TextLine type={"Name"} value={selectedGame === null ? "" : selectedGame.name} />
      <TextLine type={"Description"} value={selectedGame === null ? "" : selectedGame.description} />
      <TextLine type={"Gamemode"} value={selectedGame === null ? "" : selectedGame.gameMode} />
      <TextLine type={"Robot 1 Team"} value={selectedGame === null ? "" : selectedGame.robot1Team} />
      <TextLine type={"Robot 2 Team"} value={selectedGame === null ? "" : selectedGame.robot2Team} />
      <TextLine type={"Robot 3 Team"} value={selectedGame === null ? "" : selectedGame.robot3Team} />
      <TextLine type={"Maximum hp of robots"} value={selectedGame === null ? "" : selectedGame.maxHp} />
      <TextLine type={"Game Time"} value={selectedGame === null ? "" : selectedGame.gameTime} />
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
    alignItems: "flex-start",
  },
  title: {
    color: "white",
  },
  field: {
    color: "white",
    marginTop: "10px",
    width: "100%",
    display: "flex",
  },
}