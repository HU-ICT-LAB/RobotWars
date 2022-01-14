import React from 'react';
import Header from "../components/header"


export default function GameOverviewPage() {
    var games = getGames()

    function getGames() {
        return ["Game 1","Game 2","Game 3"]
    }
    
    function handleChange(event) {
        console.log("make request for: " + event.target.value)
    }

    return (
        <div style={styles.container}>
            <Header/>
            <div style={styles.content}>
                <div style={styles.overview}>
                    <select onChange={handleChange} style={styles.select}>
                        {games.map((item, index)=> {
                            return <option>{item}</option>
                        })}
                    </select>
                    <text style={styles.field}>Name:</text>
                    <text style={styles.field}>Description:</text>
                    <text style={styles.field}>Max HP of the robots:</text>
                    <text style={styles.field}>Time:</text>
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
    width: "200px",
    display: "flex"
  }
}