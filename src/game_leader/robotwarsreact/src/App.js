import './App.css';
import MenuPage from "./pages/menu_page"
import CreateGamePage from "./pages/create_game_page"
import GameOverviewPage from "./pages/game_overview_page"
import { Routes, Route } from "react-router-dom"

export default function App() {
  return (
    <div className="App">
      <Routes>
        <Route path="/" element={<MenuPage/>}></Route>
        <Route path="create-game" element={<CreateGamePage/>}></Route>
        <Route path="game-overview" element={<GameOverviewPage/>}></Route>
      </Routes>
    </div>
  );
}
