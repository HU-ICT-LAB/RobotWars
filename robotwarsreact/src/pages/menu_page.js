import React from 'react';
import { MenuButton } from '../components/menu_button/menu_button';

export const MenuPage = ()=> {
  return (
    <div>
      <h1 style={styles.title} >RobotWars Menu</h1>
      <div style={styles.buttons}>
        <MenuButton text={"Get robots"} adress={"robots"}/>
        <MenuButton text={"Start robots"} adress={"start"}/>
      </div>
    </div>
  );
}

const styles = {
  title: {
    color: "white",
    paddingBottom: "20px",
  },

  buttons: {
    paddingBottom: "20px",
  },
}