import React from 'react';

export default function SelectRobotBox() {
    return(
    <select style={styles.select}>
        <option style={styles.selectOption}>Robot 1</option>
        <option style={styles.selectOption}>Robot 2</option>
        <option style={styles.selectOption}>Robot 3</option>
        <option style={styles.selectOption}>Robot 4</option>
    </select>
    )
}

const styles = {
    select: {
        width: "200px",
        height: "20px",
        borderRadius: "10px"
      },
      selectOption: {
        
      },
}