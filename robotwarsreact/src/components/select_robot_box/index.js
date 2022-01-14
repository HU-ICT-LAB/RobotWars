import React from 'react';

export default function SelectRobotBox() {
    function handleChange(event){
        console.log(event.target.value)
    }
    
    return(
    <select style={styles.select} onChange={handleChange}>
        <option style={styles.selectOption}>Robot 1</option>
        <option style={styles.selectOption}>Robot 2</option>
        <option style={styles.selectOption}>Robot 3</option>
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