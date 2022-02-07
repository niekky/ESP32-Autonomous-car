'use strict';

import { initializeApp } from "https://www.gstatic.com/firebasejs/9.6.6/firebase-app.js";
import { getDatabase, ref, get, child, set } from "https://www.gstatic.com/firebasejs/9.6.6/firebase-database.js";
// TODO: Add SDKs for Firebase products that you want to use
// https://firebase.google.com/docs/web/setup#available-libraries

// Your web app's Firebase configuration
// For Firebase JS SDK v7.20.0 and later, measurementId is optional
const firebaseConfig = {
  apiKey: "AIzaSyA7-H8J9JW2x4IEn0t3l79JYnWNZD_Z9hM",
  authDomain: "nodemcu-a4907.firebaseapp.com",
  databaseURL: "https://nodemcu-a4907-default-rtdb.asia-southeast1.firebasedatabase.app",
  projectId: "nodemcu-a4907",
  storageBucket: "nodemcu-a4907.appspot.com",
  messagingSenderId: "1064805756238",
  appId: "1:1064805756238:web:6bd517bea7950607eec21f",
  measurementId: "G-F91FR6SJV3"
};

// Init values
var n=0;
var toggle=false;
const engine_status=document.querySelector(".engine_status");
const goToLogin=document.querySelector(".back");
const car_id=localStorage.getItem("storageName");
const graph = document.getElementById('graph');
console.log(car_id);

// Initialize Firebase
const app = initializeApp(firebaseConfig);
const db=getDatabase(app);
const dbRef=ref(db);

initializeGraph();

// WIP
function getPIDOutput(){
  return Math.random()-0.5;
}

function initializeGraph(){
  Plotly.newPlot( graph, [{
    y:[getPIDOutput()],
    type:"line" }], {
    margin: { t: 0 } } );
}

//Read previous inputs
get(child(dbRef,"IDs/"+car_id)).then((snapshot) => {
    if (snapshot.exists()){
        try{
            const values_obj=snapshot.val();
            document.querySelector(".P_value").value=values_obj["P"].toString();
            document.querySelector(".I_value").value=values_obj["I"].toString();
            document.querySelector(".D_value").value=values_obj["D"].toString();
            document.querySelector(".Motor_value").value=values_obj["Motor"].toString();
            document.querySelector(".Servo_value").value=values_obj["Servo"].toString();
        } catch(err) {
            document.querySelector(".P_value").value="0"
            document.querySelector(".I_value").value="0"
            document.querySelector(".D_value").value="0"
            document.querySelector(".Motor_value").value="0"
            document.querySelector(".Servo_value").value="0"
        }
    } else{
        console.log("Snapshot not available");
        document.querySelector(".P_value").value="0"
        document.querySelector(".I_value").value="0"
        document.querySelector(".D_value").value="0"
        document.querySelector(".Motor_value").value="0"
        document.querySelector(".Servo_value").value="0"
    }
  }).catch((error) => {
    console.error(error);
  })


// P,I,D,Motor,Servo inputs
document.querySelector(".enter").addEventListener("click", function(){
    var P_value=document.querySelector(".P_value").value;
    var I_value=document.querySelector(".I_value").value;
    var D_value=document.querySelector(".D_value").value;
    var motor_value=document.querySelector(".Motor_value").value;
    var servo_value=document.querySelector(".Servo_value").value;
    // send values
    if (P_value>=0 && I_value>=0 && D_value>=0 && motor_value>=0 && servo_value>=0){
        set(ref(db, "IDs/"+car_id), {
            P: P_value,
            I: I_value,
            D: D_value,
            Motor: motor_value,
            Servo: servo_value,
            Toggle: toggle
          });
    } else {
        console.log("Inputs are invalid!");
    }
})

// Toggle button
engine_status.addEventListener("click", function(){
    n=n+1;
    // OFF mode
    if (n%2==0){
        engine_status.innerText="OFF";
        toggle=false;

    // ON mode
    } else {
        engine_status.innerText="ON";
        toggle=true;
        // GRAPHING
        initializeGraph();
        var cnt=0;
        var realtime= setInterval(function(){
          Plotly.extendTraces(graph,{ y:[[getPIDOutput()]]},[0]);
          cnt++;
          if (cnt>250){
            Plotly.relayout(graph,{
              xaxis:{
                range:[cnt-250,cnt]
              } 
            })
          }
          if (toggle==false){
            clearInterval(realtime);
          }
        },20);
    }
    

    // Update toggle value
    get(child(dbRef,"IDs/"+car_id)).then((snapshot) => {
        if (snapshot.exists()){
          const values_obj=snapshot.val();
          values_obj["Toggle"]=toggle;
          set(ref(db, "IDs/"+car_id), {
            P: values_obj["P"],
            I: values_obj["I"],
            D: values_obj["D"],
            Motor: values_obj["Motor"],
            Servo: values_obj["Servo"],
            Toggle: values_obj["Toggle"]
          });
        
        } else{
          console.log("Snapshot not available");
        }
      }).catch((error) => {
        console.error(error);
      })
    })

goToLogin.addEventListener("click", function(){
    window.location.href = "index.html";
})

