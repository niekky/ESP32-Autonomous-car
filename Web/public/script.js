'use strict';

import { initializeApp } from "https://www.gstatic.com/firebasejs/9.6.6/firebase-app.js";
import { getDatabase, ref, get, child, set } from "https://www.gstatic.com/firebasejs/9.6.6/firebase-database.js";
// TODO: Add SDKs for Firebase products that you want to use
// https://firebase.google.com/docs/web/setup#available-libraries

// Your web app's Firebase configuration
// For Firebase JS SDK v7.20.0 and later, measurementId is optional
const firebaseConfig = {
  apiKey: "AIzaSyA7-H8J9JW2x4IEn0t3l79JYnWNZD_Z0xx",
  authDomain: "nodemcu-aXXXX.firebaseapp.com",
  databaseURL: "https://nodemcu-aXXXX-default-rtdb.asia-southeastX.firebasedatabase.app",
  projectId: "nodemcu-aXXXX",
  storageBucket: "nodemcu-aXXXX.appspot.com",
  messagingSenderId: "1064805756238",
  appId: "1:1064805756XXX:web:6bd517bea7950607eecxxf",
  measurementId: "G-F91FR6SXXX"
};

// Init values
var n=0;
var toggle=false;
const engine_status=document.querySelector(".engine_status");
const goToLogin=document.querySelector(".back");
const goToTraffic=document.querySelector(".traffic")
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
            document.querySelector(".P_servo_value").value=values_obj["P_servo"].toString();
            document.querySelector(".I_servo_value").value=values_obj["I_servo"].toString();
            document.querySelector(".D_servo_value").value=values_obj["D_servo"].toString();
            document.querySelector(".P_motor_value").value=values_obj["P_motor"].toString();
            document.querySelector(".I_motor_value").value=values_obj["I_motor"].toString();
            document.querySelector(".D_motor_value").value=values_obj["D_motor"].toString();
            document.querySelector(".Motor_value").value=values_obj["Motor"]
            document.querySelector(".Servo_value").value=values_obj["Servo"]
        } catch(err) {
            document.querySelector(".P_servo_value").value="0";
            document.querySelector(".I_servo_value").value="0";
            document.querySelector(".D_servo_value").value="0";
            document.querySelector(".P_motor_value").value="0";
            document.querySelector(".I_motor_value").value="0";
            document.querySelector(".D_motor_value").value="0";
            document.querySelector(".Motor_value").value=0;
            document.querySelector(".Servo_value").value=0;
        }
    } else{
        console.log("Snapshot not available");
        document.querySelector(".P_servo_value").value="0";
        document.querySelector(".I_servo_value").value="0";
        document.querySelector(".D_servo_value").value="0";
        document.querySelector(".P_motor_value").value="0";
        document.querySelector(".I_motor_value").value="0";
        document.querySelector(".D_motor_value").value="0";
        document.querySelector(".Motor_value").value=0;
        document.querySelector(".Servo_value").value=0;
    }
  }).catch((error) => {
    console.error(error);
  })


// P,I,D,Motor,Servo inputs
document.querySelector(".enter").addEventListener("click", function(){
    var P_servo_value=document.querySelector(".P_servo_value").value;
    var I_servo_value=document.querySelector(".I_servo_value").value;
    var D_servo_value=document.querySelector(".D_servo_value").value;
    var motor_value=document.querySelector(".Motor_value").value;
    var servo_value=document.querySelector(".Servo_value").value;
    var P_motor_value=document.querySelector(".P_motor_value").value;
    var I_motor_value=document.querySelector(".I_motor_value").value;
    var D_motor_value=document.querySelector(".D_motor_value").value;

    // send values
    if (P_servo_value>=0 && I_servo_value>=0 && D_servo_value>=0 && motor_value>=0 && servo_value>=0){
        set(ref(db, "IDs/"+car_id), {
            P_servo: parseFloat(P_servo_value),
            I_servo: parseFloat(I_servo_value),
            D_servo: parseFloat(D_servo_value),
            P_motor: parseFloat(P_motor_value),
            I_motor: parseFloat(I_motor_value),
            D_motor: parseFloat(D_motor_value),
            Motor: parseInt(motor_value,10),
            Servo: parseInt(servo_value,10),
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
        },500);
    }
    

    // Update toggle value
    get(child(dbRef,"IDs/"+car_id)).then((snapshot) => {
        if (snapshot.exists()){
          const values_obj=snapshot.val();
          values_obj["Toggle"]=toggle;
          set(ref(db, "IDs/"+car_id), {
            P_servo: values_obj["P_servo"],
            I_servo: values_obj["I_servo"],
            D_servo: values_obj["D_servo"],
            P_motor: values_obj["P_motor"],
            I_motor: values_obj["I_motor"],
            D_motor: values_obj["D_motor"],
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

goToTraffic.addEventListener("click", function(){
    window.location.href = "traffic.html";
})

goToLogin.addEventListener("click", function(){
    window.location.href = "index.html";
})

