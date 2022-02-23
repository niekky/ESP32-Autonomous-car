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


var n=0;
const circles = document.querySelectorAll('.circle')
let activeLight = 0;
const btn_red=document.querySelector(".red");
const btn_yellow=document.querySelector(".yellow");
const btn_green=document.querySelector(".green");
const btn_back=document.querySelector(".back");
const btn_auto=document.querySelector(".auto");


const app = initializeApp(firebaseConfig);
const db=getDatabase(app);
const dbRef=ref(db);


btn_red.addEventListener("click", function(){   

	activeLight = 0;
	circles[activeLight].className = 'circle';
	const currentLight = circles[activeLight]
	currentLight.classList.add(currentLight.getAttribute('color'));

    set(ref(db, "traffic"), {
        count: 0,
        light: "red",
        toggle: "red"
    }); 
})
btn_yellow.addEventListener("click", function(){   

	activeLight = 1;
	circles[activeLight].className = 'circle';
	const currentLight = circles[activeLight]
	currentLight.classList.add(currentLight.getAttribute('color'));

    set(ref(db, "traffic"), {
        count: 0,
        light: "yellow",
        toggle: "yellow"
    }); 
})
btn_green.addEventListener("click", function(){   

	activeLight = 2;
	circles[activeLight].className = 'circle';
	const currentLight = circles[activeLight]
	currentLight.classList.add(currentLight.getAttribute('color'));

    set(ref(db, "traffic"), {
        count: 0,
        light: "green",
        toggle: "green"
    }); 
})

// // // Toggle button
// // engine_status.addEventListener("click", function(){
// //     n=n+1;
// //     // OFF mode
// //     if (n%2==0){
// //         engine_status.innerText="OFF";
// //         toggle=false;

// //     // ON mode
// //     } else {
// //         engine_status.innerText="ON";
// //         toggle=true;
// //         // GRAPHING
// //         initializeGraph();
// //         var cnt=0;
// //         var realtime= setInterval(function(){
          
// //           if (toggle==false){
// //             clearInterval(realtime);
// //           }
// //         },20);
// //     }
    

//     // // Update toggle value
//     // get(child(dbRef,"IDs/"+car_id)).then((snapshot) => {
//     //     if (snapshot.exists()){
//     //       const values_obj=snapshot.val();
//     //       values_obj["Toggle"]=toggle;
//     //       set(ref(db, "IDs/"+car_id), {
//     //         P: values_obj["P"],
//     //         I: values_obj["I"],
//     //         D: values_obj["D"],
//     //         Motor: values_obj["Motor"],
//     //         Servo: values_obj["Servo"],
//     //         Toggle: values_obj["Toggle"]
//     //       });
        
//     //     } else{
//     //       console.log("Snapshot not available");
//     //     }
//     //   }).catch((error) => {
//     //     console.error(error);
//     //   })
//     // })


btn_back.addEventListener("click", function(){
    window.location.href = "controller.html";
})


// //====================================================================================================================

// const circles = document.querySelectorAll('.circle')
// let activeLight = 0;

// circles[activeLight].className = 'circle';
// const currentLight = circles[activeLight]
// currentLight.classList.add(currentLight.getAttribute('color'));
