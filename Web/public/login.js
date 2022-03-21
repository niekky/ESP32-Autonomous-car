"use strict";
// Import the functions you need from the SDKs you need
import { initializeApp } from "https://www.gstatic.com/firebasejs/9.6.6/firebase-app.js";
import { getDatabase, ref, get, child } from "https://www.gstatic.com/firebasejs/9.6.6/firebase-database.js";
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

// Initialize Firebase
const app = initializeApp(firebaseConfig);
const db=getDatabase(app);
const dbRef=ref(db);

document.querySelector(".login").addEventListener("click", function(){
    var ID_name=document.querySelector(".ID").value;
    get(child(dbRef,"/IDs")).then((snapshot) => {
      if (snapshot.exists()){
        const id_list=Object.keys(snapshot.val());
        // console.log("DEBUG: ID_list: ",id_list);
        // console.log("DEBUG: ID_name: ",ID_name);
        if (id_list.includes(ID_name)){
          console.log("Login Successfully!");
          localStorage.setItem("storageName",ID_name);
          window.location.href = "controller.html";
        } else {
          console.log("Wrong ID!");
        }
      } else{
        console.log("Snapshot not available");
      }
    }).catch((error) => {
      console.error(error);
    })
})

