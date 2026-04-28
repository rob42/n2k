// Get current sensor readings when the page loads  
window.addEventListener('load', getReadings);

var labelSignalK = document.getElementById('signalk-data');
var labelZenoh = document.getElementById('zenoh-data');

// Function to get current readings on the webpage when it loads for the first time
function getReadings(){
  var xhr = new XMLHttpRequest();
  xhr.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var myObj = JSON.parse(this.responseText);
      console.log(myObj);
      updateHtml(myObj);
      
    }
  }; 
  xhr.open("GET", "/readings", true);
  xhr.send();
}

function updateHtml(myObj){
  // var n2k = myObj.signalk;
  //   var zenoh = myObj.zenoh;
  //   labelN2k.textContent = n2k;
  //   labelZenoh.textContent = zenoh;
  labelSignalK.textContent = JSON.stringify(myObj,null,2);
}

if (!!window.EventSource) {
  var source = new EventSource('/events');
  
  source.addEventListener('open', function(e) {
    console.log("Events Connected");
  }, false);

  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  }, false);
  
  source.addEventListener('message', function(e) {
    console.log("message", e.data);
  }, false);
  
  source.addEventListener('new_readings', function(e) {
    console.log("new_readings", e.data);
    var myObj = JSON.parse(e.data);
    console.log(myObj);
    updateHtml(myObj);
  }, false);
}
//++++++++++++++++menus below here +++++++++++++//
 
// Select the two elements that will be clickable
const toggleButton = document.getElementById("button-menu");
const navWrapper = document.getElementById("nav");
const navitems = document.getElementById("nav-items");

/* 
  Whenever the button is clicked, 
  add and remove the necessary classes 
  to display the menu.
*/
toggleButton.addEventListener("click", () => {
  //get the latest list of other nodes
  var xhr = new XMLHttpRequest();
  xhr.onload = function() {
  if (xhr.status >= 200 && xhr.status < 300) {
      console.log(this.responseText);
      var links = JSON.parse(this.responseText);
      console.log(links);
      navitems.innerHTML = '';
      if(links != null){
        links.forEach(function(link) { 
          navitems.innerHTML += '<a class="link-item" href="http://'+link+'.local" >'+link+'</a>';
        });
      }
      toggleButton.classList.toggle("close");
      navWrapper.classList.toggle("show");
    }
  }; 
  xhr.open("GET", "/menu", true);
  xhr.send();
  
});

/* 
  When a click occurs outside the link container, 
  the menu should hide.
*/
navWrapper.addEventListener("click", e => {
  if (e.target.id === "nav") {
    navWrapper.classList.remove("show");
    toggleButton.classList.remove("close");
  }
});