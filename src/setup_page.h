const char setup_html[] PROGMEM = R"rawliteral(<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Grow box</title>
  <script src='https://code.jquery.com/jquery-2.2.4.js'></script>
</head>
<body>
  <p>Dehumidifier settings</p>
  <input type='number' id='lower' placeholder='lower humidity bound'></input><br><button id='setLower'>Set lower</button><br>
  <br><input type='number' id='higher' placeholder='higher humidity bound'></input><br><button id ='setHigher'>Set higher</button>
  <p id="currentvals"></p>
  <script>
              function setLowerBound(){
                var oReq = new XMLHttpRequest();
                oReq.open("GET", "/lowBound?int="+document.getElementById("lower").value);
                oReq.send();
              }  

              function setHigherBound(){
                var oReq = new XMLHttpRequest();
                oReq.open("GET", "/highBound?int="+document.getElementById("higher").value);
                oReq.send();
              } 
              document.getElementById("setLower").onclick = function() {setLowerBound()};
              document.getElementById("setHigher").onclick = function() {setHigherBound()};


              function getEnviron(){
                      var oReq = new XMLHttpRequest();
                      oReq.addEventListener("load", environDisplay);
                      oReq.open("GET", "/environ");
                      oReq.send();
               }
              function environDisplay(){
                console.log(this.responseText)
                 document.getElementById('currentvals').innerHTML = this.responseText;
              }

              function sleep(milliseconds) {
                var start = new Date().getTime();
                for (var i = 0; i < 1e7; i++) {
                  if ((new Date().getTime() - start) > milliseconds){
                    break;
                  }
                }
              }

              sleep(2000);
              setInterval(getEnviron, 5000);
              getEnviron();
    </script>
  </body>  
</html>)rawliteral";