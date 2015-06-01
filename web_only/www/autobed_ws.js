var AutoBed = (function () {
    'use strict';

    var logTimer = null;
    var log = function (txt) {
        var logDiv =  document.getElementById('log');
        logDiv.innerHTML = txt;
        if (logTimer !== null) {
            clearTimeout(logTimer);
        }
//        logTimer = setTimeout( function () {logDiv.innerHTML = 'Ready';}, 3000);
    }

    var handleAJAXResponse = function (event) {
        if (this.readyState < 4) {
           console.log("State", this.readyState);
	} else if (this.readyState === 4) {
            console.log("AJAX Request Finished, response received: ", this.responseText);
            if (this.status === 200) {
                log( this.responseText );
            } else {
                log( "Error sending command." );
            }
        }
    };

    var sendOKFlag = false;
    var sendTimer = null;

    var sendAJAX = function (cmd) {
        var cmdTxt = "cmd="+cmd;
        var xhr;
        if (window.XMLHttpRequest) { //Modern Browsers, IE7+
            xhr = new XMLHttpRequest();
         } else { // IE5, IE6
            xhr = new ActiveXObject("Microsoft.XMLHTTP"); 
         }
         xhr.open("POST", "cgi-bin/autobed_controller.cgi", true);
         xhr.setRequestHeader("Content-type", "application/x-www-form-urlencoded; charset=UTF-8");
         xhr.onreadystatechange = handleAJAXResponse;
         xhr.send(cmdTxt);
    };

    var sendCmd = function (cmd) {
        if (sendOKFlag) {
            sendAJAX(cmd);
            sendTimer = setTimeout(function(){ sendCmd(cmd) }, 250)
        } else {
            clearTimeout(sendTimer);
        }
    }

    var setupButtonCBs = function (buttonElement) {
//        buttonElement.addEventListener("click", function (event) {
//            sendCmd(event.target.value);
//        });

        buttonElement.addEventListener("mouseup", function(event){
                    sendOKFlag = false;
                    clearTimeout(sendTimer);
                    });
        buttonElement.addEventListener("mouseout", function(event){
                    sendOKFlag = false;
                    clearTimeout(sendTimer);
                    });
        buttonElement.addEventListener("mouseleave", function(event){
                    sendOKFlag = false;
                    clearTimeout(sendTimer);
                    });
        buttonElement.addEventListener("mousedown", function(event) {
                    sendOKFlag = true;
                    sendCmd(event.target.value);
                    });
    };

    var onOpen = function () {
        setupButtonCBs(document.getElementById("head-up"));
        setupButtonCBs(document.getElementById("bed-up"));
        setupButtonCBs(document.getElementById("legs-up"));
        setupButtonCBs(document.getElementById("head-down"));
        setupButtonCBs(document.getElementById("bed-down"));
        setupButtonCBs(document.getElementById("legs-down"));
    }

    var onMessage = function (msg) {
        console.log("WS Received: ", msg);
    }

    var onClose = function () {
        console.log("Websocket closed by AutoBed server.");
    }
    
    var init = function () {
        if ("WebSocket" in window) {
        var ws = new WebSocket("ws://"+window.location.host+":8028");
        ws.onopen = onOpen;
        ws.onmessage = onMessage;
        ws.onclose = onClose;
            
        } else {
            alert("This browser does not support websockts, which are required for AutoBed operation.")
        }
        

    };

    return {init: init};

}());
