var AutoBed = (function () {
    'use strict';

    var logTimer = null;
    var log = function (txt, permanent) {
	permanent = (permanent === undefined) ? false : permanent;
        var logDiv =  document.getElementById('log');
        logDiv.innerHTML = txt;
        console.log(txt);
        if (logTimer !== null) {
            clearTimeout(logTimer);
        }
	if (!permanent) {
		logTimer = setTimeout( function () {logDiv.innerHTML = 'Ready';}, 3000);
	}
    }

    var sendOKFlag = false;
    var sendTimer = null;

    var sendCmd = function (cmd) {
        if (sendOKFlag) {
            ws.send(cmd.toString())
            sendTimer = setTimeout(function(){ sendCmd(cmd) }, 75)
        } else {
            clearTimeout(sendTimer);
        }
    }

    var setupButtonCBs = function (buttonElement) {
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
            if (event.which === 1) { // Only respond to left mouse button (right will stick button down)
	            sendOKFlag = true;
	            sendCmd(event.target.value);
	    }
	    });
    };
    var heartBeatTimer, lastBeatTime;
    var checkHeartBeat = function () {
        if (new Date() - lastBeatTime > 11000) {
            ws.close()
            ws.onclose()
	} else {
	    ws.send('----heartbeat----')
	}
    }

    var onOpen = function () {
	var disconElems = document.querySelectorAll('.disconnected')
	for (var i=0; i<disconElems.length; i+=1) {
            disconElems[i].className = disconElems[i].className.replace("disconnected", "connected")
 	}
	heartBeatTimer = setInterval(checkHeartBeat, 5000);
        setupButtonCBs(document.getElementById("head-up"));
        setupButtonCBs(document.getElementById("bed-up"));
        setupButtonCBs(document.getElementById("legs-up"));
        setupButtonCBs(document.getElementById("head-down"));
        setupButtonCBs(document.getElementById("bed-down"));
        setupButtonCBs(document.getElementById("legs-down"));
        log("Connected to AutoBed.")
    }

    var onMessage = function (msg) {
	if (msg.data === '----heartbeat----') {
            lastBeatTime = new Date();
        }
    }

    var onClose = function () {
	log("Connection to Autobed Controller Closed", true)
        clearTimeout(heartBeatTimer);
	var conElems = document.querySelectorAll('.connected')
	for (var i=0; i<conElems.length; i+=1) {
            conElems[i].className = conElems[i].className.replace("connected", "disconnected")
 	}
    }

    var ws;

    var init = function () {
        if ("WebSocket" in window) {
	    ws = new WebSocket("ws://"+window.location.hostname+":828");
	    ws.onopen = onOpen;
	    ws.onmessage = onMessage;
	    ws.onclose = onClose;
        } else {
            alert("This browser does not support websockts, which are required for AutoBed operation.")
        }
    };

    return {init: init};

}());
