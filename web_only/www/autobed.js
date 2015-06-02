var AutoBed = (function () {
    'use strict';

    var logTimer = null;
    var log = function (txt, permanent) {
	permanent = (permanent === undefined) ? false : permanent;
        var logDiv =  document.getElementById('log');
        logDiv.innerHTML = txt;
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
        log("Connected to AutoBed.")
    }

    var onMessage = function (msg) {
        console.log("WS Received: ", msg);
    }

    var onClose = function () {
        console.log("Cannot connect to Autobed Server");
	log("Cannot connect to Autobed Server.", true)
    }

    var ws;

    var init = function () {
        if ("WebSocket" in window) {
	    ws = new WebSocket("ws://"+window.location.host+":828");
	    ws.onopen = onOpen;
	    ws.onmessage = onMessage;
	    ws.onclose = onClose;
        } else {
            alert("This browser does not support websockts, which are required for AutoBed operation.")
        }
    };

    return {init: init};

}());
