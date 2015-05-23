var AutoBed = (function () {
    'use strict';

    var logTimer = null;
    var log = function (txt) {
        var logDiv =  document.getElementById('log');
        logDiv.innerHTML = txt;
        if (logTimer !== null) {
            clearTimeout(logTimer);
        }
        logTimer = setTimeout( function () {logDiv.innerHTML = 'Ready';}, 3000);
    }

    var handleAJAXResponse = function (event) {
        if (this.readyState === 4) {
//        switch (this.readyState) {
//            case 0:
//                console.log("AJAX Request not initialized");
//                break;
//            case 1:
//                console.log("AJAX: Server Connection Established");
//                break;
//            case 2:
//                console.log("AJAX Request Received");
//                break;
//            case 3:
//                console.log("AJAX Request being processed");
//                break;
//            case 4:
            console.log("AJAX Request Finished, response received");
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

    var init = function () {
        setupButtonCBs(document.getElementById("head-up"));
        setupButtonCBs(document.getElementById("bed-up"));
        setupButtonCBs(document.getElementById("legs-up"));
        setupButtonCBs(document.getElementById("head-down"));
        setupButtonCBs(document.getElementById("bed-down"));
        setupButtonCBs(document.getElementById("legs-down"));
    };

    return {init: init};

}());

AutoBed.sendDiffCmd = function (string) {
    if (AutoBed.send) {
        AutoBed.controlsPub.publish({'data':string});
        AutoBed.diffTimer = setTimeout(function () {
                AutoBed.sendDiffCmd(string)}
                , 250)
    } else {
        clearTimeout(AutoBed.diffTimer);
    }
};
